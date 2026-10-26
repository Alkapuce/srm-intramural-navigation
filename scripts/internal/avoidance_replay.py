"""Real-time raw Livox replay; recorded TF/odometry are never published into this graph."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import numpy as np
import yaml


def raw_chain(root, session, out, vehicle):
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import TransformStamped, Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage
    from tf2_ros import StaticTransformBroadcaster
    from scipy.spatial.transform import Rotation
    from intramural_mission.livox_adapter import LivoxAdapter
    from intramural_mission.adapter import Adapter
    from ament_index_python.packages import get_package_prefix
    config=json.loads((session/'map/arena.json').read_text());config['allow_bypass']=False
    config_file=out/'arena.json';config_file.write_text(json.dumps(config)+'\n')
    rclpy.init(domain_id=92,args=['--ros-args','-p','use_sim_time:=true','-p','localization:=sensors',
        '-p','motion_permitted:=false','-p',f'arena_config:={config_file}',
        '-r','__ns:=/intramural_robot','-r','livox/imu:=/livox/imu'])
    peer=rclpy.create_node('raw_replay_observer');converter=LivoxAdapter();guard=Adapter()
    executor=SingleThreadedExecutor()
    for node in (peer,converter,guard):executor.add_node(node)
    static=StaticTransformBroadcaster(peer)
    t=TransformStamped();t.header.frame_id='base_footprint';t.child_frame_id='livox_frame'
    t.header.stamp=peer.get_clock().now().to_msg()
    xyzrpy=config['lidar_pose'];t.transform.translation.x,t.transform.translation.y,t.transform.translation.z=map(float,xyzrpy[:3])
    q=Rotation.from_euler('xyz',xyzrpy[3:]).as_quat()
    t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w=map(float,q)
    static.sendTransform(t)
    counts=dict(odometry=0,registered_scan=0,converted_scan=0,map_transforms=0,serial_frames=0)
    poses=[];nonzero=[]
    def odometry(message):
        p=message.pose.pose.position;counts['odometry']+=1
        poses.append([message.header.stamp.sec+message.header.stamp.nanosec/1e9,p.x,p.y])
    def tf(message):counts['map_transforms']+=sum(t.header.frame_id=='map' and t.child_frame_id=='odom' for t in message.transforms)
    def serial(message):counts['serial_frames']=max(counts['serial_frames'],json.loads(message.data)['frames'])
    peer.create_subscription(Odometry,'odometry',odometry,qos_profile_sensor_data)
    for topic,key in [('registered_scan','registered_scan'),('livox/lidar','converted_scan')]:
        peer.create_subscription(PointCloud2,topic,lambda m,k=key:counts.__setitem__(k,counts[k]+1),qos_profile_sensor_data)
    peer.create_subscription(TFMessage,'/tf',tf,qos_profile_sensor_data)
    peer.create_subscription(String,'base/sim_status',serial,10)
    peer.create_subscription(Twist,'cmd_vel_transport',lambda m:nonzero.append([m.linear.x,m.linear.y]) if m.linear.x or m.linear.y else None,10)
    processes=[];logs=[];env=dict(os.environ,ROS_DOMAIN_ID='92')
    def start(name,command):
        file=(out/(name+'.log')).open('w');logs.append(file)
        p=subprocess.Popen(command,env=env,stdout=file,stderr=subprocess.STDOUT,start_new_session=True);processes.append(p)
        return p
    def ros_node(package,executable,name,parameters):
        file=out/(name+'.yaml');file.write_text(yaml.safe_dump({'/**':{'ros__parameters':dict(parameters,use_sim_time=True)}}))
        executable=str(Path(get_package_prefix(package))/'lib'/package/executable)
        return start(name,[executable,'--ros-args','-r','__ns:=/intramural_robot',
            '-r','__node:='+name,'--params-file',str(file)])
    params=yaml.safe_load((session/'map/nav.yaml').read_text())
    source=yaml.safe_load((vehicle/'point-lio.yaml').read_text())['/**']['ros__parameters']
    ros_node('point_lio','pointlio_mapping','point_lio',source)
    for package,exe,name in [('loam_interface','loam_interface_node','loam_interface'),
                             ('sensor_scan_generation','sensor_scan_generation_node','sensor_scan_generation'),
                             ('small_gicp_relocalization','small_gicp_relocalization_node','small_gicp_relocalization')]:
        values=params[name]['ros__parameters'].copy()
        if name=='small_gicp_relocalization':values['prior_pcd_file']=str(session/'map/prior.pcd')
        ros_node(package,exe,name,values)
    device=out/'sim-tty'
    start('sim_mcu',[str(Path(get_package_prefix('intramural_base'))/'lib/intramural_base/sim_mcu'),'--device-file',str(device),
        '--ros-args','-r','__ns:=/intramural_robot'])
    failure=None;exits=[]
    try:
        due=time.monotonic()+8
        while time.monotonic()<due:
            executor.spin_once(timeout_sec=.02)
            if any(p.poll() is not None for p in processes):raise RuntimeError('Replay child exited at startup')
        if not device.exists():raise RuntimeError('Private pseudo-terminal unavailable')
        ros_node('intramural_base','serial_bridge','serial_bridge',dict(simulation=True,device_file=str(device),
            calibration=str(root/'config/base-sim.json')))
        player=start('bag_play',['ros2','bag','play',str(session/'bag'),'--clock','--rate','1.0',
            '--delay','1.0','--topics','/livox/lidar','/livox/imu'])
        deadline=time.monotonic()+90
        while player.poll() is None and time.monotonic()<deadline:
            executor.spin_once(timeout_sec=.01)
            if any(p.poll() is not None for p in processes[:-1]):raise RuntimeError('Replay processing child exited')
        if player.poll() is None:raise RuntimeError('Raw bag playback timed out')
        if player.returncode:raise RuntimeError('Raw bag player failed')
        drain=time.monotonic()+1
        while time.monotonic()<drain:executor.spin_once(timeout_sec=.01)
        if counts['odometry']<10 or counts['converted_scan']<10 or counts['registered_scan']<10 or counts['map_transforms']<5:
            raise RuntimeError('Insufficient real processing output: '+str(counts))
        if counts['serial_frames']<100 or nonzero:raise RuntimeError('Standby serial path did not remain zero')
    except Exception as error:failure=str(error)
    finally:
        guard.stop()
        # Serial exits before the simulator and localization publishers.
        ordering=[p for p in processes if Path(p.args[0]).name=='serial_bridge']
        ordering += [p for p in reversed(processes) if p not in ordering]
        for p in ordering:
            if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
            try:p.wait(timeout=15)
            except subprocess.TimeoutExpired:
                p.kill();p.wait();failure=failure or 'Replay child forced exit'
            exits.append(dict(command=p.args[:4],code=p.returncode))
            if p.returncode:failure=failure or 'Replay child abnormal exit'
        executor.shutdown()
        for node in (guard,converter,peer):node.destroy_node()
        rclpy.shutdown()
        for file in logs:file.close()
    report=dict(passed=failure is None,failure=failure,counts=counts,exits=exits,nonzero_transport=len(nonzero),
        scope='1x raw Livox + IMU through Point-LIO, small_gicp and private zero-command PTY; no recorded TF/odometry injection')
    (out/'raw-chain.json').write_text(json.dumps(report,indent=2)+'\n')
    (out/'raw-odometry.json').write_text(json.dumps(poses)+'\n')
    return report
