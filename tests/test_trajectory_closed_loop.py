"""Production mission -> guard -> delayed plant -> TF, with a person stop and restart."""
from array import array
import json
from pathlib import Path
import shutil
import tempfile
import time
import unittest

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import Imu, PointCloud2, PointField
from std_srvs.srv import Trigger
from intramural_mission.adapter import Adapter
from intramural_mission.trajectory_mission import TrajectoryMission
from intramural_mission.response import Plant

ROOT=Path(__file__).resolve().parents[1]


class TrajectoryClosedLoopTests(unittest.TestCase):
    def test_complete_ordered_route_person_removal_and_terminal_stop(self):
        c=json.loads((ROOT/'config/arena.json').read_text())
        c.update(json.loads((ROOT/'config/navigation-profiles.json').read_text())['safe'])
        c.update(lower_pole=[1.,0.],upper_pole=[1.,1.4],loop_radius=.7,parking=[-.5,-.5],
                 bounds=[-2.,4.,-2.,4.],lidar_pose=[0.]*6,optimize_route=False,max_speed=.65)
        with tempfile.TemporaryDirectory(dir=ROOT/'cache') as folder:
            file=Path(folder)/'arena.json';file.write_text(json.dumps(c))
            rclpy.init(domain_id=87,args=['--ros-args','-p',f'arena_config:={file}',
                                        '-r','cmd_vel_nav2_result:=cmd_vel_controller'])
            mission=TrajectoryMission();guard=Adapter();peer=rclpy.create_node('closed_loop_sensors')
            executor=SingleThreadedExecutor()
            for node in (mission,guard,peer):executor.add_node(node)
            ground=peer.create_publisher(PointCloud2,'livox/lidar',10)
            imu=peer.create_publisher(Imu,'livox/imu',10)
            odom=peer.create_publisher(Odometry,'chassis_odometry_gt',10)
            qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
            maps=peer.create_publisher(OccupancyGrid,'map',qos)
            costs=peer.create_publisher(OccupancyGrid,'local_costmap/costmap',qos)
            grid=OccupancyGrid();grid.header.frame_id='map';grid.info.width=grid.info.height=120
            grid.info.resolution=.05;grid.info.origin.position.x=grid.info.origin.position.y=-2.
            grid.info.origin.orientation.w=1.;grid.data=[0]*14400
            client=peer.create_client(Trigger,'mission/start')
            sent=np.zeros(2)
            def command(msg):
                nonlocal sent
                sent=np.array([msg.linear.x,msg.linear.y])
            peer.create_subscription(Twist,'cmd_vel',command,10)
            plant=Plant(c['parking'],.05,.05);request=None;person=None;blocked=False;resumed=False
            began=time.monotonic();previous=began;deadline=began
            try:
                while time.monotonic()-began<90:
                    now=time.monotonic();plant.dt=min(.05,max(.005,now-previous));previous=now
                    plant.step(sent)
                    msg=Odometry();msg.header.stamp=peer.get_clock().now().to_msg()
                    msg.pose.pose.position.x,msg.pose.pose.position.y=map(float,plant.pose)
                    msg.pose.pose.orientation.w=1.;odom.publish(msg);imu.publish(Imu())
                    maps.publish(grid);costs.publish(grid)
                    if person is None and mission.state=='RUNNING' and mission.detail.get('s',0.)>.8:
                        person=now
                    xyz=[[1.,0.,0.]]
                    if person is not None and now-person<1.:
                        norm=np.linalg.norm(sent)
                        if norm>.05 and not blocked:person_point=plant.pose+sent/norm*.45
                        if 'person_point' in locals():
                            xy=person_point-plant.pose;xyz=[[xy[0]+dx,xy[1],.3] for dx in (-.01,0.,.01)]
                    cloud=PointCloud2(height=1,width=len(xyz),point_step=12,row_step=12*len(xyz))
                    cloud.header=msg.header;cloud.fields=[PointField(name=k,offset=4*i,datatype=7,count=1) for i,k in enumerate('xyz')]
                    cloud.data=array('B',np.asarray(xyz,dtype='<f4').tobytes());ground.publish(cloud)
                    for _ in range(35):executor.spin_once(timeout_sec=0.)
                    if request is None and mission.ready() and client.service_is_ready():request=client.call_async(Trigger.Request())
                    blocked=blocked or mission.state=='BLOCKED'
                    resumed=resumed or (blocked and mission.state=='RUNNING')
                    if mission.state in ('DONE','FAULT'):break
                    deadline+=.025;time.sleep(max(0.,deadline-time.monotonic()))
                self.assertIsNotNone(request)
                self.assertTrue(request.done() and request.result().success)
                self.assertTrue(blocked and resumed,(blocked,resumed,mission.reason))
                self.assertEqual(mission.state,'DONE',(mission.reason,mission.detail))
                self.assertEqual(len(mission.history),10)
                self.assertEqual(mission.blocked_count,1)
                self.assertLess(np.linalg.norm(plant.pose-c['parking']),c['parking_tolerance'])
                self.assertEqual(guard.output,[0.,0.])
            finally:
                mission.stop();guard.stop()
                if hasattr(mission,'log'):mission.log.close()
                if mission.run_id:
                    evidence=ROOT/'artifacts/tests/trajectory-closed-loop'/mission.run_id
                    evidence.parent.mkdir(parents=True,exist_ok=True)
                    shutil.copytree(ROOT/'artifacts/navigation-runs'/mission.run_id,evidence)
                    shutil.rmtree(ROOT/'artifacts/navigation-runs'/mission.run_id)
                executor.shutdown()
                for node in (mission,guard,peer):node.destroy_node()
                rclpy.shutdown()


if __name__=='__main__':unittest.main()
