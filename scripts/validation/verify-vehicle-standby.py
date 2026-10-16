#!/usr/bin/env python3
"""Activate real navigation without goals, verify TF/data and locked zero output."""
import argparse
import json
import subprocess
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from lifecycle_msgs.srv import GetState
from nav2_msgs.srv import ManageLifecycleNodes
from nav_msgs.msg import Odometry, OccupancyGrid
from rcl_interfaces.srv import GetParameters
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

p=argparse.ArgumentParser(description=__doc__)
p.add_argument("--output",required=True)
p.add_argument("--map-config",help="Also check live scan alignment against this generated map")
args=p.parse_args()
out=Path(args.output).resolve();out.mkdir(parents=True,exist_ok=True)
rclpy.init();n=rclpy.create_node("vehicle_standby_verifier")
tf=Buffer(node=n);listener=TransformListener(tf,n)
counts={};latest={};speeds=[];edges={};subscriptions=[]
ns="/intramural_robot"
def collect(topic,msg):
    counts[topic]=counts.get(topic,0)+1;latest[topic]=msg
def command(msg):
    speeds.append([msg.linear.x,msg.linear.y,msg.linear.z,msg.angular.x,msg.angular.y,msg.angular.z])
def call(typ,name,request,timeout=10):
    cli=n.create_client(typ,name)
    try:
        if not cli.wait_for_service(timeout_sec=timeout):raise RuntimeError("Missing service "+name)
        future=cli.call_async(request);rclpy.spin_until_future_complete(n,future,timeout_sec=timeout)
        if not future.done() or future.result() is None:raise RuntimeError("No response from "+name)
        return future.result()
    finally:n.destroy_client(cli)
try:
    root=Path(__file__).resolve().parents[2]
    source=root/"scripts/validation/tf-authorities.cpp"
    binary=root/"cache/stage2/tf-authorities"
    binary.parent.mkdir(parents=True,exist_ok=True)
    if not binary.exists() or binary.stat().st_mtime<source.stat().st_mtime:
        includes=["-I"+str(d) for d in Path("/opt/ros/humble/include").iterdir() if d.is_dir()]
        subprocess.run(["g++","-std=c++17",str(source),*includes,"-L/opt/ros/humble/lib","-Wl,-rpath,/opt/ros/humble/lib","-lrclcpp","-ltracetools","-lrosidl_runtime_c","-llibstatistics_collector","-lstatistics_msgs__rosidl_typesupport_cpp","-lrcutils","-lrmw","-lrcl","-ltf2_msgs__rosidl_typesupport_cpp","-o",str(binary)],check=True)
    captured=subprocess.check_output([str(binary),""],text=True,timeout=10)
    (out/"tf-authorities.tsv").write_text(captured)
    for row in captured.splitlines():
        f=row.split("\t")
        if f[0]=="TF":edges.setdefault(f[1]+"->"+f[2],set()).add(f[3])
    for topic,typ in [("odometry",Odometry),("livox/lidar",PointCloud2),("registered_scan",PointCloud2),("terrain_map",PointCloud2),("terrain_map_ext",PointCloud2)]:
        subscriptions.append(n.create_subscription(typ,ns+"/"+topic,lambda msg,t=topic:collect(t,msg),qos_profile_sensor_data))
    latched=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    for topic,typ in [("mission/status",String),("motion_guard/status",String),("local_costmap/costmap",OccupancyGrid),("map",OccupancyGrid)]:
        subscriptions.append(n.create_subscription(typ,ns+"/"+topic,lambda msg,t=topic:collect(t,msg),latched if topic!="motion_guard/status" else 10))
    subscriptions.append(n.create_subscription(Twist,ns+"/cmd_vel",command,10))
    states={name:call(GetState,ns+"/"+name+"/get_state",GetState.Request()).current_state.label for name in ["map_server","controller_server","velocity_smoother"]}
    if not all(s=="active" for s in states.values()):
        req=ManageLifecycleNodes.Request();req.command=0
        startup=call(ManageLifecycleNodes,ns+"/lifecycle_manager_vehicle/manage_nodes",req,30)
        if not startup.success:raise RuntimeError("Vehicle lifecycle startup failed")
    deadline=time.monotonic()+12
    while time.monotonic()<deadline:rclpy.spin_once(n,timeout_sec=.1)
    required=["odometry","livox/lidar","registered_scan","terrain_map","terrain_map_ext","mission/status","motion_guard/status","local_costmap/costmap","map"]
    if any(counts.get(t,0)<1 for t in required):raise RuntimeError("Missing live data "+str(counts))
    if len(speeds)<100 or any(any(v!=0.0 for v in msg) for msg in speeds):raise RuntimeError("Nonzero or missing standby velocity")
    m=json.loads(latest["mission/status"].data);g=json.loads(latest["motion_guard/status"].data)
    if m["state"]!="IDLE" or m["start_permitted"] or m["ready"] or g["motion_permitted"]:raise RuntimeError("Standby interlock open")
    attempt=call(Trigger,ns+"/mission/start",Trigger.Request())
    if attempt.success:raise RuntimeError("Standby unexpectedly accepted mission start")
    states={name:call(GetState,ns+"/"+name+"/get_state",GetState.Request()).current_state.label for name in ["map_server","controller_server","velocity_smoother"]}
    if any(s!="active" for s in states.values()):raise RuntimeError("Inactive lifecycle "+str(states))
    clocks={}
    for name in ["loam_interface","sensor_scan_generation","small_gicp_relocalization","terrain_analysis","terrain_analysis_ext","controller_server","velocity_smoother","motion_guard","mission","livox_raw_adapter"]:
        q=GetParameters.Request();q.names=["use_sim_time"]
        clocks[name]=call(GetParameters,ns+"/"+name+"/get_parameters",q).values[0].bool_value
    if any(clocks.values()):raise RuntimeError("Real navigation uses a simulation clock")
    transform=tf.lookup_transform("map","base_footprint",rclpy.time.Time())
    age=n.get_clock().now().nanoseconds*1e-9-transform.header.stamp.sec-transform.header.stamp.nanosec*1e-9
    if abs(age)>.3:raise RuntimeError("Stale map/base TF: "+str(age))
    if not all(e in edges for e in ["map->odom","odom->base_footprint"]) or any(len(v)!=1 for v in edges.values()):raise RuntimeError("Missing or competing TF authorities")
    publishers=n.get_publishers_info_by_topic(ns+"/cmd_vel")
    if [x.node_name for x in publishers]!=["motion_guard"]:raise RuntimeError("Competing final velocity publisher")
    readers=[x.node_name for x in n.get_subscriptions_info_by_topic(ns+"/cmd_vel")]
    if readers!=[n.get_name()]:raise RuntimeError("Standby velocity unexpectedly has a transport subscriber: "+str(readers))
    report={"lifecycle":states,"samples":counts,"zero_velocity_samples":len(speeds),"mission_start_rejected":not attempt.success,"mission_start_response":attempt.message,"mission_state":m["state"],"motion_permitted":g["motion_permitted"],"use_sim_time":clocks,"map_to_base_age_s":age,"tf_authorities":{k:sorted(v) for k,v in edges.items()},"map_to_base_translation":[transform.transform.translation.x,transform.transform.translation.y,transform.transform.translation.z],"raw_lidar_frame":latest["livox/lidar"].header.frame_id,"odometry_frame":latest["odometry"].header.frame_id,"physical_serial_opened":False,"field_geometry_verified":False,"independent_precision_test":False}
    if args.map_config:
        import numpy as np
        from scipy.spatial import cKDTree
        from scipy.spatial.transform import Rotation
        from intramural_mission.proximity import lidar_to_body
        folder=Path(args.map_config).resolve()
        arena=json.loads((folder/"arena.json").read_text())
        header,raw=(folder/"prior.pcd").read_bytes().split(b"DATA binary\n",1)
        if b"FIELDS x y z\n" not in header:raise RuntimeError("Expected generated XYZ prior")
        target=lidar_to_body(np.frombuffer(raw,dtype="<f4").reshape(-1,3),arena["lidar_pose"])
        scan=latest["registered_scan"]
        if scan.header.frame_id!="odom" or scan.is_bigendian:raise RuntimeError("Unexpected registered scan frame/format")
        fields={f.name:f for f in scan.fields}
        if any(fields[k].datatype!=7 for k in "xyz"):raise RuntimeError("Expected float32 XYZ")
        source=np.stack([np.ndarray((scan.height,scan.width),dtype="<f4",buffer=scan.data,
                         offset=fields[k].offset,strides=(scan.row_step,scan.point_step)).reshape(-1) for k in "xyz"],axis=1)
        source=source[np.isfinite(source).all(axis=1)][::3]
        transform=tf.lookup_transform("map","odom",rclpy.time.Time()).transform
        q=transform.rotation;t=transform.translation
        aligned=Rotation.from_quat([q.x,q.y,q.z,q.w]).apply(source)+[t.x,t.y,t.z]
        source_mask=np.linalg.norm(aligned,axis=1)<11
        source=source[source_mask];aligned=aligned[source_mask]
        if len(aligned)<100:raise RuntimeError("Insufficient overlapping scan points")
        tree=cKDTree(target)
        before=np.quantile(tree.query(source)[0],[.5,.9,.99]).tolist()
        after=np.quantile(tree.query(aligned)[0],[.5,.9,.99]).tolist()
        registration={"source_points":len(aligned),"map_points":len(target),
            "nearest_map_distance_before_m":before,"nearest_map_distance_after_m":after,
            "limits":"Local scan/map consistency after restart; not independent pose precision or arbitrary global relocalization"}
        (out/"registration-residual.json").write_text(json.dumps(registration,indent=2)+"\n")
        if after[0]>.1 or after[1]>.3:raise RuntimeError("Live scan is not aligned with the recorded map")
        report["registration"]=registration
    (out/"standby-results.json").write_text(json.dumps(report,indent=2)+"\n");print(json.dumps(report,indent=2))
finally:n.destroy_node();rclpy.shutdown()
