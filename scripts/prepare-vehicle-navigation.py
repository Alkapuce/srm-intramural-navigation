#!/usr/bin/env python3
"""Build a local experimental map from real registered scans; leave vehicle motion locked."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
import rosbag2_py
import yaml
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation
from intramural_mission.proximity import lidar_to_body
from internal.occupancy import fit_floor, raycast_grid

p = argparse.ArgumentParser(description=__doc__)
p.add_argument("--bag", required=True)
p.add_argument("--vehicle-config", required=True)
p.add_argument("--output", required=True)
p.add_argument("--arena-config", help="Measured field configuration in the map frame; omission keeps experimental geometry")
args = p.parse_args()
root = Path(__file__).resolve().parents[1]
out, vehicle, bag = map(lambda s: Path(s).resolve(), (args.output, args.vehicle_config, args.bag))
if not any(out.is_relative_to(root / d) for d in ("cache", "artifacts")) or out.exists():
    raise ValueError("Choose a new output under project cache or artifacts")
for name in ("handoff.json", "point-lio.yaml", "base-vehicle.json"):
    if not (vehicle/name).is_file():
        raise ValueError("Missing vehicle configuration: " + name)
reference = json.loads((root/"config/vehicle-srm-reference.json").read_text())["mid360"]
arena = json.loads((root/"config/arena.json").read_text())
if args.arena_config:
    arena = json.loads(Path(args.arena_config).read_text())
    from intramural_mission.route import segments, clearance
    route = segments(arena)
    if any(min(clearance(arena,point)) < .05 for _,points in route for point in points):
        raise ValueError("Measured route lacks pole/boundary clearance")
arena["allow_bypass"] = False
arena["lidar_pose"] = reference["base_to_lidar_xyz_m"] + reference["base_to_lidar_rpy_rad"]
arena["assumptions"] = "Experimental lab map; route geometry is unmeasured and mission start is locked"
reader = rosbag2_py.SequentialReader()
reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3"), rosbag2_py.ConverterOptions("", ""))
cloud_topic = "/intramural_robot/cloud_registered"
odom_topic = "/intramural_robot/aft_mapped_to_init"
reader.set_filter(rosbag2_py.StorageFilter(topics=[cloud_topic, odom_topic]))
lio = yaml.safe_load((vehicle/"point-lio.yaml").read_text())["/**"]["ros__parameters"]["mapping"]
if lio["extrinsic_est_en"]:
    raise ValueError("Ray origins require fixed recorded lidar/IMU extrinsics")
extrinsic_t = np.asarray(lio["extrinsic_T"], dtype=float)
poses, stamps = {}, []
clouds = []
while reader.has_next():
    topic, raw, timestamp = reader.read_next()
    if topic == odom_topic:
        m = deserialize_message(raw, Odometry)
        if m.header.frame_id != "camera_init":
            raise ValueError("Expected camera_init odometry for ray origins")
        p0, q = m.pose.pose.position, m.pose.pose.orientation
        sensor = Rotation.from_quat([q.x,q.y,q.z,q.w]).apply(extrinsic_t) + [p0.x,p0.y,p0.z]
        if not np.isfinite(sensor).all():
            raise ValueError("Nonfinite lidar ray origin")
        poses[(m.header.stamp.sec,m.header.stamp.nanosec)] = sensor
        continue
    m = deserialize_message(raw, PointCloud2)
    if m.header.frame_id != "camera_init" or m.is_bigendian:
        raise ValueError("Expected real Point-LIO camera_init cloud")
    fields = {f.name:f for f in m.fields}
    if any(fields[k].datatype != 7 for k in "xyz"):
        raise ValueError("Expected float32 XYZ")
    dtype = np.dtype({"names":list("xyz"), "formats":["<f4"]*3,
                      "offsets":[fields[k].offset for k in "xyz"], "itemsize":m.point_step})
    a = np.ndarray((m.height,m.width), dtype=dtype, buffer=m.data, strides=(m.row_step,m.point_step)).reshape(-1)
    xyz = np.stack([a[k] for k in "xyz"], axis=1)
    clouds.append(xyz[np.isfinite(xyz).all(axis=1) & (np.linalg.norm(xyz,axis=1)<12)])
    stamps.append((m.header.stamp.sec,m.header.stamp.nanosec))
missing = sum(s not in poses for s in stamps)
if missing > max(1,len(clouds)//100):
    raise ValueError("Registered scans lack timestamp-matched odometry; cannot reconstruct ray origins")
clouds = [cloud for cloud,stamp in zip(clouds,stamps) if stamp in poses]
sensor_origins = np.asarray([poses[s] for s in stamps if s in poses])
if len(clouds)<10:
    raise ValueError("Insufficient real registered scans")
xyz = np.concatenate(clouds)
_, indices = np.unique(np.floor(xyz/.05).astype(np.int32), axis=0, return_index=True)
xyz = xyz[indices].astype("<f4")
if len(xyz)<500:
    raise ValueError("Insufficient real map points")
body = lidar_to_body(xyz, arena["lidar_pose"])
res = .05
origin = np.floor(body[:,:2].min(axis=0)-1)
size = np.ceil((body[:,:2].max(axis=0)+1-origin)/res).astype(int)
if max(size)>1000:
    raise ValueError("Map exceeds experimental bounds")
sensor_origins = lidar_to_body(sensor_origins, arena["lidar_pose"])
plane, floor_report = fit_floor(body, sensor_origins)
pixels, occupancy_report = raycast_grid(
    [lidar_to_body(cloud,arena["lidar_pose"]) for cloud in clouds],
    sensor_origins, plane, origin, size, res, root)
occupancy_report.update(floor_plane=floor_report, unmatched_scans_skipped=missing)
out.mkdir(parents=True)
(out/"occupancy.json").write_text(json.dumps(occupancy_report,indent=2)+"\n")
header = f"VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH {len(xyz)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(xyz)}\nDATA binary\n"
(out/"prior.pcd").write_bytes(header.encode()+xyz.tobytes())
(out/"map.pgm").write_bytes(f"P5\n{size[0]} {size[1]}\n255\n".encode()+pixels[::-1].tobytes())
(out/"map.yaml").write_text(yaml.safe_dump({"image":"map.pgm","resolution":res,"origin":[float(origin[0]),float(origin[1]),0.0],"negate":0,"occupied_thresh":.65,"free_thresh":.2}))
(out/"arena.json").write_text(json.dumps(arena,indent=2)+"\n")
params = yaml.safe_load((root/"config/stage2-nav.yaml").read_text())
names = ["loam_interface","sensor_scan_generation","small_gicp_relocalization","terrain_analysis","terrain_analysis_ext","controller_server","local_costmap","map_server","velocity_smoother"]
params = {k:params[k] for k in names}
def configure(value):
    if isinstance(value,dict):
        for k,v in value.items():
            if k=="use_sim_time":value[k]=False
            elif k in ("base_frame","robot_base_frame"):value[k]="base_footprint"
            elif k in ("lidar_frame","sensor_frame"):value[k]="livox_frame"
            elif k=="topic" and isinstance(v,str):value[k]=v.replace("<robot_namespace>","/"+arena["namespace"])
            else:configure(v)
    elif isinstance(value,list):
        for v in value:configure(v)
configure(params)
params["small_gicp_relocalization"]["ros__parameters"]["prior_pcd_file"]=str(out/"prior.pcd")
params["map_server"]["ros__parameters"]["yaml_filename"]=str(out/"map.yaml")
controller=params["controller_server"]["ros__parameters"]
controller["min_y_velocity_threshold"]=.001
controller["FollowPath"].update(enable_rotation=False,v_angular_min=0.0,v_angular_max=0.0,v_linear_min=-.2,v_linear_max=.2,translation_ki=0.0)
cost=params["local_costmap"]["local_costmap"]["ros__parameters"]
cost.update(robot_radius=arena["robot_radius"],always_send_full_costmap=True)
smooth=params["velocity_smoother"]["ros__parameters"]
smooth.update(max_velocity=[.2,.2,0.0],min_velocity=[-.2,-.2,0.0])
(out/"nav.yaml").write_text(yaml.safe_dump(params,sort_keys=False))
manifest={"purpose":"Real recorded map; motion remains locked pending field and vehicle acceptance","source_bag":str(bag),"registered_scans":len(clouds),"map_points":len(xyz),"vehicle_config":str(vehicle),"field_geometry_source":str(Path(args.arena_config).resolve()) if args.arena_config else None,"field_geometry_verified":False,"obstacle_policy":"stop_and_wait","motion_permitted":False,"start_permitted":False,"unknown_space_preserved":True,"files_sha256":{f.name:hashlib.sha256(f.read_bytes()).hexdigest() for f in out.iterdir()}}
(out/"manifest.json").write_text(json.dumps(manifest,indent=2)+"\n")
print(json.dumps(manifest,indent=2))
