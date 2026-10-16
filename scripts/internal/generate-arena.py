#!/usr/bin/env python3
"""Generate the organizer robot, Gazebo arena, grid and analytic prior cloud."""

import json, math, hashlib, os
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory
from xmacro.xmacro4sdf import XMLMacro4sdf
from intramural_mission.route import segments, clearance
from organizer_model import build_robot

root = Path(__file__).resolve().parents[2]
c = json.loads((root / "config/arena.json").read_text())
out = root / "simulation/generated"
out.mkdir(parents=True, exist_ok=True)
for name, val in c.items():
    if isinstance(val, (int, float)) and (not math.isfinite(val) or val <= 0):
        raise ValueError(name)
for phase, path in segments(c):
    if any(min(clearance(c, p)) < 0.1 for p in path):
        raise ValueError("Reference route too close to pole or boundary")
ns = c["namespace"]
transport = os.environ.get("INTRAMURAL_TRANSPORT", "direct")
if transport not in ("direct", "serial-sim"):
    raise ValueError("Unknown arena transport")
resources = Path(get_package_share_directory("rmu_gazebo_simulator"))
world = ET.parse(resources / "resource/worlds/empty_world.sdf").getroot()
w = world.find("world")
w.find("plugin[@name='ignition::gazebo::systems::Sensors']/render_engine").text = (
    "ogre" if Path("/dev/dxg").exists() and os.environ.get("LIBGL_ALWAYS_SOFTWARE", "0") != "1" else "ogre2"
)
ET.SubElement(w, "physics", {"name": "physics", "type": "ignored"}).append(
    ET.fromstring("<max_step_size>0.005</max_step_size>")
)


def box(name, x, y, z, sizes, color=".3 .3 .3 1", static=True):
    a = ET.fromstring(
        f'<model name="{name}"><static>{str(static).lower()}</static><pose>{x} {y} {z} 0 0 0</pose><link name="link"><collision name="collision"><geometry><box><size>{" ".join(map(str, sizes))}</size></box></geometry></collision><visual name="visual"><geometry><box><size>{" ".join(map(str, sizes))}</size></box></geometry><material><ambient>{color}</ambient><diffuse>{color}</diffuse></material></visual></link></model>'
    )
    w.append(a)
    return a


xmin, xmax, ymin, ymax = c["bounds"]
boxes = [
    ("west", xmin - 0.05, (ymin + ymax) / 2, (0.1, ymax - ymin + 0.2, 1.5)),
    ("east", xmax + 0.05, (ymin + ymax) / 2, (0.1, ymax - ymin + 0.2, 1.5)),
    ("south", (xmin + xmax) / 2, ymin - 0.05, (xmax - xmin, 0.1, 1.5)),
    ("north", (xmin + xmax) / 2, ymax + 0.05, (xmax - xmin, 0.1, 1.5)),
]
for name, x, y, size in boxes:
    box("boundary_" + name, x, y, 0.75, size)
for name in ("lower_pole", "upper_pole"):
    x, y = c[name]
    r = c["pole_radius"]
    h = c["pole_height"]
    w.append(
        ET.fromstring(
            f'<model name="{name}"><static>true</static><pose>{x} {y} {h / 2} 0 0 0</pose><link name="link"><collision name="collision"><geometry><cylinder><radius>{r}</radius><length>{h}</length></cylinder></geometry></collision><visual name="visual"><geometry><cylinder><radius>{r}</radius><length>{h}</length></cylinder></geometry><material><ambient>1 .5 .1 1</ambient><diffuse>1 .5 .1 1</diffuse></material></visual></link></model>'
        )
    )
box(
    "movable_obstacle",
    *c["obstacle_initial"],
    c["obstacle_size"][2] / 2,
    c["obstacle_size"],
    color=".8 .1 .1 1",
)
# Painted parking and crossing marks have no collision geometry.
for name, p, size in [
    ("parking", c["parking"], (c["robot_length"] + 0.2, c["robot_width"] + 0.2, 0.002)),
    (
        "crossing",
        [
            (c["lower_pole"][0] + c["upper_pole"][0]) / 2,
            (c["lower_pole"][1] + c["upper_pole"][1]) / 2,
        ],
        (0.15, 0.15, 0.002),
    ),
]:
    mark = box(name, *p, 0.002, size, color=".1 .8 .2 1")
    mark.find("link").remove(mark.find("link/collision"))
ET.indent(world)
ET.ElementTree(world).write(out / "arena.sdf", encoding="unicode", xml_declaration=True)
# Public sensor definitions supplement the organizer's original geometry.
x = XMLMacro4sdf()
x.set_xml_file(
    str(
        Path(get_package_share_directory("pb2025_robot_description"))
        / "resource/xmacro/simulation_robot.sdf.xmacro"
    )
)
x.generate({"global_initial_color": "red"})
source = ET.fromstring(x.to_string()).find("model")
robot, description = build_robot(root, c, source, out)
(out / "organizer-model.json").write_text(json.dumps(description, indent=2) + "\n")
model = robot.find("model")
model.append(
    ET.fromstring(
        f'<plugin filename="ignition-gazebo-velocity-control-system" name="ignition::gazebo::systems::VelocityControl"><topic>/{ns}/cmd_vel</topic></plugin>'
    )
)
model.append(
    ET.fromstring(
        f'<plugin filename="ignition-gazebo-odometry-publisher-system" name="ignition::gazebo::systems::OdometryPublisher"><odom_frame>world</odom_frame><robot_base_frame>base_footprint</robot_base_frame><odom_topic>/{ns}/odometry_gt</odom_topic><odom_publish_frequency>50</odom_publish_frequency><dimensions>2</dimensions></plugin>'
    )
)
ET.indent(robot)
ET.ElementTree(robot).write(out / "robot.sdf", encoding="unicode", xml_declaration=True)
# ROS bridges expose truth for validation; sensor mode never uses it for localization.
bridges = []
for rt, gt, typ, gtyp, direction in [
    (
        "chassis_odometry_gt",
        f"/{ns}/odometry_gt",
        "nav_msgs/msg/Odometry",
        "ignition.msgs.Odometry",
        "GZ_TO_ROS",
    ),
    (
        "cmd_vel_transport" if transport == "serial-sim" else "cmd_vel",
        f"/{ns}/cmd_vel",
        "geometry_msgs/msg/Twist",
        "ignition.msgs.Twist",
        "ROS_TO_GZ",
    ),
    (
        "livox/lidar",
        f"/world/default/model/{ns}/link/front_mid360/sensor/front_mid360_lidar/scan/points",
        "sensor_msgs/msg/PointCloud2",
        "ignition.msgs.PointCloudPacked",
        "GZ_TO_ROS",
    ),
    (
        "livox/imu",
        f"/world/default/model/{ns}/link/front_mid360/sensor/front_mid360_imu/imu",
        "sensor_msgs/msg/Imu",
        "ignition.msgs.IMU",
        "GZ_TO_ROS",
    ),
]:
    bridges.append(
        dict(
            ros_topic_name=f"/{ns}/" + rt,
            gz_topic_name=gt,
            ros_type_name=typ,
            gz_type_name=gtyp,
            direction=direction,
        )
    )
bridges.append(
    dict(
        ros_topic_name="/clock",
        gz_topic_name="/clock",
        ros_type_name="rosgraph_msgs/msg/Clock",
        gz_type_name="ignition.msgs.Clock",
        direction="GZ_TO_ROS",
    )
)
(out / "bridge.yaml").write_text(yaml.safe_dump(bridges))
res = c["map_resolution"]
width = math.ceil((xmax - xmin) / res) + 2
height = math.ceil((ymax - ymin) / res) + 2
origin = [xmin - res, ymin - res, 0.0]
image = np.full((height, width), 254, dtype=np.uint8)
for row in range(height):
    for col in range(width):
        p = [origin[0] + (col + 0.5) * res, origin[1] + (height - row - 0.5) * res]
        if not (xmin < p[0] < xmax and ymin < p[1] < ymax) or any(
            math.dist(p, c[k]) < c["pole_radius"] for k in ("lower_pole", "upper_pole")
        ):
            image[row, col] = 0
(out / "map.pgm").write_bytes(f"P5\n{width} {height}\n255\n".encode() + image.tobytes())
(out / "map.yaml").write_text(
    yaml.safe_dump(
        dict(
            image="map.pgm",
            resolution=res,
            origin=origin,
            negate=0,
            occupied_thresh=0.65,
            free_thresh=0.25,
            mode="trinary",
        )
    )
)
# Analytic geometry prior, including stored obstacle; in startup lidar axes.
points = []
for xval in np.arange(xmin, xmax, 0.1):
    for yval in np.arange(ymin, ymax, 0.1):
        points.append([xval, yval, 0.0])
for _, xval, yval, size in boxes + [
    ("obstacle", *c["obstacle_initial"], tuple(c["obstacle_size"]))
]:
    for t in np.arange(-size[0] / 2, size[0] / 2 + 0.05, 0.08):
        for z in np.arange(0, size[2], 0.08):
            points.extend(
                [[xval + t, yval - size[1] / 2, z], [xval + t, yval + size[1] / 2, z]]
            )
    for t in np.arange(-size[1] / 2, size[1] / 2 + 0.05, 0.08):
        for z in np.arange(0, size[2], 0.08):
            points.extend(
                [[xval - size[0] / 2, yval + t, z], [xval + size[0] / 2, yval + t, z]]
            )
for k in ("lower_pole", "upper_pole"):
    for angle in np.arange(0, 2 * math.pi, 0.12):
        for z in np.arange(0, c["pole_height"], 0.05):
            points.append(
                [
                    c[k][0] + c["pole_radius"] * math.cos(angle),
                    c[k][1] + c["pole_radius"] * math.sin(angle),
                    z,
                ]
            )
px, py, pz, roll, pitch, yaw = c["lidar_pose"]
cr, sr, cp, sp, cy, sy = (
    math.cos(roll),
    math.sin(roll),
    math.cos(pitch),
    math.sin(pitch),
    math.cos(yaw),
    math.sin(yaw),
)
R = np.array(
    [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]
)
pts = ((np.array(points) - np.array([px, py, pz])) @ R).astype("<f4")
header = f"# Analytic arena prior in initial lidar coordinates\nVERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH {len(pts)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(pts)}\nDATA binary\n"
(out / "prior.pcd").write_bytes(header.encode() + pts.tobytes())
# Controller and localization parameters derived from the fixed public baseline.
p = yaml.safe_load((root / "config/stage2-nav.yaml").read_text())
for name in ["loam_interface", "sensor_scan_generation", "small_gicp_relocalization"]:
    a = p[name]["ros__parameters"]
    for k in ["base_frame", "robot_base_frame"]:
        a[k] = "base_footprint"
for k in ["gravity", "gravity_init"]:
    p["point_lio"]["ros__parameters"]["mapping"][k] = [0.0, 0.0, -9.81]
p["lidar_adapter"] = {"ros__parameters": {"use_sim_time": True}}
p["point_lio"]["ros__parameters"]["preprocess"]["timestamp_unit"] = 0
p["small_gicp_relocalization"]["ros__parameters"]["prior_pcd_file"] = str(
    out / "prior.pcd"
)
p["map_server"]["ros__parameters"]["yaml_filename"] = str(out / "map.yaml")
a = p["controller_server"]["ros__parameters"]
a["min_y_velocity_threshold"] = 0.001
a["failure_tolerance"] = 0.0
a["progress_checker"]["required_movement_radius"] = 0.1
a["progress_checker"]["movement_time_allowance"] = 30.0
a["general_goal_checker"]["xy_goal_tolerance"] = 0.06
f = a["FollowPath"]
f.update(
    enable_rotation=False,
    v_angular_min=0.0,
    v_angular_max=0.0,
    v_linear_min=-c["max_speed"],
    v_linear_max=c["max_speed"],
    translation_ki=0.0,
    lookahead_dist=0.18,
    use_velocity_scaled_lookahead_dist=False,
    min_lookahead_dist=0.18,
    max_lookahead_dist=0.18,
    min_approach_linear_velocity=0.05,
    approach_velocity_scaling_dist=0.2,
    use_interpolation=True,
    reduction_ratio_at_high_curvature=0.9,
)
for k in ["local_costmap", "global_costmap"]:
    a = p[k][k]["ros__parameters"]
    a["robot_base_frame"] = "base_footprint"
    a["robot_radius"] = c["robot_radius"]
    a["inflation_layer"]["inflation_radius"] = 0.6
    a["always_send_full_costmap"] = True
    a["plugins"] = ["static_layer", "intensity_voxel_layer", "inflation_layer"]
    a["intensity_voxel_layer"][
        "terrain_map" if k == "local_costmap" else "terrain_map_ext"
    ]["topic"] = f"/{ns}/" + (
        "terrain_map" if k == "local_costmap" else "terrain_map_ext"
    )
a = p["velocity_smoother"]["ros__parameters"]
a["max_velocity"] = [c["max_speed"], c["max_speed"], 0.0]
a["min_velocity"] = [-c["max_speed"], -c["max_speed"], 0.0]
a["max_accel"] = [c["max_acceleration"]] * 2 + [1.0]
a["max_decel"] = [-c["max_acceleration"]] * 2 + [-1.0]
active_parameters = [
    "lidar_adapter",
    "point_lio",
    "loam_interface",
    "sensor_scan_generation",
    "terrain_analysis",
    "terrain_analysis_ext",
    "small_gicp_relocalization",
    "controller_server",
    "local_costmap",
    "map_server",
    "velocity_smoother",
]
(out / "nav.yaml").write_text(
    yaml.safe_dump({k: p[k] for k in active_parameters}, sort_keys=False)
)
(out / "route.json").write_text(json.dumps(segments(c), indent=2) + "\n")
(out / "manifest.json").write_text(
    json.dumps(
        {
            "configuration": c,
            "files_sha256": {
                f.name: hashlib.sha256(f.read_bytes()).hexdigest()
                for f in out.iterdir()
                if f.is_file() and f.name != "manifest.json"
            },
        },
        indent=2,
    )
    + "\n"
)
print(f"Generated organizer sentry and arena; {len(pts)} analytic prior points")
