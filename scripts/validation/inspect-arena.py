#!/usr/bin/env python3
"""Check actual TF publishers, localization provenance, clocks and control ownership."""

import argparse, json, subprocess, time
from pathlib import Path
import rclpy
from rcl_interfaces.srv import GetParameters

p = argparse.ArgumentParser()
p.add_argument("--output", required=True)
p.add_argument("--mode", choices=["ideal", "sensors"], required=True)
p.add_argument("--transport", choices=["direct", "serial-sim"], default="direct")
args = p.parse_args()
out = Path(args.output)
c = json.loads(Path("/work/config/arena.json").read_text())
ns = "/" + c["namespace"]
source = Path("/work/scripts/validation/tf-authorities.cpp")
binary = Path("/work/cache/stage2/tf-authorities")
binary.parent.mkdir(parents=True, exist_ok=True)
if not binary.exists() or binary.stat().st_mtime < source.stat().st_mtime:
    includes = [
        "-I" + str(p) for p in Path("/opt/ros/humble/include").iterdir() if p.is_dir()
    ]
    subprocess.run(
        [
            "g++",
            "-std=c++17",
            str(source),
            *includes,
            "-L/opt/ros/humble/lib",
            "-Wl,-rpath,/opt/ros/humble/lib",
            "-lrclcpp",
            "-ltracetools",
            "-lrosidl_runtime_c",
            "-llibstatistics_collector",
            "-lstatistics_msgs__rosidl_typesupport_cpp",
            "-lrcutils",
            "-lrmw",
            "-lrcl",
            "-ltf2_msgs__rosidl_typesupport_cpp",
            "-o",
            str(binary),
        ],
        check=True,
    )
captured = subprocess.check_output([str(binary), ns], text=True, timeout=15)
(out / "tf-authorities.tsv").write_text(captured)
authorities = {}
edges = {}
for row in captured.splitlines():
    f = row.split("\t")
    if f[0] == "PUB":
        authorities[f[1]] = f[2] + "/" + f[3]
    if f[0] == "TF":
        edges.setdefault(f[1] + "->" + f[2], set()).add(f[3])
if not edges or any(len(gids) != 1 for gids in edges.values()):
    raise RuntimeError("No TF or duplicate actual TF publisher")
if any(gid not in authorities for gids in edges.values() for gid in gids):
    raise RuntimeError("Unresolved TF publisher")
actual = {edge: authorities[next(iter(gids))] for edge, gids in edges.items()}
expected = {
    "map->odom": ns
    + ("/motion_guard" if args.mode == "ideal" else "/small_gicp_relocalization"),
    "odom->base_footprint": ns
    + ("/motion_guard" if args.mode == "ideal" else "/sensor_scan_generation"),
}
if any(actual.get(edge) != name for edge, name in expected.items()):
    raise RuntimeError(f"Localization provenance mismatch: {actual}")
rclpy.init()
node = rclpy.create_node("arena_graph_verifier")
try:
    clocks = {}
    required = [
        "controller_server",
        "velocity_smoother",
        "map_server",
        "motion_guard",
        "mission",
        "terrain_analysis",
        "terrain_analysis_ext",
    ]
    if args.mode == "sensors":
        required += [
            "point_lio",
            "small_gicp_relocalization",
            "loam_interface",
            "sensor_scan_generation",
            "lidar_adapter",
        ]
    for name in required:
        cli = node.create_client(GetParameters, ns + "/" + name + "/get_parameters")
        if not cli.wait_for_service(timeout_sec=3):
            raise RuntimeError("Missing node " + name)
        req = GetParameters.Request()
        req.names = ["use_sim_time"]
        future = cli.call_async(req)
        rclpy.spin_until_future_complete(node, future, timeout_sec=3)
        if not future.done() or not future.result():
            raise RuntimeError("No parameter response " + name)
        clocks[name] = future.result().values[0].bool_value
        node.destroy_client(cli)
    pointlio_time_unit = None
    if args.mode == "sensors":
        cli = node.create_client(GetParameters, ns + "/point_lio/get_parameters")
        req = GetParameters.Request()
        req.names = ["preprocess.timestamp_unit"]
        future = cli.call_async(req)
        rclpy.spin_until_future_complete(node, future, timeout_sec=3)
        if not future.done() or not future.result():
            raise RuntimeError("No Point-LIO timestamp unit response")
        if future.result().values[0].type != 2:
            raise RuntimeError("Missing integer Point-LIO timestamp unit")
        pointlio_time_unit = future.result().values[0].integer_value
        if pointlio_time_unit != 0:
            raise RuntimeError("Point-LIO point time must be seconds")
        node.destroy_client(cli)
    if not all(clocks.values()):
        raise RuntimeError("Simulation clocks mismatch")
    # DDS endpoint discovery can precede its ROS node-name metadata.
    deadline = time.monotonic() + 5
    expected_nodes = {(name, ns) for name in required}
    while time.monotonic() < deadline:
        nodes = set(node.get_node_names_and_namespaces())
        if expected_nodes <= nodes:
            break
        rclpy.spin_once(node, timeout_sec=0.05)
    if not expected_nodes <= nodes:
        raise RuntimeError(
            "Incomplete actual node graph: " + str(expected_nodes - nodes)
        )
    final_publishers = [
        p.node_name for p in node.get_publishers_info_by_topic(ns + "/cmd_vel")
    ]
    if final_publishers != ["motion_guard"]:
        raise RuntimeError(
            "Competing final velocity publisher " + str(final_publishers)
        )
    transport_publishers = [p.node_name for p in node.get_publishers_info_by_topic(ns + "/cmd_vel_transport")]
    direct_bridge_readers = [p.node_name for p in node.get_subscriptions_info_by_topic(ns + "/cmd_vel") if p.node_name == "ros_gz_bridge"]
    if args.transport == "serial-sim" and (transport_publishers != ["sim_mcu"] or direct_bridge_readers):
        raise RuntimeError("Serial simulation bypass or competing transport publisher")
    if args.transport == "direct" and transport_publishers:
        raise RuntimeError("Unexpected simulated MCU in direct mode")
    truth_readers = [
        p.node_name
        for p in node.get_subscriptions_info_by_topic(ns + "/chassis_odometry_gt")
    ]
    if args.mode == "sensors" and any(
        name not in ("arena_verifier",) for name in truth_readers
    ):
        raise RuntimeError("Sensor navigation reads truth: " + str(truth_readers))
    deadline = time.monotonic() + 10
    while True:
        terrain_readers = [(p.node_namespace, p.node_name)
            for p in node.get_subscriptions_info_by_topic(ns + "/terrain_map")]
        if (ns + '/local_costmap', 'local_costmap') in terrain_readers or time.monotonic() >= deadline:
            break
        rclpy.spin_once(node, timeout_sec=.1)
    if (ns + "/local_costmap", "local_costmap") not in terrain_readers:
        raise RuntimeError(
            "Local costmap is not connected to actual terrain topic: "
            + str(terrain_readers)
        )
    report = {
        "mode": args.mode,
        "pointlio_timestamp_unit": pointlio_time_unit,
        "tf_authorities": actual,
        "use_sim_time": clocks,
        "final_velocity_publishers": final_publishers,
        "transport_velocity_publishers": transport_publishers,
        "ground_truth_readers": truth_readers,
        "terrain_readers": terrain_readers,
        "nodes": sorted(nodes),
        "topics": node.get_topic_names_and_types(),
        "services": node.get_service_names_and_types(),
    }
    (out / "interfaces.json").write_text(json.dumps(report, indent=2) + "\n")
    print("PASS TF authorities, clocks, control ownership and localization provenance")
finally:
    node.destroy_node()
    rclpy.shutdown()
