"""Same controller and mission for ideal and sensor localization modes."""

import os, json
from pathlib import Path
from launch import LaunchDescription
from launch.actions import GroupAction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml
from intramural_base.runtime import shutdown_before_peers


def generate_launch_description():
    root = Path(__file__).resolve().parents[1]
    c = json.loads((root / "config/arena.json").read_text())
    ns = c["namespace"]
    trajectory = c.get("navigation_engine")=="trajectory"
    mode = os.environ.get("INTRAMURAL_LOCALIZATION", "ideal")
    transport = os.environ.get("INTRAMURAL_TRANSPORT", "direct")
    if mode not in ("ideal", "sensors"):
        raise ValueError(mode)
    if transport not in ("direct", "serial-sim"):
        raise ValueError(transport)
    params = ParameterFile(
        RewrittenYaml(
            source_file=str(root / "simulation/generated/nav.yaml"),
            root_key=ns,
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )

    def failed(event, context):
        if event.returncode:
            raise RuntimeError(f"Arena navigation child failed: {event.returncode}")
        return []

    specs = [
        ("nav2_map_server", "map_server", "map_server"),
        ("nav2_controller", "controller_server", "controller_server"),
        ("nav2_velocity_smoother", "velocity_smoother", "velocity_smoother"),
        ("terrain_analysis", "terrainAnalysis", "terrain_analysis"),
        ("terrain_analysis_ext", "terrainAnalysisExt", "terrain_analysis_ext"),
    ]
    if mode == "sensors":
        specs += [
            (
                "intramural_mission",
                "lidar_adapter",
                "lidar_adapter",
            ),
            ("point_lio", "pointlio_mapping", "point_lio"),
            (
                "small_gicp_relocalization",
                "small_gicp_relocalization_node",
                "small_gicp_relocalization",
            ),
            ("loam_interface", "loam_interface_node", "loam_interface"),
            (
                "sensor_scan_generation",
                "sensor_scan_generation_node",
                "sensor_scan_generation",
            ),
        ]
    nodes = []
    for package, executable, name in specs:
        remaps = []
        if name == "controller_server":
            remaps = [("cmd_vel", "legacy_cmd_vel" if trajectory else "cmd_vel_controller")]
        if name == "velocity_smoother":
            remaps = [
                ("cmd_vel", "legacy_cmd_vel" if trajectory else "cmd_vel_controller"),
                ("cmd_vel_smoothed", "cmd_vel_nav2_result"),
            ]
        nodes.append(
            Node(
                package=package,
                executable=executable,
                name=name,
                parameters=[params, {"use_sim_time": True}],
                remappings=remaps,
                output="screen",
            )
        )
    nodes.extend(
        [
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_arena",
                parameters=[
                    {
                        "use_sim_time": True,
                        "autostart": False,
                        "node_names": [
                            "map_server",
                            "controller_server",
                            "velocity_smoother",
                        ],
                    }
                ],
                output="screen",
            ),
            Node(
                package="intramural_mission",
                executable="adapter",
                remappings=[("cmd_vel_nav2_result","cmd_vel_controller")] if trajectory else [],
                parameters=[{"use_sim_time": True, "localization": mode, "arena_config": str(root / "config/arena.json")}],
                output="screen",
            ),
            Node(
                package="intramural_mission",
                executable="trajectory_mission" if trajectory else "mission",
                parameters=[
                    {
                        "use_sim_time": True,
                        "arena_config": str(root / "config/arena.json"),
                        "variant": os.environ.get(
                            "INTRAMURAL_MISSION_VARIANT", "figure8"
                        ),
                    }
                ],
                output="screen",
            ),
        ]
    )
    serial = None
    if transport == "serial-sim":
        serial = Node(package="intramural_base", executable="serial_bridge",
            name="serial_bridge", parameters=[{"simulation": True,
                "device_file": str(root / "cache/runtime/intramural-sim-tty"),
                "calibration": str(root / "simulation/generated/base-sim.json")}], output="screen")
        nodes.append(serial)
    return LaunchDescription(
        [
            RegisterEventHandler(OnProcessExit(on_exit=failed)),
            GroupAction(
                [
                    PushRosNamespace(ns),
                    SetRemap("/tf", "tf"),
                    SetRemap("/tf_static", "tf_static"),
                    *nodes,
                ]
            ),
        ] + ([shutdown_before_peers(serial)] if serial is not None else [])
    )
