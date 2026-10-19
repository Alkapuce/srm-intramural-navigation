"""Real navigation; standby by default, explicit accepted map/vehicle required for motion."""
import json
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml
from intramural_base.runtime import shutdown_before_peers


def start(context):
    root = Path(__file__).resolve().parents[1]
    folder = Path(LaunchConfiguration("navigation_config").perform(context)).resolve()
    manifest = json.loads((folder/"manifest.json").read_text())
    if manifest["motion_permitted"] or manifest["start_permitted"]:
        raise ValueError("This entry is restricted to vehicle standby")
    c = json.loads((folder/"arena.json").read_text())
    trajectory = c.get("navigation_engine")=="trajectory"
    motion = LaunchConfiguration("enable_motion").perform(context).lower()=="true"
    vehicle = Path(LaunchConfiguration("vehicle_config").perform(context)).resolve()
    device = LaunchConfiguration("serial_device").perform(context)
    if motion:
        from intramural_base.transport import validate_vehicle_config, CommandPolicy
        base = json.loads((vehicle/"base-vehicle.json").read_text())
        validate_vehicle_config(base)
        CommandPolicy(base["body_to_protocol"], base["max_body_speed_mps"], base["max_body_acceleration_mps2"], base["max_protocol_axis"], base["input_timeout_seconds"])
        if not manifest.get("field_geometry_verified"):
            raise ValueError("Confirm actual parking/pole geometry in this map before enabling motion")
        if not 0 < c["max_speed"] <= min(1.5, base["max_body_speed_mps"]):
            raise ValueError("Real mission speed must be positive and no greater than 1.5 and vehicle limit")
        if c.get('continuous_corners', False) and not (
            c.get('raw_collision_follow_path') and 0 < c.get('lateral_acceleration',0) <= c['max_acceleration']
            and 0 < c.get('braking_acceleration',0) <= c['max_acceleration']
        ):
            raise ValueError("Continuous corners require ordered path and positive lateral/braking acceleration bounds")
        if trajectory and c['max_acceleration']>base['max_body_acceleration_mps2']:
            raise ValueError('Profile acceleration exceeds the configured vehicle interface limit')
        if not device.startswith("/dev/"):
            raise ValueError("Explicit serial device under /dev required for motion")
    sim = LaunchConfiguration("use_sim_time").perform(context).lower()=="true"
    params = ParameterFile(RewrittenYaml(source_file=str(folder/"nav.yaml"), root_key=c["namespace"],
        param_rewrites={"use_sim_time":str(sim).lower()}, convert_types=True))
    specs = [
        ("loam_interface","loam_interface_node","loam_interface"),
        ("sensor_scan_generation","sensor_scan_generation_node","sensor_scan_generation"),
        ("small_gicp_relocalization","small_gicp_relocalization_node","small_gicp_relocalization"),
        ("terrain_analysis","terrainAnalysis","terrain_analysis"),
        ("terrain_analysis_ext","terrainAnalysisExt","terrain_analysis_ext"),
        ("nav2_map_server","map_server","map_server"),
        ("nav2_controller","controller_server","controller_server"),
        ("nav2_velocity_smoother","velocity_smoother","velocity_smoother"),
    ]
    nodes=[]
    for package,exe,name in specs:
        remaps=[]
        if name=="controller_server":remaps=[("cmd_vel","legacy_cmd_vel" if trajectory else "cmd_vel_controller")]
        elif name=="velocity_smoother":remaps=[("cmd_vel","legacy_cmd_vel" if trajectory else "cmd_vel_controller"),("cmd_vel_smoothed","cmd_vel_nav2_result")]
        nodes.append(Node(package=package,executable=exe,name=name,parameters=[params,{"use_sim_time":sim}],remappings=remaps,output="screen"))
    nodes += [
        Node(package="intramural_mission",executable="livox_adapter",parameters=[{"use_sim_time":sim}],output="screen"),
        Node(package="intramural_mission",executable="adapter",parameters=[{"use_sim_time":sim,"localization":"sensors","motion_permitted":motion,"arena_config":str(folder/"arena.json")}],remappings=[("livox/imu","/livox/imu")]+([("cmd_vel_nav2_result","cmd_vel_controller")]if trajectory else []),output="screen"),
        Node(package="intramural_mission",executable="trajectory_mission" if trajectory else "mission",parameters=[{"use_sim_time":sim,"start_permitted":motion,"arena_config":str(folder/"arena.json"),"variant":c.get("mission_variant","figure8")}],output="screen"),
        Node(package="nav2_lifecycle_manager",executable="lifecycle_manager",name="lifecycle_manager_vehicle",
             parameters=[{"use_sim_time":sim,"autostart":motion,"node_names":["map_server","controller_server","velocity_smoother"]}],output="screen"),
    ]
    serial = None
    if motion:
        serial = Node(package="intramural_base", executable="serial_bridge", name="serial_bridge",
            parameters=[{"use_sim_time":False,"simulation":False,"device":device,"calibration":str(vehicle/"base-vehicle.json")}], output="screen")
        nodes.append(serial)
    pose=c["lidar_pose"]
    static=Node(package="tf2_ros",executable="static_transform_publisher",name="vehicle_lidar_mount",
        arguments=["--x",str(pose[0]),"--y",str(pose[1]),"--z",str(pose[2]),"--roll",str(pose[3]),"--pitch",str(pose[4]),"--yaw",str(pose[5]),"--frame-id","base_footprint","--child-frame-id","livox_frame"],
        parameters=[{"use_sim_time":sim}],output="screen")
    return [static,GroupAction([PushRosNamespace(c["namespace"]),*nodes])] + ([shutdown_before_peers(serial)] if serial is not None else [])


def generate_launch_description():
    def failed(event,context):
        if event.returncode:raise RuntimeError(f"Vehicle navigation child failed: {event.returncode}")
        return []
    return LaunchDescription([DeclareLaunchArgument("navigation_config"),DeclareLaunchArgument("use_sim_time",default_value="false"),
        DeclareLaunchArgument("enable_motion",default_value="false"),
        DeclareLaunchArgument("vehicle_config",default_value=str(Path(__file__).resolve().parents[1]/"cache/vehicle-current")),
        DeclareLaunchArgument("serial_device",default_value=""),
        RegisterEventHandler(OnProcessExit(on_exit=failed)),OpaqueFunction(function=start)])
