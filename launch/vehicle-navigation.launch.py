"""Real sensor navigation standby. No serial process and no permitted motion."""
import json
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def start(context):
    root = Path(__file__).resolve().parents[1]
    folder = Path(LaunchConfiguration("navigation_config").perform(context)).resolve()
    manifest = json.loads((folder/"manifest.json").read_text())
    if manifest["motion_permitted"] or manifest["start_permitted"]:
        raise ValueError("This entry is restricted to vehicle standby")
    c = json.loads((folder/"arena.json").read_text())
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
        if name=="controller_server":remaps=[("cmd_vel","cmd_vel_controller")]
        elif name=="velocity_smoother":remaps=[("cmd_vel","cmd_vel_controller"),("cmd_vel_smoothed","cmd_vel_nav2_result")]
        nodes.append(Node(package=package,executable=exe,name=name,parameters=[params,{"use_sim_time":sim}],remappings=remaps,output="screen"))
    nodes += [
        Node(package="intramural_mission",executable="livox_adapter",parameters=[{"use_sim_time":sim}],output="screen"),
        Node(package="intramural_mission",executable="adapter",parameters=[{"use_sim_time":sim,"localization":"sensors","motion_permitted":False,"arena_config":str(folder/"arena.json")}],remappings=[("livox/imu","/livox/imu")],output="screen"),
        Node(package="intramural_mission",executable="mission",parameters=[{"use_sim_time":sim,"start_permitted":False,"arena_config":str(folder/"arena.json")}],output="screen"),
        Node(package="nav2_lifecycle_manager",executable="lifecycle_manager",name="lifecycle_manager_vehicle",
             parameters=[{"use_sim_time":sim,"autostart":False,"node_names":["map_server","controller_server","velocity_smoother"]}],output="screen"),
    ]
    pose=c["lidar_pose"]
    static=Node(package="tf2_ros",executable="static_transform_publisher",name="vehicle_lidar_mount",
        arguments=["--x",str(pose[0]),"--y",str(pose[1]),"--z",str(pose[2]),"--roll",str(pose[3]),"--pitch",str(pose[4]),"--yaw",str(pose[5]),"--frame-id","base_footprint","--child-frame-id","livox_frame"],
        parameters=[{"use_sim_time":sim}],output="screen")
    return [static,GroupAction([PushRosNamespace(c["namespace"]),*nodes])]


def generate_launch_description():
    def failed(event,context):
        if event.returncode:raise RuntimeError(f"Vehicle navigation child failed: {event.returncode}")
        return []
    return LaunchDescription([DeclareLaunchArgument("navigation_config"),DeclareLaunchArgument("use_sim_time",default_value="false"),
        RegisterEventHandler(OnProcessExit(on_exit=failed)),OpaqueFunction(function=start)])
