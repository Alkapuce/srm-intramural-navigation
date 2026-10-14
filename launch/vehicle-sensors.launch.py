"""Real Livox and Point-LIO input diagnostics; no base or motion nodes."""

from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start(context):
    folder = Path(LaunchConfiguration("vehicle_config").perform(context)).resolve()
    valid = False
    if Path("/work/cache").exists() and folder.is_relative_to(Path("/work/cache")):
        valid = True
    elif "cache" in folder.parts or "artifacts" in folder.parts:
        valid = True
    if not valid:
        raise ValueError("Vehicle runtime parameters must be under cache or artifacts directory")
    for name in ("livox-driver.yaml", "point-lio.yaml", "handoff.json"):
        if not (folder / name).is_file():
            raise ValueError("Missing generated vehicle configuration: " + name)
    return [
        Node(package="livox_ros_driver2", executable="livox_ros_driver2_node",
             name="livox_lidar_publisher", parameters=[str(folder / "livox-driver.yaml")], output="screen"),
        Node(package="point_lio", executable="pointlio_mapping", name="point_lio",
             namespace="intramural_robot", parameters=[str(folder / "point-lio.yaml")], output="screen"),
    ]


def generate_launch_description():
    def failed(event, context):
        if event.returncode:
            raise RuntimeError(f"Vehicle sensor child failed: {event.returncode}")
        return []
    default_config = (
        "/work/cache/vehicle-srm-reference"
        if Path("/work").exists()
        else str(Path(__file__).resolve().parents[1] / "cache/vehicle-srm-reference")
    )
    return LaunchDescription([
        DeclareLaunchArgument("vehicle_config", default_value=default_config),
        RegisterEventHandler(OnProcessExit(on_exit=failed)),
        OpaqueFunction(function=start),
    ])
