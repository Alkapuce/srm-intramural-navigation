"""Organizer model with public simulated sensors and task-level planar drive."""

import os, json
from pathlib import Path
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():
    root = Path(__file__).resolve().parents[1]
    c = json.loads(Path(os.environ.get("INTRAMURAL_ARENA_CONFIG", str(root / "config/arena.json"))).read_text())
    ns = c["namespace"]
    folder = root / "simulation/generated"

    def failed(event, context):
        if event.returncode:
            raise RuntimeError(f"Arena simulation child failed: {event.returncode}")
        return []

    return LaunchDescription(
        [
            RegisterEventHandler(OnProcessExit(on_exit=failed)),
            Node(
                package="ros_gz_sim",
                executable="create",
                arguments=[
                    "-file",
                    str(folder / "robot.sdf"),
                    "-name",
                    ns,
                    "-x",
                    str(c["parking"][0]),
                    "-y",
                    str(c["parking"][1]),
                    "-z",
                    "0.01",
                ],
                output="screen",
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                namespace=ns,
                parameters=[{"config_file": str(folder / "bridge.yaml")}],
                output="screen",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                namespace=ns,
                parameters=[
                    {
                        "use_sim_time": True,
                        "robot_description": (folder / "robot.urdf").read_text(),
                    }
                ],
                remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
                output="screen",
            ),
        ]
    )
