"""ROS nodes kept alive until Gazebo publishers have stopped."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('rmu_gazebo_simulator'))
    clock = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'], output='screen')
    includes = [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            str(share / 'launch' / filename)))
        for filename in ['spawn_robots.launch.py', 'referee_system.launch.py']
    ]
    return LaunchDescription([clock, *includes])
