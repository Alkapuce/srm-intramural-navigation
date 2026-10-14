"""Use the pinned upstream sensor/navigation chain with an explicit simulation clock."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch_ros.actions import SetParameter


def generate_launch_description():
    share = Path(get_package_share_directory('pb2025_nav_bringup'))
    def child_failed(event, context):
        if event.returncode != 0:
            raise RuntimeError(f'Navigation child failed: {event.returncode}')
        return []
    return LaunchDescription([
        RegisterEventHandler(OnProcessExit(on_exit=child_failed)),
        # The upstream RViz/joy launch does not forward its clock argument.
        SetParameter(name='use_sim_time', value=True),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            str(share / 'launch/rm_navigation_simulation_launch.py')),
            launch_arguments={
                'slam': 'False', 'use_composition': 'False', 'world': 'rmuc_2025',
                'prior_pcd_file': '/work/cache/maps/rmuc_2025.pcd',
                'params_file': '/work/config/stage2-nav.yaml',
                'rviz_config_file': '/work/config/stage2-nav.rviz',
                'use_rviz': 'False',
            }.items()),
    ])
