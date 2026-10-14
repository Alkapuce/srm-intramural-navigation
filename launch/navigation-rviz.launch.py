"""Navigation viewer; keep Nav2 plugin code alive until its ROS callbacks are gone."""
import os
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():
    def child_failed(event, context):
        if event.returncode != 0:
            raise RuntimeError(f'RViz exited with {event.returncode}')
        return []
    # Humble's Nav2 panel leaves callback-group weak_ptr control blocks whose
    # virtual destructors live in libnav2_rviz_plugins. VisualizerApp destroys
    # its plugin loaders before its ROS node. Preloading prevents dlclose from
    # unmapping those destructors while the node still refers to them.
    preload='/opt/ros/humble/lib/libnav2_rviz_plugins.so'
    if os.environ.get('LD_PRELOAD'):
        preload+=':'+os.environ['LD_PRELOAD']
    return LaunchDescription([
        RegisterEventHandler(OnProcessExit(on_exit=child_failed)),
        Node(package='rviz2',executable='rviz2',namespace='red_standard_robot1',
             arguments=['-d','/work/config/stage2-nav.rviz'],
             parameters=[{'use_sim_time':True}],
             remappings=[('/tf','tf'),('/tf_static','tf_static')],
             additional_env={'LD_PRELOAD':preload},output='screen'),
    ])
