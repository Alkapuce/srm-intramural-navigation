"""Launch the pinned RM simulator without an intervening signal-swallowing shell."""
import os
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, EmitEvent
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown


def generate_launch_description():
    # Mesa D3D12 exposes OpenGL 3.3 here. OGRE 1 supports this backend;
    # Fortress OGRE 2's texture-copy path requires unavailable GL features.
    engine_argument = DeclareLaunchArgument('render_engine', default_value=
        'ogre' if os.environ.get('LIBGL_ALWAYS_SOFTWARE', '0') == '0' else 'ogre2')
    render_engine = LaunchConfiguration('render_engine')
    share = Path(get_package_share_directory('rmu_gazebo_simulator'))
    config = yaml.safe_load((share / 'config/gz_world.yaml').read_text())
    world = share / 'resource/worlds' / f"{config['world']}_world.sdf"
    gui = Path(__file__).resolve().parents[1] / 'config/navigation-gui.config'
    gazebo = ExecuteProcess(
        cmd=['python3', '/work/scripts/internal/gazebo-process.py', '--world', str(world),
             '--gui-config', str(gui), '--engine', render_engine],
        name='gazebo', shell=False, output='screen', sigterm_timeout='50',
        sigkill_timeout='5')
    stop_others = RegisterEventHandler(OnProcessExit(target_action=gazebo,
        on_exit=[EmitEvent(event=Shutdown(reason='Gazebo exited'))]))
    return LaunchDescription([engine_argument, stop_others, gazebo])
