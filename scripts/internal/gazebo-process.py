#!/usr/bin/env python3
"""Own Gazebo children and stop server before GUI, preserving child failures."""
import argparse
import os
import signal
import subprocess
import time
from pathlib import Path

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--world', required=True)
parser.add_argument('--gui-config', required=True)
parser.add_argument('--engine', required=True)
parser.add_argument('--nodes-launch', default=str(Path(__file__).resolve().parents[2] / 'launch/simulator-nodes.launch.py'))
parser.add_argument('--no-gui', action='store_true')
args = parser.parse_args()
interrupted = False


def interrupt(signum, frame):
    global interrupted
    interrupted = True


signal.signal(signal.SIGINT, interrupt)
signal.signal(signal.SIGTERM, interrupt)
common = ['ruby', '/usr/bin/ign', 'gazebo']
server = subprocess.Popen(common + ['-s', args.world, '--render-engine', args.engine,
                                  '--force-version', '6'], start_new_session=True)
viewer = None
ros_nodes = None
failure = False


def stop(proc, name):
    global failure
    if proc is None:
        return
    if proc.poll() is None:
        proc.send_signal(signal.SIGINT)
        try:
            proc.wait(timeout=15)
        except subprocess.TimeoutExpired:
            print(f'{name}: needed forced termination', flush=True)
            failure = True
            proc.kill()
            proc.wait(timeout=5)
    if proc.returncode != 0:
        print(f'{name}: abnormal exit {proc.returncode}', flush=True)
        failure = True
    else:
        print(f'{name}: clean exit 0', flush=True)


try:
    gui_command = common + ['-g', '--gui-config', args.gui_config,
        '--render-engine', args.engine, '--force-version', '6']
    if not args.no_gui:
        gui_env = os.environ.copy()
        if Path('/dev/dxg').exists() and gui_env.get('LIBGL_ALWAYS_SOFTWARE') != '1':
            # Qt's disposable GLX threading probe crashes in WSL Mesa; choose basic explicitly.
            gui_env['QT_OPENGL_NO_SANITY_CHECK'] = '1'
            gui_env['QSG_RENDER_LOOP'] = 'basic'
            gui_env['QSG_INFO'] = '1'
        viewer = subprocess.Popen(gui_command, start_new_session=True, env=gui_env)
    ros_nodes = subprocess.Popen(['ros2', 'launch',
        args.nodes_launch], start_new_session=True)
    while (not interrupted and server.poll() is None and (viewer is None or viewer.poll() is None)
           and ros_nodes.poll() is None):
        time.sleep(0.1)
finally:
    # Leave the GUI alive while the server releases sensor/render resources.
    stop(server, 'Gazebo server')
    stop(viewer, 'Gazebo GUI')
    # No more sensor/clock callbacks can race ROS publisher destruction.
    stop(ros_nodes, 'ROS simulation nodes')
raise SystemExit(1 if failure else 0)
