#!/usr/bin/env python3
"""Own simulator and sensor navigation processes; preserve abnormal exits."""
import argparse
import signal
import subprocess
import time
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--no-rviz', action='store_true')
args = parser.parse_args()
interrupted = False
failure = False

def interrupt(signum, frame):
    global interrupted
    interrupted = True
signal.signal(signal.SIGINT, interrupt)
signal.signal(signal.SIGTERM, interrupt)

def stop(proc, name, timeout):
    global failure
    if proc is None:
        return
    if proc.poll() is None:
        proc.send_signal(signal.SIGINT)
        try:
            proc.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            print(f'{name}: needed forced termination', flush=True)
            proc.kill()
            proc.wait()
            failure = True
    if proc.returncode:
        print(f'{name}: abnormal exit {proc.returncode}', flush=True)
        failure = True
    else:
        print(f'{name}: clean exit 0', flush=True)

simulator = subprocess.Popen(['ros2', 'launch', '/work/launch/simulator.launch.py'],
    start_new_session=True)
navigation = None
viewer = None
try:
    navigation = subprocess.Popen(['ros2', 'launch', '/work/launch/navigation.launch.py'], start_new_session=True)
    if not args.no_rviz:
        viewer = subprocess.Popen(['ros2','launch','/work/launch/navigation-rviz.launch.py'],
                                  start_new_session=True)
    deadline = time.monotonic() + 60
    started = False
    while not interrupted and time.monotonic() < deadline:
        if simulator.poll() is not None or navigation.poll() is not None:
            raise RuntimeError('Simulation or navigation exited during startup')
        result = subprocess.run(['ign', 'service', '-s', '/world/default/control',
            '--reqtype', 'ignition.msgs.WorldControl', '--reptype', 'ignition.msgs.Boolean',
            '--timeout', '1000', '--req', 'pause: false'], capture_output=True, text=True,
            timeout=5)
        if result.returncode == 0 and 'true' in result.stdout:
            started = True
            break
        time.sleep(0.5)
    if not interrupted and not started:
        raise RuntimeError('Gazebo world control did not become available')
    while (not interrupted and simulator.poll() is None and navigation.poll() is None
           and (viewer is None or viewer.poll() is None)):
        time.sleep(0.1)
    if not interrupted:
        print('System process exited before requested shutdown',flush=True)
        failure = True
finally:
    # Remove the goal-requesting GUI before cleaning up its action servers.
    # TF and simulation clock remain alive throughout viewer destruction.
    stop(viewer,'Navigation RViz',20)
    if navigation is not None and navigation.poll() is None:
        try:
            result = subprocess.run(['python3','/work/scripts/internal/shutdown-navigation.py'],timeout=40)
            if result.returncode:
                raise RuntimeError('Lifecycle shutdown was not acknowledged')
        except Exception as exc:
            print(f'Navigation lifecycle shutdown failed: {exc}',flush=True)
            failure = True
    # Stop all simulation data publishers before destroying navigation subscribers.
    stop(simulator, 'Simulation', 55)
    stop(navigation, 'Navigation', 20)
raise SystemExit(1 if failure else 0)
