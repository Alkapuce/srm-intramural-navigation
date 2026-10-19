#!/usr/bin/env python3
"""Run inside the development container; capture only task GUI windows."""
import argparse
import json
import os
import re
import signal
import subprocess
import time
from pathlib import Path

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--output', default='/work/artifacts/stage1/gui')
parser.add_argument('--base-lifecycle', action='store_true')
options = parser.parse_args()
out = Path(options.output).resolve()
if not out.is_relative_to(Path('/work/artifacts')):
    raise ValueError('GUI verification output must stay under /work/artifacts')
out.mkdir(parents=True, exist_ok=True)
results = []
graphics = subprocess.check_output(['glxinfo', '-B'], text=True)
(out / 'graphics.txt').write_text(graphics)
expect_gpu = os.environ.get('LIBGL_ALWAYS_SOFTWARE', '0') == '0'
if expect_gpu and 'Accelerated: yes' not in graphics:
    raise RuntimeError('GPU requested but GLX reports no hardware acceleration')


def driver_maps(pid):
    """Capture the renderer processes themselves, beyond a separate GLX probe."""
    found = {}
    def visit(current):
        base = Path('/proc') / str(current)
        try:
            command = (base / 'cmdline').read_bytes().replace(b'\0', b' ').decode()
            if current == pid or command.startswith('ign gazebo'):
                lines = [line for line in (base / 'maps').read_text().splitlines()
                         if '/usr/lib/wsl/' in line]
                found[str(current)] = {'command': command, 'wsl_driver_maps': lines}
            for child in (base / 'task' / str(current) / 'children').read_text().split():
                visit(int(child))
        except (FileNotFoundError, ProcessLookupError):
            pass
    visit(pid)
    return found
for filename in ('results.json', 'clock.json', 'sensors.json'):
    (out / filename).unlink(missing_ok=True)


def find_window(pattern):
    tree = subprocess.check_output(['xwininfo', '-root', '-tree'], text=True)
    candidates = []
    for line in tree.splitlines():
        if re.search(pattern, line, re.I):
            match = re.search(r'(0x[0-9a-f]+)', line)
            size = re.search(r'(\d+)x(\d+)[+-]', line)
            if match:
                area = int(size[1]) * int(size[2]) if size else 0
                candidates.append((area, match[1]))
    return max(candidates)[1] if candidates else None


def verify(name, command, pattern):
    with (out / f'{name}.log').open('w') as log:
        proc = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                start_new_session=True)
        try:
            deadline = time.monotonic() + 90
            window = None
            while time.monotonic() < deadline:
                if proc.poll() is not None:
                    raise RuntimeError(f'{name} exited early: {proc.returncode}')
                window = find_window(pattern)
                if window:
                    break
                time.sleep(1)
            if not window:
                raise RuntimeError(f'{name}: no visible window')
            time.sleep(8)
            if proc.poll() is not None:
                raise RuntimeError(f'{name} exited before capture')
            if name == 'gazebo':
                control = subprocess.run([
                    'ign', 'service', '-s', '/world/default/control',
                    '--reqtype', 'ignition.msgs.WorldControl',
                    '--reptype', 'ignition.msgs.Boolean', '--timeout', '5000',
                    '--req', 'pause: false'], capture_output=True, text=True, check=True)
                if 'true' not in control.stdout:
                    raise RuntimeError(f'Cannot start physics: {control.stdout}')
                import rclpy
                from rosgraph_msgs.msg import Clock
                from sensor_msgs.msg import PointCloud2, Imu
                from rclpy.qos import qos_profile_sensor_data
                rclpy.init()
                node = rclpy.create_node('stage1_clock_check')
                ticks = []
                sensors = {}
                def lidar_sample(msg):
                    sensors['lidar'] = {'width': msg.width, 'height': msg.height,
                        'bytes': len(msg.data), 'frame_id': msg.header.frame_id,
                        'fields': [field.name for field in msg.fields]}
                def imu_sample(msg):
                    sensors['imu'] = {'frame_id': msg.header.frame_id,
                        'stamp': msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9}
                sensor_subscriptions = [
                    node.create_subscription(PointCloud2,
                        '/red_standard_robot1/livox/lidar', lidar_sample,
                        qos_profile_sensor_data),
                    node.create_subscription(Imu, '/red_standard_robot1/livox/imu',
                        imu_sample, qos_profile_sensor_data)]
                subscription = node.create_subscription(
                    Clock, '/clock', lambda msg: ticks.append(
                        msg.clock.sec + msg.clock.nanosec / 1e9), 10)
                deadline = time.monotonic() + 60
                try:
                    while time.monotonic() < deadline:
                        rclpy.spin_once(node, timeout_sec=0.2)
                        if len(ticks) > 1 and ticks[-1] - ticks[0] >= 2.0 and len(sensors) == 2:
                            break
                    if len(ticks) < 2 or ticks[-1] - ticks[0] < 2.0:
                        raise RuntimeError('ROS /clock did not advance')
                    if len(sensors) != 2 or sensors['lidar']['bytes'] == 0:
                        raise RuntimeError('No live lidar/IMU sensor samples')
                    (out / 'sensors.json').write_text(json.dumps(sensors, indent=2) + '\n')
                    (out / 'clock.json').write_text(json.dumps(
                        {'first': ticks[0], 'last': ticks[-1], 'samples': len(ticks)},
                        indent=2) + '\n')
                finally:
                    node.destroy_node()
                    rclpy.shutdown()
                # Exercise the public plugin after initial rendering, too.
                for state in (2, 3, 1):
                    subprocess.run(['ign', 'topic', '-t',
                        '/red_standard_robot1/color/set_state',
                        '-m', 'ignition.msgs.Int32', '-p', f'data: {state}'],
                        check=True, timeout=10)
                    time.sleep(2)
                (out / 'ignition-topics.txt').write_text(subprocess.check_output(
                    ['ign', 'topic', '-l'], text=True, timeout=10))
                if options.base_lifecycle:
                    subprocess.run(['python3', '/work/scripts/validation/verify-base-lifecycle.py'],
                                   check=True, timeout=90)
            # RViz may replace its splash window after graphics initialization.
            window = find_window(pattern)
            if not window:
                raise RuntimeError(f'{name}: main window disappeared before capture')
            mappings = driver_maps(proc.pid)
            (out / f'{name}-driver-maps.json').write_text(json.dumps(mappings, indent=2) + '\n')
            if expect_gpu:
                renderers = [value for value in mappings.values()
                             if value['wsl_driver_maps']]
                minimum = 1 if name == 'rviz' else 2
                if sum(any('/usr/lib/wsl/drivers/' in line
                           for line in value['wsl_driver_maps'])
                       for value in renderers) < minimum:
                    raise RuntimeError(f'{name}: actual renderer processes did not load vGPU drivers')
            dump = out / f'{name}.xwd'
            subprocess.run(['xwd', '-silent', '-id', window, '-out', str(dump)], check=True)
            subprocess.run(['convert', str(dump), str(out / f'{name}.png')], check=True)
            dump.unlink()
            colors = int(subprocess.check_output([
                'identify', '-format', '%k', str(out / f'{name}.png')], text=True))
            if colors < 64:
                raise RuntimeError(f'{name}: screenshot is blank or lacks rendered content')
            if name == 'gazebo':
                scene_colors = int(subprocess.check_output([
                    'convert', str(out / f'{name}.png'), '-gravity', 'center',
                    '-crop', '60%x50%+0+0', '+repage', '-format', '%k', 'info:'], text=True))
                if scene_colors < 64:
                    raise RuntimeError('gazebo: window appeared but the central scene is blank')
        finally:
            if proc.poll() is None:
                proc.send_signal(signal.SIGINT)
                try:
                    proc.wait(timeout=55)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid, signal.SIGTERM)
                    proc.wait(timeout=10)
                    raise RuntimeError(f'{name}: needed forced termination')
        if proc.returncode != 0:
            raise RuntimeError(f'{name}: nonzero exit code {proc.returncode}')
        log.flush()
        text = (out / f'{name}.log').read_text()
        if any(marker in text for marker in
               ['escalating to', 'process has died', 'Segmentation fault', 'Aborted',
                'exception occurred', 'Traceback (most recent call last)',
                'Another item already exists', 'needed forced termination', 'abnormal exit']):
            raise RuntimeError(f'{name}: child process did not exit cleanly; inspect log')
        time.sleep(1)
        probe = subprocess.run(['xwininfo', '-id', window], stdout=subprocess.DEVNULL,
                               stderr=subprocess.DEVNULL)
        if probe.returncode == 0:
            raise RuntimeError(f'{name}: GUI window remained after shutdown')
        results.append({'application': name, 'window': window,
                        'exit_code': proc.returncode, 'screenshot': f'{name}.png'})
        print(json.dumps(results[-1]), flush=True)


verify('rviz', ['rviz2'], r'"[^"\n]*RViz')
verify('gazebo', ['ros2', 'launch', '/work/launch/simulator.launch.py'],
       r'"[^"\n]*(Gazebo|Ignition)')
(out / 'results.json').write_text(json.dumps(results, indent=2) + '\n')
