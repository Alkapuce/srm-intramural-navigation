#!/usr/bin/env python3
"""Run while the simulator is publishing, to exercise controller teardown."""
import json
import signal
import subprocess
import time
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_prefix
from rmoss_interfaces.msg import Gimbal

out = Path('/work/artifacts/stage1/base-lifecycle')
out.mkdir(parents=True, exist_ok=True)
binary = Path(get_package_prefix('rmoss_gz_base')) / 'lib/rmoss_gz_base/rmua19_robot_base'
config = '/work/nav_ws/install/rmu_gazebo_simulator/share/rmu_gazebo_simulator/config/base_params.yaml'
results = []
rclpy.init()
observer = rclpy.create_node('base_lifecycle_observer')
try:
    for iteration in range(8):
        namespace = f'/stage1_lifecycle_{iteration}'
        samples = []
        subscription = observer.create_subscription(Gimbal,
            namespace + '/robot_base/gimbal_state', samples.append, 10)
        with (out / f'{iteration}.log').open('w') as log:
            proc = subprocess.Popen([str(binary), '--ros-args', '-r',
                '__ns:=' + namespace, '--params-file', config],
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            try:
                deadline = time.monotonic() + 10
                while time.monotonic() < deadline and len(samples) < 3:
                    if proc.poll() is not None:
                        raise RuntimeError(f'Base exited early: {proc.returncode}')
                    rclpy.spin_once(observer, timeout_sec=0.1)
                if len(samples) < 3:
                    raise RuntimeError('Base controller did not publish live state')
            finally:
                if proc.poll() is None:
                    proc.send_signal(signal.SIGINT)
                    proc.wait(timeout=10)
            if proc.returncode != 0:
                raise RuntimeError(f'Base iteration {iteration}: exit {proc.returncode}')
            results.append({'iteration': iteration, 'state_samples': len(samples),
                            'exit_code': proc.returncode})
        observer.destroy_subscription(subscription)
finally:
    observer.destroy_node()
    rclpy.shutdown()
(out / 'results.json').write_text(json.dumps(results, indent=2) + '\n')
print(f'PASS: {len(results)} live base controller starts / SIGINT exits')
