"""Exercise the installed driver with no configured devices or physical SDK access."""

import json
import os
import signal
import subprocess
import tempfile
import time
import unittest
from pathlib import Path

from ament_index_python.packages import get_package_prefix


class LivoxShutdownTest(unittest.TestCase):
    def test_interrupt_and_terminate_with_no_sensor_data(self):
        root = Path(__file__).resolve().parents[1]
        binary = Path(get_package_prefix("livox_ros_driver2")) / "lib/livox_ros_driver2/livox_ros_driver2_node"
        with tempfile.TemporaryDirectory(dir=root / "cache") as folder:
            config = Path(folder) / "no-devices.json"
            config.write_text(json.dumps({"lidar_summary_info": {"lidar_type": 0}}))
            for sig in (signal.SIGINT, signal.SIGTERM):
                with self.subTest(signal=sig), (Path(folder) / "driver.log").open("w+") as log:
                    env = {**os.environ, "ROS_DOMAIN_ID": "227", "ROS_LOCALHOST_ONLY": "1"}
                    process = subprocess.Popen(
                        [str(binary), "--ros-args", "-p", "user_config_path:=" + str(config)],
                        env=env, stdout=log, stderr=subprocess.STDOUT,
                    )
                    try:
                        time.sleep(3.5)  # Both production polling threads have entered their waits.
                        self.assertIsNone(process.poll())
                        process.send_signal(sig)
                        self.assertEqual(process.wait(timeout=3), 0)
                    finally:
                        if process.poll() is None:
                            process.kill()
                            process.wait(timeout=3)
