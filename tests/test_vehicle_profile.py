"""Check hardware handoff configuration without network, device or private source access."""

import json
import sys
import tempfile
import unittest
from pathlib import Path

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from internal.vehicle_profile import prepare


class VehicleProfileTest(unittest.TestCase):
    def setUp(self):
        self.root = Path(__file__).resolve().parents[1]
        self.temp = tempfile.TemporaryDirectory(dir=self.root / "cache")
        self.addCleanup(self.temp.cleanup)
        self.output = Path(self.temp.name) / "vehicle"

    def test_public_driver_and_point_lio_handoff_leave_motion_unverified(self):
        before = (self.root / "config/arena.json").read_bytes()
        result = prepare(self.root, self.output, "192.168.1.51", "192.168.1.185")
        self.assertFalse(result["vehicle_validated"])
        network = json.loads((self.output / "MID360_config.json").read_text())
        self.assertEqual(network["lidar_configs"][0]["ip"], "192.168.1.185")
        self.assertEqual(network["MID360"]["host_net_info"]["point_data_ip"], "192.168.1.51")
        driver = yaml.safe_load((self.output / "livox-driver.yaml").read_text())["/**"]["ros__parameters"]
        self.assertEqual(driver["xfer_format"], 1)
        self.assertFalse(driver["use_sim_time"])
        self.assertEqual(driver["user_config_path"], str(Path("/work") / self.output.relative_to(self.root) / "MID360_config.json"))
        point = yaml.safe_load((self.output / "point-lio.yaml").read_text())["/**"]["ros__parameters"]
        self.assertEqual(point["preprocess"]["lidar_type"], 1)
        self.assertEqual(point["preprocess"]["timestamp_unit"], 3)
        self.assertEqual(point["common"]["lid_topic"], "/livox/lidar")
        base = json.loads((self.output / "base-vehicle.json").read_text())
        self.assertFalse(base["firmware_protocol_verified"])
        self.assertFalse(base["link_loss_stop_verified"])
        self.assertFalse(base["calibrated_for_vehicle"])
        self.assertIsNone(base["body_to_protocol"][0][0])
        self.assertEqual((self.root / "config/arena.json").read_bytes(), before)

    def test_refuse_overwrite_and_invalid_network_before_writing(self):
        for host, lidar in (("bad", "192.168.1.184"), ("0.0.0.0", "192.168.1.184"),
                            ("192.168.1.50", "192.168.1.50")):
            with self.subTest(host=host), self.assertRaises(ValueError):
                prepare(self.root, self.output, host, lidar)
            self.assertFalse(self.output.exists())
        prepare(self.root, self.output)
        with self.assertRaisesRegex(ValueError, "already exists"):
            prepare(self.root, self.output)


if __name__ == "__main__":
    unittest.main()
