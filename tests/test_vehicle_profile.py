"""Check hardware handoff configuration without network, device or private source access."""

import importlib.util
import json
import socket
import sys
import tempfile
import unittest
from pathlib import Path

import yaml
from launch import LaunchContext

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
        result = prepare(self.root, self.output, "192.168.1.51", "192.168.1.185", runtime_root="/work")
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

    def test_native_driver_configuration_is_readable(self):
        prepare(self.root, self.output, runtime_root=self.root)
        driver = yaml.safe_load((self.output / "livox-driver.yaml").read_text())["/**"]["ros__parameters"]
        path = Path(driver["user_config_path"])
        self.assertEqual(path.parent, self.output)
        network = json.loads(path.read_text())
        self.assertIn("MID360", network)

    def test_real_navigation_motion_requires_explicit_accepted_field_and_vehicle(self):
        prepare(self.root, self.output, runtime_root=self.root)
        folder = Path(self.temp.name) / "map"
        folder.mkdir()
        arena = json.loads((self.root / "config/arena.json").read_text())
        arena["max_speed"] = .4
        (folder / "arena.json").write_text(json.dumps(arena))
        (folder / "nav.yaml").write_text("{}\n")
        manifest = {"motion_permitted": False, "start_permitted": False}
        (folder / "manifest.json").write_text(json.dumps(manifest))
        spec = importlib.util.spec_from_file_location("vehicle_navigation", self.root / "launch/vehicle-navigation.launch.py")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        context = LaunchContext()
        context.launch_configurations.update(navigation_config=str(folder), vehicle_config=str(self.output),
            serial_device="/dev/never-opened-test", enable_motion="false", use_sim_time="false")
        standby = module.start(context)
        context.launch_configurations["enable_motion"] = "true"
        with self.assertRaisesRegex(ValueError, "Vehicle validation is missing"):
            module.start(context)
        base = json.loads((self.output / "base-vehicle.json").read_text())
        base.update(calibrated_for_vehicle=True, firmware_protocol_verified=True, link_loss_stop_verified=True,
            body_to_protocol=[[1., 0.], [0., 1.]], max_body_speed_mps=.4,max_protocol_axis=.4)
        (self.output / "base-vehicle.json").write_text(json.dumps(base))
        with self.assertRaisesRegex(ValueError, "parking/pole geometry"):
            module.start(context)
        manifest["field_geometry_verified"] = True
        (folder / "manifest.json").write_text(json.dumps(manifest))
        drive = module.start(context)
        self.assertEqual(len(drive[-1].get_sub_entities()), len(standby[-1].get_sub_entities()) + 1)
        arena["max_speed"] = .41
        (folder / "arena.json").write_text(json.dumps(arena))
        with self.assertRaisesRegex(ValueError, "mission speed"):
            module.start(context)

    def test_sensor_launch_rejects_an_occupied_udp_port_before_starting_nodes(self):
        prepare(self.root, self.output, runtime_root=self.root)
        config_path = self.output / "MID360_config.json"
        config = json.loads(config_path.read_text())
        spec = importlib.util.spec_from_file_location("vehicle_sensors", self.root / "launch/vehicle-sensors.launch.py")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        context = LaunchContext()
        context.launch_configurations["vehicle_config"] = str(self.output)
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as owner:
            owner.bind(("127.0.0.1", 0))
            config["MID360"]["host_net_info"]["cmd_data_ip"] = "127.0.0.1"
            config["MID360"]["host_net_info"]["cmd_data_port"] = owner.getsockname()[1]
            config_path.write_text(json.dumps(config))
            with self.assertRaisesRegex(RuntimeError, "endpoint .* unavailable"):
                module.start(context)

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
