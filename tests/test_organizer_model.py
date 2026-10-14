"""Check actual organizer conversion and conservative navigation geometry."""

import copy
import json
import subprocess
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from xmacro.xmacro4sdf import XMLMacro4sdf

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts/internal"))
from organizer_model import build_robot


class OrganizerModelTests(unittest.TestCase):
    def setUp(self):
        self.c = json.loads((ROOT / "config/arena.json").read_text())
        x = XMLMacro4sdf()
        x.set_xml_file(str(Path(get_package_share_directory("pb2025_robot_description")) /
                           "resource/xmacro/simulation_robot.sdf.xmacro"))
        x.generate({"global_initial_color": "red"})
        self.public = ET.fromstring(x.to_string()).find("model")
        self.temp = tempfile.TemporaryDirectory(dir=ROOT / "cache")
        self.addCleanup(self.temp.cleanup)
        self.out = Path(self.temp.name)

    def test_source_geometry_frames_and_sensor_origin(self):
        path = ROOT / "算法组校内赛规则/sentry_robot_cylinder.xacro"
        original = path.read_bytes()
        sdf, info = build_robot(ROOT, self.c, self.public, self.out)
        body = sdf.find("model/link[@name='base_footprint']")
        self.assertEqual(len(body.findall("collision")), 5)
        self.assertAlmostEqual(float(body.find("inertial/mass").text), 10.2)
        self.assertGreater(info["collision_envelope_radius_m"], .27)
        self.assertLess(info["collision_envelope_radius_m"], .29)
        sensor = sdf.find("model/link[@name='front_mid360']/sensor[@type='gpu_lidar']")
        self.assertEqual(sensor.find("pose").text, "0 0 0 0 0 0")
        self.assertEqual(sensor.find('ray/visibility_mask').text,'4')
        self.assertTrue(all(v.find('visibility_flags').text == '1' for v in body.findall('visual')))
        joint = sdf.find("model/joint[@name='front_livox_joint']/pose")
        self.assertEqual(list(map(float, joint.text.split())), self.c["lidar_pose"])
        urdf = ET.parse(self.out / "robot.urdf").getroot()
        self.assertEqual(len(urdf.findall("link")), 8)
        self.assertEqual(urdf.find("joint[@name='livox_to_sim_sensor']/parent").get("link"), "livox_frame")
        for mesh in urdf.findall(".//mesh"):
            self.assertTrue(Path(mesh.get("filename").removeprefix("file://")).is_file())
        subprocess.run(["check_urdf", str(self.out / "robot.urdf")], check=True, capture_output=True)
        self.assertEqual(path.read_bytes(), original)

    def test_body_only_radius_and_invalid_mount_are_rejected(self):
        for updates in ({"robot_radius": .27}, {"lidar_pose": [0, 0, float("nan"), 0, 0, 0]}):
            with self.subTest(updates=updates), self.assertRaises(ValueError):
                build_robot(ROOT, {**copy.deepcopy(self.c), **updates}, self.public, self.out)
