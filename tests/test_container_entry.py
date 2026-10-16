"""Check NUC runtime arguments with a fake Docker executable; no devices are opened."""

import json
import os
import subprocess
import tempfile
import unittest
from pathlib import Path


class ContainerEntryTest(unittest.TestCase):
    def setUp(self):
        self.root = Path(__file__).resolve().parents[1]
        (self.root / "cache").mkdir(exist_ok=True)
        self.temp = tempfile.TemporaryDirectory(dir=self.root / "cache")
        self.addCleanup(self.temp.cleanup)
        self.folder = Path(self.temp.name)
        docker = self.folder / "docker"
        docker.write_text('#!/usr/bin/env python3\nimport json,sys\nprint(json.dumps(sys.argv[1:]))\n')
        docker.chmod(0o755)
        self.env = os.environ.copy()
        self.env.update(PATH=str(self.folder) + os.pathsep + self.env["PATH"],
                        INTRAMURAL_NATIVE="0",
                        INTRAMURAL_ROS_DOMAIN_ID="091", INTRAMURAL_IGN_PARTITION="personal_nuc")
        self.env.pop("INTRAMURAL_SERIAL_DEVICE", None)

    def test_personal_domain_and_explicit_device_mapping(self):
        # /dev/null is only a harmless character-device fixture in the captured arguments.
        self.env["INTRAMURAL_SERIAL_DEVICE"] = "/dev/null"
        result = subprocess.run(["bash", str(self.root / "scripts/run.sh"), "true"],
                                env=self.env, text=True, capture_output=True, check=True)
        args = json.loads(result.stdout)
        self.assertIn("ROS_DOMAIN_ID=91", args)
        self.assertIn("IGN_PARTITION=personal_nuc", args)
        self.assertIn("/dev/null:/dev/intramural-base", args)
        self.assertIn("--group-add", args)
        self.assertNotIn("--privileged", args)
        self.assertNotIn("/dev:/dev", args)
        self.assertIn("io.srm.intramural.root=" + str(self.root), args)

    def test_reject_bad_domain_or_missing_device_before_docker(self):
        for override in ({"INTRAMURAL_ROS_DOMAIN_ID":"233"},
                         {"INTRAMURAL_ROS_DOMAIN_ID":"bad"},
                         {"INTRAMURAL_SERIAL_DEVICE":"/dev/intramural-fixture-missing"}):
            with self.subTest(override=override):
                result = subprocess.run(["bash", str(self.root / "scripts/run.sh"), "true"],
                                        env={**self.env, **override}, text=True, capture_output=True)
                self.assertNotEqual(result.returncode, 0)
                self.assertEqual(result.stdout, "")

    def test_navigation_entry_forwards_personal_vehicle_configuration(self):
        env = {**self.env, "INTRAMURAL_NAVIGATION_CONFIG": "/work/cache/field-map",
               "INTRAMURAL_VEHICLE_CONFIG": "/work/cache/personal-vehicle"}
        result = subprocess.run(["bash", str(self.root / "scripts/vehicle-navigation.sh"),
                                 "enable_motion:=true", "serial_device:=/dev/null"],
                                env=env, text=True, capture_output=True, check=True)
        args = json.loads(result.stdout)
        self.assertIn("navigation_config:=/work/cache/field-map", args)
        self.assertIn("vehicle_config:=/work/cache/personal-vehicle", args)
        self.assertIn("enable_motion:=true", args)


if __name__ == "__main__":
    unittest.main()
