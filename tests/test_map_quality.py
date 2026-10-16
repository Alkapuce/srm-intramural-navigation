"""Exercise the map-quality CLI against complete, sparse and malformed raster maps."""
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import numpy as np
from PIL import Image
import yaml


class MapQualityTests(unittest.TestCase):
    def check_map(self, grid, expected_code=0):
        root = Path(__file__).resolve().parents[1]
        evidence = root / "artifacts/tests/map-quality"
        evidence.mkdir(parents=True, exist_ok=True)
        folder = Path(tempfile.mkdtemp(dir=evidence))
        Image.fromarray(grid).save(folder / "map.pgm")
        (folder / "map.yaml").write_text(yaml.safe_dump({"image": "map.pgm", "resolution": .05}))
        (folder / "arena.json").write_text(json.dumps({"robot_radius": .29}))
        result = subprocess.run([sys.executable, str(root / "scripts/validation/verify-map-quality.py"),
                                 "--map-config", str(folder), "--output", str(folder / "quality.json")],
                                capture_output=True, text=True, timeout=20)
        (folder / "process.log").write_text(result.stdout + result.stderr)
        self.assertEqual(result.returncode, expected_code, result.stderr)
        return json.loads((folder / "quality.json").read_text()) if expected_code == 0 else None

    def test_observed_room_has_clearance(self):
        grid = np.zeros((80, 80), dtype=np.uint8)
        grid[1:-1, 1:-1] = 254
        report = self.check_map(grid)
        self.assertTrue(report["navigation_grid_has_clearance"])
        self.assertEqual(report["robot_clear_components"], 1)
        self.assertGreater(report["largest_robot_clear_area_m2"], 4)
        self.assertFalse(report["competition_route_verified"])

    def test_sparse_ground_returns_do_not_make_unknown_free(self):
        grid = np.full((80, 80), 205, dtype=np.uint8)
        grid[::2, ::2] = 254
        report = self.check_map(grid)
        self.assertGreater(report["free_area_m2"], 0)
        self.assertFalse(report["navigation_grid_has_clearance"])
        self.assertEqual(report["robot_clear_area_m2"], 0)
        self.assertTrue(all(r["clear_area_m2"] == 0 for r in report["radius_sensitivity"]))

    def test_rejects_unexpected_pixel_encoding(self):
        self.check_map(np.full((20, 20), 123, dtype=np.uint8), expected_code=1)


if __name__ == "__main__":
    unittest.main()
