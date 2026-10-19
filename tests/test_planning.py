"""Map compiler entry cases through the same interface used after site mapping."""

import json
import math
import unittest
from pathlib import Path
import numpy as np
from nav_msgs.msg import OccupancyGrid
from intramural_mission.planning import compile_route
from intramural_mission.route import segments
from intramural_mission.trajectory import OmniTracker

ROOT = Path(__file__).resolve().parents[1]


class PlanningTests(unittest.TestCase):
    def test_fresh_map_and_arbitrary_entry_are_one_ordered_smooth_plan(self):
        c = json.loads((ROOT / "config/arena.json").read_text())
        c.update(json.loads((ROOT / "config/navigation-profiles.json").read_text())["safe"])
        c.update(
            lower_pole=[0.0, 0.0],
            upper_pole=[0.0, 1.246],
            loop_radius=0.623,
            parking=[2.0, -1.0],
            bounds=[-4.0, 4.0, -4.0, 4.0],
            entry_angle=0.0,
            pole_clearance_from_map=True,
        )
        g = OccupancyGrid()
        g.header.frame_id = "map"
        g.info.width = g.info.height = 320
        g.info.resolution = 0.025
        g.info.origin.position.x = g.info.origin.position.y = -4.0
        g.info.origin.orientation.w = 1.0
        yy, xx = np.mgrid[:320, :320]
        pixels = np.zeros((320, 320), dtype=np.int8)
        for px, py in (c["lower_pole"], c["upper_pole"]):
            pixels[
                np.hypot(-4 + (xx + 0.5) * 0.025 - px, -4 + (yy + 0.5) * 0.025 - py)
                < 0.15
            ] = 100
        g.data = pixels.ravel().tolist()
        gate = segments(c, smooth=False)[0][1][-1]
        for start in (None, [0.6, 1.8], [0.6, 0.3], gate):
            plan, route, report = compile_route(c, g, start)
            self.assertEqual(len(plan.gates), 10)
            self.assertEqual(plan.points[-1], tuple(c["parking"]))
            self.assertGreater(min(plan.clearance), 0)
            self.assertLessEqual(max(map(abs, plan.curvature)), 2 / c["robot_radius"])
            tracker = OmniTracker(plan, c)
            _, state = tracker.step(plan.points[0], 0.0, 0.025)
            self.assertFalse(state["done"])
        # Neither a fresh heartbeat nor a valid start pose makes an unknown map traversable.
        g.data = [-1] * len(g.data)
        with self.assertRaises(ValueError):
            compile_route(c, g)


if __name__ == "__main__":
    unittest.main()
