"""Map compiler entry cases through the same interface used after site mapping."""

import json
import math
import unittest
from pathlib import Path
import numpy as np
from nav_msgs.msg import OccupancyGrid
from intramural_mission.planning import compile_route, GridMap
from intramural_mission.response import predict_execution
from intramural_mission.route import segments
from intramural_mission.trajectory import OmniTracker

ROOT = Path(__file__).resolve().parents[1]


class PlanningTests(unittest.TestCase):
    def test_default_candidate_tracks_known_faster_plants_without_retuning(self):
        c=json.loads((ROOT/'config/arena.json').read_text())
        c.update(json.loads((ROOT/'config/navigation-profiles.json').read_text())['candidate'])
        c.pop('optimized_route',None)
        g=OccupancyGrid();g.header.frame_id='map';g.info.width=g.info.height=200;g.info.resolution=.05
        g.info.origin.position.x=g.info.origin.position.y=-2.;g.info.origin.orientation.w=1.
        yy,xx=np.mgrid[:200,:200];pixels=np.zeros((200,200),dtype=np.int8)
        for x,y in (c['lower_pole'],c['upper_pole']):
            pixels[np.hypot(-2+(xx+.5)*.05-x,-2+(yy+.5)*.05-y)<c['pole_radius']]=100
        g.data=pixels.ravel().tolist()
        plan,_,_=compile_route(c,g)
        for plant in c['response_models']:
            result=predict_execution(plan,c,GridMap(g,c),plant_response=plant)
            self.assertTrue(result['passed'],result)
            self.assertGreaterEqual(result['minimum_clearance'],c['execution_clearance'],result)

    def test_changed_pole_spacing_in_a_corridor_can_use_narrower_loops(self):
        c=json.loads((ROOT/'config/arena.json').read_text())
        c.update(json.loads((ROOT/'config/navigation-profiles.json').read_text())['safe'])
        c.update(lower_pole=[0.,0.],upper_pole=[0.,3.],loop_radius=1.5,parking=[.3,-2.],entry_angle=0.)
        g=OccupancyGrid();g.header.frame_id='map';g.info.width=120;g.info.height=320;g.info.resolution=.025
        g.info.origin.position.x=-1.5;g.info.origin.position.y=-2.5;g.info.origin.orientation.w=1.
        yy,xx=np.mgrid[:320,:120];x=-1.5+(xx+.5)*.025;y=-2.5+(yy+.5)*.025
        pixels=np.where(np.abs(x)>1.15,100,0)
        for px,py in (c['lower_pole'],c['upper_pole']):pixels[np.hypot(x-px,y-py)<.15]=100
        g.data=pixels.astype(np.int8).ravel().tolist()
        with self.assertRaises(ValueError):compile_route(dict(c,optimize_route=False),g)
        plan,_,report=compile_route(c,g)
        self.assertTrue(report['execution_prediction']['passed'])
        self.assertLess(max(report['optimized_route']['shape'][:2]),1.)
        self.assertEqual(len(plan.gates),10)
        self.assertGreater(min(plan.clearance),0)
        # Keep the controller model fixed while the actual plant becomes much slower.
        stressed=predict_execution(plan,c,GridMap(g,c),plant_response=(1.,.6))
        self.assertFalse(stressed['passed'])

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
            self.assertTrue(np.isfinite(plan.curvature).all())
            self.assertLessEqual(max(np.asarray(plan.speed)**2*np.abs(plan.curvature)),c['lateral_acceleration']+1e-6)
            tracker = OmniTracker(plan, c)
            _, state = tracker.step(plan.points[0], 0.0, 0.025)
            self.assertFalse(state["done"])
            self.assertTrue(report["execution_prediction"]["passed"])
            self.assertGreaterEqual(min(p['minimum_clearance'] for p in report['response_predictions']),
                                    c['execution_clearance'])
            frozen = dict(c,optimized_route=report["optimized_route"])
            reloaded,_,cached = compile_route(frozen,g,start)
            np.testing.assert_allclose(reloaded.points,plan.points,atol=1e-10)
            np.testing.assert_allclose(reloaded.speed,plan.speed,atol=1e-10)
            self.assertEqual(cached["candidates_evaluated"],1)
        # Neither a fresh heartbeat nor a valid start pose makes an unknown map traversable.
        g.data = [-1] * len(g.data)
        with self.assertRaises(ValueError):
            compile_route(c, g)


if __name__ == "__main__":
    unittest.main()
