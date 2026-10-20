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
from intramural_mission.trajectory import OmniTracker, RoutePlan

ROOT = Path(__file__).resolve().parents[1]


class PlanningTests(unittest.TestCase):
    def test_tangent_entry_uses_short_approach_when_side_guides_are_blocked(self):
        c=json.loads((ROOT/'config/arena.json').read_text())
        c.update(lower_pole=[0.,0.],upper_pole=[-1.65,-.60],loop_radius=.878,
                 robot_radius=.29,pole_radius=.15,minimum_clearance=.02,
                 mission_variant='figure8',optimize_route=True,bounds=[-3.,3.,-3.,3.])
        g=OccupancyGrid();g.header.frame_id='map';g.info.width=g.info.height=120
        g.info.resolution=.05;g.info.origin.position.x=g.info.origin.position.y=-3.
        g.info.origin.orientation.w=1.;yy,xx=np.mgrid[:120,:120]
        x,y=-3+(xx+.5)*.05,-3+(yy+.5)*.05
        pixels=np.where(y>1.3,100,0)
        for px,py in (c['lower_pole'],c['upper_pole']):
            pixels[np.hypot(x-px,y-py)<.15]=100
        g.data=pixels.astype(np.int8).ravel().tolist();measured=GridMap(g,c)
        start=[-1.22,-1.];gate=[-.25,.684];t=np.array([.949,.316]);t/=np.linalg.norm(t)
        paths=list(measured.connectors(start,gate,t))
        self.assertTrue(paths)
        for path in paths:
            np.testing.assert_allclose(path[0],start)
            np.testing.assert_allclose(path[-1],gate)
            self.assertGreater(min(measured.clearance(path)),0.)
            end=np.array(path[-1])-path[-2]
            self.assertGreater(np.dot(end,t)/np.linalg.norm(end),.98)

    def test_runtime_map_clearance_keeps_body_protection_without_route_pole_margin(self):
        c=json.loads((ROOT/'config/arena.json').read_text())
        c.update(lower_pole=[0.,0.],upper_pole=[0.,1.2],robot_radius=.29,
                 pole_radius=.15,minimum_clearance=.02,mission_variant='figure8')
        g=OccupancyGrid();g.header.frame_id='map';g.info.width=g.info.height=80
        g.info.resolution=.05;g.info.origin.position.x=g.info.origin.position.y=-2.
        g.info.origin.orientation.w=1.;pixels=np.zeros((80,80),dtype=np.int8)
        pixels[40,40]=100;pixels[50,30]=-1;g.data=pixels.ravel().tolist()
        measured=GridMap(g,c)
        self.assertLess(measured.clearance([[.4336,0.]])[0],0.)
        self.assertGreater(measured.clearance([[.4336,0.]],include_poles=False)[0],0.)
        self.assertLess(measured.clearance([[.2,0.]],include_poles=False)[0],0.)
        self.assertLess(measured.clearance([[-.475,.525]],include_poles=False)[0],0.)

    def test_prediction_rejects_the_required_reserve_before_finishing_a_failed_candidate(self):
        c=json.loads((ROOT/'config/arena.json').read_text())
        c.update(json.loads((ROOT/'config/navigation-profiles.json').read_text())['safe'])
        c.update(mission_variant='return',parking=[2.,0.],lower_pole=[-5.,-5.],upper_pole=[-5.,5.])
        g=OccupancyGrid();g.header.frame_id='map';g.info.width=100;g.info.height=40
        g.info.resolution=.05;g.info.origin.position.x=g.info.origin.position.y=-1.
        g.info.origin.orientation.w=1.;g.data=[0]*4000
        measured=GridMap(g,c)
        plan=RoutePlan.build([('RETURN',[[x,0.] for x in np.linspace(0.,2.,41)])],c)
        complete=predict_execution(plan,c,measured)
        self.assertTrue(complete['passed'],complete)
        self.assertLess(complete['minimum_clearance'],.7)
        rejected=predict_execution(plan,c,measured,required_clearance=.7)
        self.assertFalse(rejected['passed'])
        self.assertEqual(rejected['reason'],'predicted_execution_clearance')
        self.assertNotIn('duration',rejected)
        for bad in (-.1,float('nan')):
            with self.assertRaises(ValueError):
                predict_execution(plan,c,measured,required_clearance=bad)

    def test_serialized_map_resolution_has_the_same_geometry_and_identity(self):
        import copy
        g=OccupancyGrid();g.header.frame_id='map';g.info.width=g.info.height=20
        g.info.resolution=.05;g.info.origin.orientation.w=1.;g.data=[0]*400
        wire=copy.deepcopy(g);wire.info.resolution=float(np.float32(.05))
        c=json.loads((ROOT/'config/arena.json').read_text())
        c['mission_variant']='return'
        a,b=GridMap(g,c),GridMap(wire,c)
        self.assertEqual(a.map_id,b.map_id)
        np.testing.assert_allclose(a.clearance([[.3,.3],[.5,.5]]),b.clearance([[.3,.3],[.5,.5]]))

    def test_fast_circles_regenerate_for_changed_pole_spacing_and_orientation(self):
        for distance, angle in ((1.4, .4), (1.8, -1.1)):
            with self.subTest(distance=distance, angle=angle):
                c=json.loads((ROOT/'config/arena.json').read_text())
                c.update(json.loads((ROOT/'config/navigation-profiles.json').read_text())['candidate'])
                c.pop('optimized_route',None)
                c['route_family']='circles'
                lower=[.3,-.2]
                upper=[lower[0]+distance*math.cos(angle),lower[1]+distance*math.sin(angle)]
                rotation=angle-math.pi/2
                parking=[lower[0]-1.4*math.cos(rotation)+.35*math.sin(rotation),
                         lower[1]-1.4*math.sin(rotation)-.35*math.cos(rotation)]
                c.update(lower_pole=lower,upper_pole=upper,loop_radius=distance/2,
                         parking=parking,bounds=[-4.,6.,-4.,6.])
                g=OccupancyGrid();g.header.frame_id='map';g.info.width=g.info.height=200
                g.info.resolution=.05;g.info.origin.position.x=g.info.origin.position.y=-4.
                g.info.origin.orientation.w=1.
                yy,xx=np.mgrid[:200,:200];pixels=np.zeros((200,200),dtype=np.int8)
                for x,y in (lower,upper):
                    pixels[np.hypot(-4+(xx+.5)*.05-x,-4+(yy+.5)*.05-y)<c['pole_radius']]=100
                g.data=pixels.ravel().tolist()
                plan,_,report=compile_route(c,g)
                self.assertIsNone(report['optimized_route']['shape'])
                self.assertEqual(report['route_family'],'circles')
                self.assertGreaterEqual(report['candidates_evaluated'],2)
                self.assertEqual(len(plan.gates),10)
                self.assertEqual(plan.points[-1],tuple(c['parking']))
                self.assertTrue(report['execution_prediction']['passed'])
                self.assertEqual(len(report['response_predictions']),len(c['response_models'])+(len(c.get('response_stress_models',[])) if c.get('require_stress_models',True) else 0))
                self.assertTrue(all(p['passed'] for p in report['response_predictions']))

    def test_fast_tracking_prediction_matches_recorded_response_on_tight_loops(self):
        c=json.loads((ROOT/'config/arena.json').read_text())
        c.update(json.loads((ROOT/'config/navigation-profiles.json').read_text())['candidate'])
        c.update(max_speed=1.2,max_acceleration=2.5,lateral_acceleration=1.4,max_jerk=7.,terminal_deceleration=1.,response_delay=.2)
        c.update(lower_pole=[0.,0.],upper_pole=[0.,1.218],loop_radius=.609,
                 parking=[-1.4,-.35],entry_angle=math.pi,
                 bounds=[-3.,3.,-3.,4.],optimized_route=dict(shape=None,entry_angle=math.pi,transition_scale=1.25,speed_scale=.35))
        g=OccupancyGrid();g.header.frame_id='map';g.info.width=240;g.info.height=280
        g.info.resolution=.025;g.info.origin.position.x=g.info.origin.position.y=-3.
        g.info.origin.orientation.w=1.
        yy,xx=np.mgrid[:280,:240];pixels=np.zeros((280,240),dtype=np.int8)
        for x,y in (c['lower_pole'],c['upper_pole']):
            pixels[np.hypot(-3+(xx+.5)*.025-x,-3+(yy+.5)*.025-y)<c['pole_radius']]=100
        g.data=pixels.ravel().tolist();plan,_,_=compile_route(c,g)
        nominal=predict_execution(plan,c,GridMap(g,c))
        self.assertTrue(nominal['passed'],nominal)
        self.assertAlmostEqual(nominal['response_delay'],float(np.median(c['response_models'],axis=0)[0]))
        self.assertEqual(c['response_delay'],.2)
        for plant in c['response_models']:
            result=predict_execution(plan,c,GridMap(g,c),plant_response=plant)
            self.assertTrue(result['passed'],result)
            self.assertLess(result['max_tracking_error'],.06,result)

    def test_frozen_route_entry_avoids_full_bases_when_only_thin_poles_are_mapped(self):
        c=json.loads((ROOT/'config/arena.json').read_text())
        c.update(json.loads((ROOT/'config/navigation-profiles.json').read_text())['safe'])
        c.update(lower_pole=[0.,0.],upper_pole=[0.,1.2],loop_radius=.6,
                 parking=[-1.4,-.35],entry_angle=math.pi,pole_clearance_from_map=True,
                 bounds=[-3.,3.,-3.,4.],optimized_route=dict(shape=None,entry_angle=math.pi,transition_scale=1.25))
        g=OccupancyGrid();g.header.frame_id='map';g.info.width=240;g.info.height=280
        g.info.resolution=.025;g.info.origin.position.x=g.info.origin.position.y=-3.
        g.info.origin.orientation.w=1.
        yy,xx=np.mgrid[:280,:240];pixels=np.zeros((280,240),dtype=np.int8)
        for x,y in (c['lower_pole'],c['upper_pole']):
            pixels[np.hypot(-3+(xx+.5)*.025-x,-3+(yy+.5)*.025-y)<.03]=100
        g.data=pixels.ravel().tolist()
        plan,route,_=compile_route(c,g,[.39,.76])
        self.assertEqual(len(plan.gates),10)
        self.assertEqual(plan.points[0],(.39,.76))
        self.assertEqual(plan.points[-1],tuple(c['parking']))
        self.assertGreater(min(plan.clearance),0)

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
            self.assertEqual(cached['optimized_route']['shape'],frozen['optimized_route']['shape'])
        # Neither a fresh heartbeat nor a valid start pose makes an unknown map traversable.
        g.data = [-1] * len(g.data)
        with self.assertRaises(ValueError):
            compile_route(c, g)


if __name__ == "__main__":
    unittest.main()
