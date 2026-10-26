"""Failure cases: association, stale scans, crossing, topology and moving splice."""
import unittest
import numpy as np
from intramural_mission.avoidance import ObstacleTracker, collision, splice
from intramural_mission.trajectory import RoutePlan


class AvoidanceTests(unittest.TestCase):
    def test_voxel_compression_preserves_real_point_support(self):
        tracker = ObstacleTracker()
        self.assertEqual(len(tracker.update([[1.,0.]],1.,[3])),1)
    def test_persistent_obstacle_produces_executable_detour(self):
        import json
        from pathlib import Path
        from nav_msgs.msg import OccupancyGrid
        from intramural_mission.avoidance import DetourPlanner
        c = json.loads((Path(__file__).resolve().parents[1]/'config/arena.json').read_text())
        c.update(mission_variant='return', max_speed=1., max_acceleration=2., lateral_acceleration=1.,
                 terminal_deceleration=1., bounds=[-2.,6.,-3.,3.], pole_clearance_from_map=True,
                 lower_pole=[-10.,-10.], upper_pole=[10.,10.])
        grid = OccupancyGrid(); grid.header.frame_id='map'; grid.info.resolution=.05
        grid.info.width=160;grid.info.height=120;grid.info.origin.position.x=-2.;grid.info.origin.position.y=-3.
        grid.info.origin.orientation.w=1.;grid.data=[0]*19200
        plan = RoutePlan.build([('RETURN', [[x,0.] for x in np.linspace(0,4,81)])],c)
        planner = DetourPlanner(c,grid)
        points = [[1.2+x,y] for x in (-.1,0.,.1) for y in (-.1,0.,.1)]
        for i in range(6):
            planner.observations(points,i*.1)
        result = planner.solve(dict(plan=plan,progress=0.,pose=[0.,0.],velocity=[0.,0.],
            command=[0.,0.],acceleration=[0.,0.],points=points,stamp=.6))
        self.assertIsNotNone(result['plan'], result)
        self.assertGreater(max(abs(y) for x,y in result['plan'].points), .4)

    def test_motion_and_missing_observation_keep_predicted_occupancy(self):
        tracker = ObstacleTracker()
        for i in range(12):
            cloud = np.array([[i*.1+x, y] for x in (-.03, 0, .03) for y in (-.03, 0, .03)])
            tracks = tracker.update(cloud, i*.1)
        self.assertEqual(len(tracks), 1)
        self.assertAlmostEqual(tracks[0]['velocity'][0], 1., delta=.15)
        self.assertTrue(tracker.update([], 1.2))
        self.assertEqual(tracker.update([], 1.2), tracker.snapshot(1.2))
        self.assertEqual(tracker.update([], 2.1), [])

    def test_crossing_checks_arrival_time_and_intermediate_sweep(self):
        track = dict(center=[.5, -.5], velocity=[0., 1.], radius=.06,
                     covariance=np.zeros((4, 4)).tolist(), age=0.)
        self.assertTrue(collision([[0, 0], [1, 0]], [0., 1.], [track], .1))
        track['velocity'] = [0., -1.]
        self.assertFalse(collision([[0, 0], [1, 0]], [0., 1.], [track], .1))

    def test_invalid_or_old_scan_does_not_advance_tracking_clock(self):
        tracker = ObstacleTracker()
        cloud = np.array([[0., 0.], [.02, 0], [0, .02]])
        tracker.update(cloud, 1.)
        with self.assertRaises(ValueError):
            tracker.update([[float('nan'), 0]], 1.1)
        self.assertEqual(tracker.stamp, 1.)
        self.assertEqual(tracker.update(cloud, .9), tracker.snapshot(1.))

    def test_splice_keeps_completed_gates_and_nonzero_boundary_speed(self):
        limits = dict(max_speed=1., max_acceleration=2., lateral_acceleration=1.,
                      braking_acceleration=.5, terminal_deceleration=1., path_spacing=.05)
        route = [('ENTER', [[x, 0.] for x in np.linspace(0, 1, 21)]),
                 ('RETURN', [[x, 0.] for x in np.linspace(1, 3, 41)])]
        plan = RoutePlan.build(route, limits)
        bridge = [[x, .15*np.sin((x-1.2)*np.pi/.8)**2] for x in np.linspace(1.2, 2., 25)]
        updated, start_s = splice(plan, 1.2, bridge, 2., limits, lambda p: np.ones(len(p)), .5)
        self.assertEqual(updated.gates[0], plan.gates[0])
        self.assertEqual(updated.points[-1], plan.points[-1])
        self.assertAlmostEqual(updated.sample(start_s)[2], .5, delta=.08)
        self.assertGreater(start_s, updated.s[updated.gates[0][1]])


if __name__ == '__main__':
    unittest.main()
