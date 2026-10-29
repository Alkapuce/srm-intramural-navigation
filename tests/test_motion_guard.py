"""Check final command safety independently of Nav2 behavior."""

import math
import json
import numpy as np
import unittest
from types import SimpleNamespace as N
from unittest.mock import patch
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud2, PointField
from array import array
from intramural_mission.adapter import Adapter


class MotionGuardTests(unittest.TestCase):
    def test_cached_compensation_matches_each_point_and_invalidates_on_new_pose_or_scan(self):
        from collections import deque
        from intramural_mission.safety import compensate_motion
        from intramural_mission.proximity import obstacle_near
        guard,_=self.guard();guard.atomic_command=True;guard.enabled=False
        guard.c.update(navigation_engine='trajectory',response_delay=.1,response_tau=.05)
        guard.odom_history=deque([(9.8,0.,0.,0.),(10.,.1,.04,.1)])
        guard.motion_history=deque();guard.body_velocity=np.array([.5,.2])
        guard.raw_near_points=np.array([[1.5,.3],[2.,.2],[2.5,-.5]])
        guard.raw_stamp=9.95;guard.raw_point_times=np.array([9.94,9.96,9.99])
        stamp=[10.];guard.get_clock=lambda:N(now=lambda:N(nanoseconds=round(stamp[0]*1e9)))
        samples=[]
        def observe(points,*args,**kwargs):
            samples.append(np.asarray(points).copy())
            return obstacle_near(points,*args,**kwargs)
        with patch('intramural_mission.adapter.obstacle_near',side_effect=observe):
            for i,t in enumerate((10.,10.02,10.04,10.06)):
                stamp[0]=t
                if i==2:guard.odom_history.append((10.04,.13,.06,.16))
                if i==3:
                    guard.raw_near_points+=.1;guard.raw_stamp=10.02
                    guard.raw_point_times=np.array([10.01,10.02,10.03]);guard.last['lidar']=10.01
                expected=compensate_motion(guard.raw_near_points,guard.raw_point_times,guard.odom_history,t)
                self.tick(guard)
                np.testing.assert_allclose(samples[-1],expected,atol=1e-12,rtol=0.)
                self.assertEqual(guard.raw_compensation_cache[1],(10.,10.,10.04,10.06)[i])
            guard.odom_history.clear();self.tick(guard)
            self.assertEqual(guard.raw_motion_problem,'ODOMETRY_HISTORY_WARMUP')
            self.assertEqual(guard.output,[0.,0.])

    def test_compensated_scan_age_does_not_hold_a_stale_tangent_through_a_turn(self):
        from collections import deque
        guard,output=self.guard();guard.atomic_command=True
        guard.c.update(navigation_engine='trajectory',max_speed=2.,max_acceleration=3.5,
            minimum_clearance=.02,response_delay=.1,response_tau=.09,
            drive_acceleration_limit=3.5,raw_collision_follow_path=True)
        guard.c.pop('drive_acceleration_limit')
        guard.body_velocity=np.array([0.,-1.2])
        guard.odom_history=deque([(9.6,0.,0.,0.),(10.,0.,-.48,0.)])
        guard.motion_history=deque([(9.6,0.,-1.2),(9.9,-.4,-1.4)])
        guard.cmd.linear.x=-.73;guard.cmd.linear.y=-1.41;guard.output=[-.73,-1.41]
        guard.body_path=lambda:[[0.,0.],[-.4,-.6],[-.9,-.5]]
        guard.collision_pose_stamp=10.
        guard.raw_stamp=9.73;guard.raw_point_times=np.array([9.73])
        guard.raw_near_points=np.array([[-.2,-1.244]])
        self.tick(guard)
        self.assertFalse(guard.hazard,guard.diagnostics[-1].data)
        self.assertGreater(np.linalg.norm(guard.output),0.)
        self.assertGreater(json.loads(guard.diagnostics[-1].data)['collision_reach_m'],.7)
        guard.raw_near_points=np.array([[-.05,-.5]])
        guard.raw_compensation_cache=None;self.tick(guard)
        self.assertTrue(guard.hazard)
        self.assertEqual(output[-1],Twist())

    def test_queued_turn_stops_even_when_the_ordered_ray_is_clear(self):
        from collections import deque
        guard,output=self.guard()
        guard.atomic_command=True
        guard.c.update(navigation_engine='trajectory',max_speed=2.,max_acceleration=3.5,
            minimum_clearance=.02,response_delay=.1,response_tau=.09,
            drive_acceleration_limit=3.5,raw_collision_follow_path=True)
        guard.body_velocity=np.array([0.,2.]);guard.odom_history=deque([(10.,0.,0.,0.)])
        guard.motion_history=deque([(9.7,0.,2.),(9.91,-.8,1.8),(9.96,-1.2,1.6)])
        guard.cmd.linear.x=-1.4;guard.cmd.linear.y=1.4;guard.output=[-1.4,1.4]
        guard.body_path=lambda:[[0.,0.],[0.,3.]]
        guard.raw_near_points=np.array([[-.5,.9]])
        self.tick(guard)
        self.assertTrue(guard.hazard)
        self.assertEqual(output[-1],Twist())

    def test_newer_run_recovers_when_the_starting_snapshot_was_stale(self):
        guard=N(atomic_command=True,snapshot_stamp=10.,snapshot_run=1,mission_state='CANCELLING',
            mission_context=('old','route'),output=[0.,0.],
            get_clock=lambda:N(now=lambda:N(nanoseconds=10_400_000_000)),touch=lambda k:None)
        def snapshot(run,sequence,stamp,state,pose_stamp=None):
            return N(data=json.dumps(dict(run_id=run,run=sequence,route_id='route',stamp=stamp,
                state=state,phase='ENTER',points=[[0.,0.],[1.,0.]],pose=[0.,0.,0.],
                pose_stamp=stamp if pose_stamp is None else pose_stamp,command=[.3,0.] if state=='RUNNING' else [0.,0.])))
        Adapter.receive_trajectory(guard,snapshot('new',2,10.1,'STARTING',9.))
        self.assertEqual(guard.snapshot_run,1)
        Adapter.receive_trajectory(guard,snapshot('new',2,10.2,'RUNNING'))
        self.assertEqual(guard.mission_context,('new','route'))
        self.assertEqual(guard.snapshot_run,2)
        self.assertEqual(guard.cmd.linear.x,.3)
        for state in ('RUNNING','DONE'):
            guard.mission_state=state
            for run,sequence in [('old',1),('other',2),('invalid',True),('invalid',-1)]:
                Adapter.receive_trajectory(guard,snapshot(run,sequence,10.3,'STARTING'))
                self.assertEqual(guard.mission_context,('new','route'))
                self.assertEqual(guard.cmd.linear.x,.3)

    def test_atomic_context_ignores_delayed_diagnostics_and_zero_start_hands_over_runs(self):
        guard=N(atomic_command=True,snapshot_stamp=10.,mission_state='RUNNING',
            mission_phase='LOWER',mission_context=('old','old-route'),output=[.2,0.],
            get_clock=lambda:N(now=lambda:N(nanoseconds=10_400_000_000)),touch=lambda k:None)
        from std_msgs.msg import String
        def snapshot(run,stamp,state,command):
            return String(data=json.dumps(dict(run_id=run,route_id=run+'-route',stamp=stamp,
                state=state,phase='ENTER',points=[[0.,0.],[1.,0.]],pose=[0.,0.,0.],
                pose_stamp=stamp,command=command)))
        Adapter.receive_trajectory(guard,snapshot('new',10.2,'STARTING',[0.,0.]))
        self.assertEqual(guard.mission_context,('new','new-route'))
        self.assertEqual(guard.output,[0.,0.])
        Adapter.receive_trajectory(guard,snapshot('old',10.25,'RUNNING',[.8,0.]))
        self.assertEqual(guard.mission_context,('new','new-route'))
        self.assertEqual(guard.cmd.linear.x,0.)
        Adapter.receive_trajectory(guard,snapshot('new',10.3,'RUNNING',[.3,0.]))
        Adapter.receive_mission_status(guard,String(data=json.dumps(dict(
            run_id='old',route_id='old-route',state='CANCELLED',phase='UPPER'))))
        self.assertEqual(guard.mission_context,('new','new-route'))
        self.assertEqual(guard.mission_state,'RUNNING')
        self.assertEqual(guard.mission_phase,'ENTER')
        Adapter.receive_trajectory(guard,snapshot('old',10.35,'RUNNING',[.8,0.]))
        self.assertEqual(guard.cmd.linear.x,.3)
        legacy=N(atomic_command=False)
        Adapter.receive_mission_status(legacy,String(data=json.dumps(dict(
            run_id='legacy',route_id='route',state='BLOCKED',phase='LOWER'))))
        self.assertEqual(legacy.mission_context,('legacy','route'))
        self.assertEqual(legacy.mission_state,'BLOCKED')

    def test_delayed_old_snapshot_cannot_clear_a_newer_atomic_command(self):
        import json
        fresh_path=object();command=Twist();command.linear.x=.3
        guard=N(atomic_command=True,snapshot_stamp=10.,mission_state='RUNNING',
            mission_context=('current','new'),cmd=command,collision_path=fresh_path,
            get_clock=lambda:N(now=lambda:N(nanoseconds=10_000_000_000)))
        for run,stamp in [('old',10.1),('current',9.9)]:
            message=N(data=json.dumps(dict(run_id=run,route_id='old',stamp=stamp,
                points=[[0.,0.],[1.,0.]],pose=[0.,0.,0.],pose_stamp=8.,command=[0.,0.])))
            Adapter.receive_trajectory(guard,message)
            self.assertIs(guard.collision_path,fresh_path)
            self.assertIs(guard.cmd,command)

    def test_offset_lidar_points_are_checked_in_body_frame(self):
        cloud = PointCloud2(height=1, width=4, point_step=12, row_step=48)
        cloud.fields = [PointField(name=k, offset=i*4, datatype=7, count=1)
                        for i, k in enumerate(("x", "y", "z"))]
        cloud.data = array("B", np.array([[0, 0, 0], [.03, 0, 0], [.95, .3, .1],
                                         [float("nan"), 0, 0]], dtype="<f4").tobytes())
        guard = N(atomic_command=False,c={"lidar_pose": [.15, -.15, .22, 0, 0, 0], "robot_radius": .29},
                  mode="sensors", touch=lambda key: None,get_clock=lambda:N(now=lambda:N(nanoseconds=0)))
        Adapter.lidar(guard, cloud)
        np.testing.assert_allclose(guard.raw_near_points, [[1.1, .15]], atol=1e-6)

    def guard(self):
        output = []
        diagnostics = []
        cmd = Twist()
        cmd.linear.x = 1.0
        cmd.linear.y = 1.0
        cmd.angular.z = 1.0
        m = N(
            get_clock=lambda:N(now=lambda:N(nanoseconds=10_000_000_000)),
            atomic_command=False,
            enabled=True, bypassing=False, motion_permitted=True,
            cmd=cmd,
            last={k: 10.0 for k in ["command", "odom", "lidar", "imu"]},
            output=[0.0, 0.0],
            last_tick=9.975,
            c={"max_speed": 0.35, "max_acceleration": 0.6, "robot_radius": .29, "bypass_speed": .2},
            pub=N(publish=output.append),
            hazard_pub=N(publish=lambda m: None),
            raw_near_points=np.empty((0, 2)),
            last_motion_direction=(0.0, 0.0),
            diagnostic=N(publish=diagnostics.append),
            diagnostics=diagnostics,
            mode="sensors",
        )
        return m, output

    def tick(self, m):
        with patch("intramural_mission.adapter.time.monotonic", return_value=10.0):
            Adapter.tick(m)

    def test_any_stale_required_input_stops_immediately(self):
        for k in ["command", "odom", "lidar", "imu"]:
            m, output = self.guard()
            m.output = [0.2, 0.1]
            m.last[k] = 8.0
            self.tick(m)
            self.assertEqual(m.output, [0.0, 0.0], k)
            self.assertEqual(output[-1], Twist())

    def test_vehicle_standby_interlock_rejects_fresh_enabled_nonzero_command(self):
        m, output = self.guard()
        m.motion_permitted = False
        m.output = [.2, .1]
        self.tick(m)
        self.assertEqual(output[-1], Twist())

    def test_disabled_and_nonfinite_stop_immediately(self):
        for value in [float("nan"), float("inf"), -float("inf")]:
            m, output = self.guard()
            m.cmd.linear.x = value
            m.output = [0.2, 0.1]
            self.tick(m)
            self.assertEqual(output[-1], Twist())
        m, output = self.guard()
        m.enabled = False
        self.tick(m)
        self.assertEqual(output[-1], Twist())

    def test_acceleration_vector_limit_and_disabled_yaw(self):
        m, output = self.guard()
        self.tick(m)
        self.assertAlmostEqual(math.hypot(*m.output), 0.6 * 0.025)
        self.assertEqual(output[-1].angular.z, 0.0)
        self.assertEqual(json.loads(m.diagnostics[-1].data)['stamp'], 10.)

    def test_bypass_speed_and_close_obstacle_stop(self):
        m, output = self.guard()
        m.bypassing = True
        m.output = [.2, 0]
        self.tick(m)
        self.assertLessEqual(math.hypot(*m.output), .2)
        m.raw_near_points = np.array([[.42, 0]])
        self.tick(m)
        self.assertEqual(output[-1], Twist())

    def test_combined_speed_limit(self):
        m, output = self.guard()
        v = 0.35 / math.sqrt(2)
        m.output = [v, v]
        self.tick(m)
        self.assertAlmostEqual(
            math.hypot(output[-1].linear.x, output[-1].linear.y), 0.35
        )
