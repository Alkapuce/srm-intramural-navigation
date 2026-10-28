"""The route/compiler/tracker used by the ROS node, against delayed holonomic motion."""

import json
import math
import unittest
from pathlib import Path
import numpy as np
from intramural_mission.route import segments
from intramural_mission.trajectory import RoutePlan, Progress, OmniTracker
from intramural_mission.safety import executable_path
from intramural_mission.proximity import obstacle_on_path

ROOT = Path(__file__).resolve().parents[1]
BASE = json.loads((ROOT / "config/arena.json").read_text())
PROFILES = json.loads((ROOT / "config/navigation-profiles.json").read_text())


def rollout(c, delay=0.2, tau=0.15, gain=1.0, noise=0.0, seed=0, plan=None):
    plan = RoutePlan.build(segments(c), c) if plan is None else plan
    tracker = OmniTracker(plan, c)
    p = np.asarray(plan.points[0], dtype=float)
    v = np.zeros(2)
    rng = np.random.default_rng(seed)
    dt = 0.025
    pending = [np.zeros(2) for _ in range(round(delay / dt))]
    trace = []
    done = False
    max_error = 0.0
    winding = [0.0, 0.0]
    previous = p.copy()
    minimum_clearance = math.inf
    for tick in range(6000):
        observed = p + rng.normal(0, noise, 2)
        yaw = 0.6 + 0.1 * math.sin(tick * dt)
        body, report = tracker.step(observed, yaw, dt)
        co, si = math.cos(yaw), math.sin(yaw)
        sent = np.array([co * body[0] - si * body[1], si * body[0] + co * body[1]])
        pending.append(sent)
        u = pending.pop(0)
        v += (1 - math.exp(-dt / tau)) * (gain * u - v)
        p += v * dt
        max_error = max(max_error, report["error"])
        for i, key in enumerate(("lower_pole", "upper_pole")):
            pole = np.asarray(c[key])
            a = previous - pole
            b = p - pole
            winding[i] += math.atan2(np.cross(a, b), np.dot(a, b))
        previous = p.copy()
        minimum_clearance = min(
            minimum_clearance,
            *(
                np.linalg.norm(p - np.asarray(c[k]))
                - c["robot_radius"]
                - c["pole_radius"]
                for k in ("lower_pole", "upper_pole")
            ),
        )
        trace.append(
            dict(time=tick * dt, pose=p.tolist(), command=sent.tolist(), **report)
        )
        if report["done"]:
            for _ in range(round((delay + 3 * tau) / dt)):
                pending.append(np.zeros(2))
                u = pending.pop(0)
                v += (1 - math.exp(-dt / tau)) * (gain * u - v)
                p += v * dt
            done = True
            break
        if report["error"] > c["max_tracking_error"]:
            break
    return dict(
        done=done,
        time=(tick + 1) * dt,
        parking_error=float(np.linalg.norm(p - c["parking"])),
        max_tracking_error=max_error,
        minimum_pole_clearance=float(minimum_clearance),
        winding=[round(v / (2 * math.pi)) for v in winding],
        max_command=max(np.linalg.norm(x["command"]) for x in trace),
        phase=tracker.progress.phase,
    ), trace


class TrajectoryTests(unittest.TestCase):
    def test_unequal_adjacent_steps_cannot_hide_a_short_reversal(self):
        c=dict(BASE,**PROFILES['candidate'])
        for points in ([[0.,0.],[.06,.02],[.05,.02],[1.,.2]],
                       [[0.,0.],[.5,0.],[.01,0.],[1.,0.]],
                       [[.00154,-.01254],[.05616,.02601],[.04938,.02586]]):
            with self.subTest(points=points):
                plan=RoutePlan.build([('RETURN',points)],c)
                self.assertEqual(plan.speed[1],0.)

    def test_reversed_departure_tail_reaches_home_without_a_feedforward_equilibrium(self):
        from intramural_mission.response import Plant
        c=dict(BASE,**PROFILES['candidate'])
        points=[[-.0595,.0394],[-.0422,.0321],[-.025,.025],[-.0187,.0225],[-.0123,.0206],
            [-.0058,.0194],[.0009,.0191],[.0077,.0195],[.0146,.0206],[.0216,.0219],
            [.0287,.0233],[.0356,.0246],[.0426,.0254],[.0494,.0259],[.0562,.0260],
            [.00154,-.01254],[0.,0.]]
        plan=RoutePlan.build([('RETURN',points)],c);tracker=OmniTracker(plan,c)
        tracker.progress.s=plan.s[-1]-.077
        plant=Plant([.2357,.1357],.09,.05,acceleration_limit=3.5)
        for i in range(600):
            command,report=tracker.step(plant.pose,0.,.025,stamp=i*.025)
            plant.step(command)
            if report['done']:break
        self.assertTrue(report['done'],report)
        self.assertLess(np.linalg.norm(plant.pose),c['parking_tolerance'])

    def test_worker_roundtrip_preserves_cached_plan_and_tracker_motion(self):
        import pickle
        c=dict(BASE,**PROFILES['candidate'])
        plan=RoutePlan.build(segments(c),c)
        plan.sample(.5)
        restored=pickle.loads(pickle.dumps(plan))
        self.assertEqual(restored,plan)
        before,after=OmniTracker(plan,c),OmniTracker(restored,c)
        for index in range(20):
            pose=plan.sample(index*.01)[0]
            a,detail_a=before.step(pose,0.,.025,stamp=index*.025)
            b,detail_b=after.step(pose,0.,.025,stamp=index*.025)
            np.testing.assert_array_equal(a,b)
            self.assertEqual(detail_a,detail_b)

    def test_parking_holds_zero_through_jitter_and_uses_distinct_pose_window(self):
        c=dict(BASE,**PROFILES['candidate'],parking=[2.,0.],route_speed_scale=1.)
        plan=RoutePlan.build([('RETURN',[[float(x),0.] for x in np.linspace(0.,2.,81)])],c)
        for drifting in (False,True):
            tracker=OmniTracker(plan,c);tracker.progress.s=1.95
            tracker.step([1.95,0.],0.,.025,stamp=0.)
            for _ in range(20):
                command,detail=tracker.step([1.95,0.],0.,.025,stamp=0.)
                self.assertFalse(detail['done'])
            for index,x in enumerate(([1.93,1.95,1.96,1.94] if not drifting else [1.88,1.84,1.80,1.76]),1):
                command,detail=tracker.step([x,0.],0.,.025,stamp=index*.1)
                np.testing.assert_array_equal(command,[0.,0.])
            self.assertEqual(detail['done'],not drifting)
            if drifting:
                for index in range(5,10):
                    command,detail=tracker.step([1.76,0.],0.,.025,stamp=index*.1)
                    np.testing.assert_array_equal(command,[0.,0.])
                self.assertFalse(detail['done'])
                self.assertEqual(detail['parking_problem'],'PARKING_OUTSIDE_TOLERANCE')

    def test_parking_zero_precedes_rest_only_when_predicted_tail_fits(self):
        c=dict(BASE,**PROFILES['candidate'],parking=[2.,0.],route_speed_scale=1.)
        plan=RoutePlan.build([('RETURN',[[float(x),0.] for x in np.linspace(0.,2.,81)])],c)
        for position,velocity,brake in ((1.96,.2,True),(1.93,.2,True),
                                        (1.93,1.6,False),(1.89,.2,False)):
            tracker=OmniTracker(plan,c);tracker.progress.s=position
            tracker.previous_pose=np.array([position-velocity*.1,0.]);tracker.previous_stamp=0.
            tracker.velocity=np.array([velocity,0.]);tracker.command=np.array([.2,0.])
            command,detail=tracker.step([position,0.],0.,.025,stamp=.1)
            self.assertEqual(detail['parking_brake'],brake)
            self.assertFalse(detail['done'])
            if brake:
                np.testing.assert_array_equal(command,[0.,0.])
                self.assertGreater(np.linalg.norm(tracker.velocity),.06)

    def test_braking_keeps_the_commands_still_in_the_actuator_delay_queue(self):
        from intramural_mission.response import Plant,predict_motion
        from collections import deque
        c=dict(BASE,**PROFILES['candidate'],drive_acceleration_limit=3.5)
        plan=RoutePlan.build([('RETURN',[[float(x),0.]for x in np.linspace(0.,4.,161)])],c)
        history=[[-.2,1.5,0.],[-.025,1.5,0.],[0.,0.,0.]]
        _,_,brakes,times,_=predict_motion(plan,c,[0.,0.],[1.5,0.],[0.,0.],[0.,0.],0.,
            horizon=0.,models=[(.1,.05)],command_history=history)[0]
        # Independent delayed actuator integration, followed by a sustained zero command.
        pending=deque(np.array([1.5,0.])for _ in range(4))
        p=np.zeros(2);v=np.array([1.5,0.]);trace=[p.copy()]
        for _ in range(round(times[0,-1]/.025)):
            pending.append(np.zeros(2));u=pending.popleft()
            change=(1-math.exp(-.025/.05))*(u-v)
            change*=min(1.,3.5*.025/max(1e-9,np.linalg.norm(change)))
            v+=change;p+=v*.025;trace.append(p.copy())
        np.testing.assert_allclose(brakes[0],np.asarray(trace)[np.rint(times[0]/.025).astype(int)],atol=1e-10)
        self.assertGreater(brakes[0,-1,0],.44)

    def test_switch_prediction_keeps_the_old_route_until_the_new_route_can_activate(self):
        from intramural_mission.response import predict_motion
        c=dict(BASE,**PROFILES['candidate'],drive_acceleration_limit=3.5)
        old=RoutePlan.build([('RETURN',[[float(x),0.]for x in np.linspace(0.,4.,161)])],c)
        new=RoutePlan.build([('RETURN',[[0.,float(y)]for y in np.linspace(0.,4.,161)])],c)
        points,times,_,_,_=predict_motion(new,c,[0.,0.],[1.,0.],[1.,0.],[0.,0.],0.,
            horizon=.3,models=[(0.,.05)],previous_plan=old,previous_progress=0.,activation_delay=.1)[0]
        np.testing.assert_allclose(points[times<=.1,1],0.,atol=1e-12)
        self.assertGreater(points[4,0],.08)
        self.assertGreater(points[-1,1],.02)

    def test_terminal_zero_waits_for_the_force_limited_tail(self):
        c=dict(BASE,**PROFILES['candidate'],parking=[0.,0.],drive_acceleration_limit=3.5,
            route_speed_scale=1.)
        plan=RoutePlan.build([('RETURN',[[2.*x,1.5*x] for x in np.linspace(1.,0.,81)])],c)
        position=np.array([.0443710771,.0386735641]);velocity=np.array([-.4811517057,-.3744119686])
        tracker=OmniTracker(plan,c);tracker.progress.s=plan.s[-1]-float(np.linalg.norm(position))
        tracker.previous_pose=position-velocity*.1;tracker.previous_stamp=0.
        tracker.velocity=velocity;tracker.command=velocity
        command,detail=tracker.step(position,0.,.025,stamp=.1,pose_age=.04)
        self.assertFalse(detail['parking_brake'],detail)
        self.assertGreater(np.linalg.norm(command),0.)

    def test_terminal_stop_reserves_the_shared_localization_error_budget(self):
        from intramural_mission.response import Plant
        c=dict(BASE,**PROFILES['candidate'],parking=[0.,0.],drive_acceleration_limit=3.5,
            route_speed_scale=1.)
        plan=RoutePlan.build([('RETURN',[[2.*x,1.5*x] for x in np.linspace(1.,0.,81)])],c)
        for delay,tau in c['response_models']:
            with self.subTest(delay=delay,tau=tau):
                plant=Plant([-.077,-.024],delay,tau,acceleration_limit=3.5)
                tracker=OmniTracker(plan,c);tracker.progress.s=plan.s[-1]
                # A biased pose can be inside the goal circle while the actual body is outside.
                for i in range(200):
                    command,detail=tracker.step(plant.pose+[.009,.008],0.,.025,stamp=i*.025)
                    plant.step(command)
                    if detail['done']:break
                self.assertTrue(detail['done'],detail)
                for _ in range(40):plant.step([0.,0.])
                self.assertLess(np.linalg.norm(plant.pose),c['parking_tolerance'])

    def test_parking_budget_must_leave_a_positive_goal_region(self):
        c=dict(BASE,**PROFILES['candidate'])
        plan=RoutePlan.build(segments(c),c)
        for budget in (-.01,.08,.1,float('nan'),float('inf')):
            with self.assertRaises(ValueError):
                OmniTracker(plan,dict(c,minimum_clearance=budget))

    def test_terminal_preview_parks_with_recorded_response_delays(self):
        c=dict(BASE,**PROFILES['candidate'],parking=[2.,0.],route_speed_scale=1.)
        plan=RoutePlan.build([('RETURN',[[float(x),0.] for x in np.linspace(0.,2.,81)])],c)
        for delay,tau in ((.1,.05243267286),(.09168461485,.08902560959)):
            with self.subTest(delay=delay,tau=tau):
                result,trace=rollout(c,delay=delay,tau=tau,plan=plan)
                self.assertTrue(result['done'],result)
                self.assertLess(result['parking_error'],c['parking_tolerance'])
                self.assertTrue(any(t['parking_brake'] and np.linalg.norm(t['velocity_observed'])>.06
                                    for t in trace))

    def test_ten_hz_position_noise_does_not_create_repeated_lateral_corrections(self):
        c=dict(BASE,**PROFILES['candidate'])
        c['max_speed']=.3
        plan=RoutePlan.build([('RETURN',[[x,0.] for x in np.linspace(0.,6.,121)])],c)
        tracker=OmniTracker(plan,c);lateral=[]
        for tick in range(160):
            t=tick*.025;frame=tick//4;stamp=frame*.1
            pose=[.3*stamp,.005*(-1 if frame%2 else 1)]
            command,_=tracker.step(pose,0.,.025,t-stamp,stamp)
            if t>1.:lateral.append(command[1])
        self.assertLess(float(np.sqrt(np.mean(np.square(lateral)))),.035)

    def test_response_scaling_slows_curves_without_lowering_straight_cruise(self):
        c=dict(BASE,**PROFILES['candidate'])
        straight=[('RETURN',[[x,0.] for x in np.linspace(0.,12.,241)])]
        plan=RoutePlan.build(straight,dict(c,route_speed_scale=.35))
        self.assertAlmostEqual(max(plan.speed),c['max_speed'])
        full=RoutePlan.build(segments(c),c)
        scaled=RoutePlan.build(segments(c),dict(c,route_speed_scale=.35))
        curves=np.abs(full.curvature)>.5
        self.assertLess(max(np.array(scaled.speed)[curves]),max(np.array(full.speed)[curves])*.6)
        self.assertTrue(all(scaled.speed[i]>0 for _,i in scaled.gates[:-1]))

    def test_planned_parking_does_not_inherit_emergency_braking_distance(self):
        c = dict(BASE, **PROFILES["candidate"])
        route = [("RETURN", [[x, 0.] for x in np.linspace(0., 6., 121)])]
        old = dict(c)
        del old["terminal_deceleration"]
        before, after = RoutePlan.build(route, old), RoutePlan.build(route, c)
        self.assertGreater(after.sample(4.)[2], before.sample(4.)[2] * 1.5)
        self.assertAlmostEqual(after.sample(4.)[2], c["max_speed"])
        self.assertAlmostEqual(after.sample(5.8)[2], math.sqrt(2*c['terminal_deceleration']*.2))
        self.assertEqual(after.speed[-1], 0.)
        self.assertLessEqual(max(abs(np.diff(np.square(after.speed)) / (2*np.diff(after.s)))), c["max_acceleration"]+1e-7)
        self.assertEqual(c["braking_acceleration"], .2)
        for bad in (0., -1., float("nan")):
            with self.assertRaises(ValueError):
                RoutePlan.build(route, dict(c, terminal_deceleration=bad))

    def test_profile_and_order_do_not_stop_at_ordinary_phase_boundaries(self):
        c = dict(BASE, **PROFILES["safe"])
        plan = RoutePlan.build(segments(c), c)
        self.assertTrue(all(a < b for a, b in zip(plan.s, plan.s[1:])))
        self.assertEqual(len(plan.gates), 10)
        for _, i in plan.gates[:-1]:
            self.assertGreater(plan.speed[i], 0)
        self.assertEqual(plan.speed[-1], 0.0)
        ds = np.diff(plan.s)
        v = np.asarray(plan.speed)
        k = np.asarray(plan.curvature)
        self.assertLessEqual(max(v * v * np.abs(k)), c["lateral_acceleration"] + 1e-7)
        self.assertLessEqual(
            max(np.abs(np.diff(v * v) / (2 * ds))), c["max_acceleration"] + 1e-7
        )
        progress = Progress(plan)
        cross = plan.points[plan.gates[3][1]]
        progress.project(cross, 0.12)
        self.assertEqual(progress.phase, 0)

    def test_delayed_motion_completes_the_full_route_with_body_yaw(self):
        c = dict(BASE, **PROFILES["safe"])
        result, _ = rollout(c)
        self.assertTrue(result["done"], result)
        self.assertEqual(result["winding"], [1, -1])
        self.assertEqual(result["phase"], 9)
        self.assertLessEqual(result["max_command"], c["max_speed"] + 1e-7)
        self.assertLess(result["parking_error"], 0.12, result)

    def test_gain_error_delay_and_position_noise_preserve_ordered_completion(self):
        c=dict(BASE,**PROFILES['safe'],route_speed_scale=.5)
        for delay,tau,gain in ((.2,.2,.85),(.3,.2,1.15)):
            with self.subTest(delay=delay,gain=gain):
                result,_=rollout(c,delay=delay,tau=tau,gain=gain,noise=.003)
                self.assertTrue(result['done'],result)
                self.assertEqual(result['winding'],[1,-1])
                self.assertEqual(result['phase'],9)
                self.assertGreater(result['minimum_pole_clearance'],0.)
                self.assertLess(result['parking_error'],c['parking_tolerance'])

    def test_duplicate_geometry_and_finite_inputs(self):
        c = dict(BASE, **PROFILES["safe"])
        route = segments(c)
        route[0][1].insert(1, route[0][1][0])
        plan = RoutePlan.build(route, c)
        self.assertTrue(np.isfinite(plan.speed).all())
        with self.assertRaises(ValueError):
            RoutePlan.build([("ENTER", [[0, 0], [math.nan, 0], [1, 0]])], c)

    def test_zero_stop_clears_acceleration_without_rewinding_progress(self):
        c = dict(BASE, **PROFILES["safe"])
        tracker = OmniTracker(RoutePlan.build(segments(c), c), c)
        tracker.step(c["parking"], 0, 0.025)
        s = tracker.progress.s
        tracker.stop()
        self.assertEqual(tracker.command.tolist(), [0, 0])
        self.assertEqual(tracker.acceleration.tolist(), [0, 0])
        self.assertEqual(tracker.progress.s, s)

    def test_safety_sweep_includes_momentum_before_joining_the_reference(self):
        path = [[0.0, 0.3], [0.0, 0.6], [0.0, 1.0]]
        points = np.array([[0.2, 0.0]])
        self.assertFalse(obstacle_on_path(points, path, reach=0.8, radius=0.1))
        swept = executable_path(path, [1.0, 0.0], 0.3)
        self.assertTrue(obstacle_on_path(points, swept, reach=0.8, radius=0.1))
        self.assertFalse(
            obstacle_on_path(np.array([[0.4, -0.5]]), swept, reach=0.8, radius=0.1)
        )


if __name__ == "__main__":
    unittest.main()
