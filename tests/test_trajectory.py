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
