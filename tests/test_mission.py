"""Safety regressions for crossings, busy requests and asynchronous cancellation."""

import json, unittest, time
from pathlib import Path
from types import SimpleNamespace as N
from concurrent.futures import Future
from unittest.mock import patch, Mock
from std_srvs.srv import Trigger
from nav_msgs.msg import Path as RosPath
from intramural_mission.route import segments, projected_progress, primitives
from intramural_mission.mission import Mission
from intramural_mission.bypass import plan, winding
import numpy as np

_root = Path(__file__).resolve().parents[1]
_config_path = Path("/work/config/arena.json") if Path("/work/config/arena.json").exists() else _root / "config/arena.json"
C = json.loads(_config_path.read_text())


def ray_winding(path, p):
    result = 0
    for a, b in zip(path, path[1:]):
        cross = (b[0] - a[0]) * (p[1] - a[1]) - (p[0] - a[0]) * (b[1] - a[1])
        if a[1] <= p[1] < b[1] and cross > 0:
            result += 1
        if b[1] <= p[1] < a[1] and cross < 0:
            result -= 1
    return result


def completed(value):
    f = Future()
    f.set_result(value)
    return f


class MissionTests(unittest.TestCase):
    def test_entry_detour_closes_loop_and_returns_along_entered_path(self):
        route = segments(C)
        blocked = [(x,y) for x in np.arange(1.25,1.751,.05) for y in np.arange(.5,1.001,.05)]
        result = plan(C,route,0,4,route[0][1][4],blocked)
        self.assertIsNotNone(result)
        route[:len(result)] = [(result[0][0],route[0][1][:4]+result[0][1])] + result[1:]
        entered = route[0][1]
        self.assertGreater(np.linalg.norm(np.asarray(entered[-1])-np.asarray(segments(C)[0][1][-1])),.1)
        m = N(index=0,variant='figure8',bypassing=True,route=route,c=C,history=[],
            get_clock=lambda:N(now=lambda:N(nanoseconds=0)),make_path=lambda points:RosPath(),
            path_pub=Mock(),client=Mock(),accepted=lambda future:None)
        m.client.send_goal_async.return_value = completed(None)
        Mission.next_segment(m)
        self.assertEqual(m.route[-1][1],list(reversed(entered)))
        self.assertEqual(m.route[-2][1][-1],entered[-1])
        points = [p for _,path in m.route for p in path]
        self.assertEqual([winding(points,C[k]) for k in ('lower_pole','upper_pole')],[1,-1])

    def test_blocked_and_aborted_paths_keep_far_obstacle_under_observation(self):
        grid = N(header=N(frame_id='odom'), data=[0], info=N(width=1,height=1,resolution=.05,
            origin=N(position=N(x=0,y=0),orientation=N(w=1,x=0,y=0,z=0))))
        transform = N(transform=N(translation=N(x=0,y=0),rotation=N(w=1,x=0,y=0,z=0)))
        for state, abort, reach in [('RUNNING',None,1.2),('BLOCKED',None,5.0),('RUNNING',1,5.0)]:
            m = N(costmap=grid,costmap_at=time.monotonic(),tf=N(lookup_transform=lambda *a:transform),
                route=segments(C),index=0,progress=0,pose=[0,0],c=C,state=state,abort_at=abort)
            with patch('intramural_mission.mission.first_blocked') as check:
                Mission.obstacle_index(m)
                self.assertEqual(check.call_args.kwargs['lookahead'],reach)
        m.state = 'BLOCKED'
        with patch('intramural_mission.mission.first_blocked', side_effect=[None,17]) as check:
            self.assertEqual(Mission.obstacle_index(m),17)
            self.assertEqual(check.call_args.kwargs['threshold'],99)

    def test_obstacle_pause_closes_gate_and_preserves_segment(self):
        gate = []
        cancel = []
        handle = N(cancel_goal_async=lambda: cancel.append(True))
        m = N(state="RUNNING", index=3, blocked_count=0, goal_handle=handle,
              enabled=gate.append, clear_since=1, abort_at=2)
        Mission.pause_for_obstacle(m, 17)
        self.assertEqual((m.state, m.index, m.blocked_index), ("BLOCKED", 3, 17))
        self.assertEqual(gate, [False])
        self.assertEqual(cancel, [True])
        self.assertIsNone(m.abort_at)

    def test_cancel_from_blocked_without_action_is_terminal(self):
        gate = []
        m = N(state="BLOCKED", goal_handle=None, goal_pending=False,
              segment_result=None, enabled=gate.append,
              get_clock=lambda: N(now=lambda: N(nanoseconds=0)))
        result = Mission.cancel(m, Trigger.Request(), Trigger.Response())
        self.assertTrue(result.success)
        self.assertEqual(m.state, "CANCELLED")
        self.assertEqual(gate, [False])

    def test_reference_winding_and_distinct_crossing_visits(self):
        route = segments(C)
        path = [p for _, points in route for p in points]
        self.assertEqual(ray_winding(path, C["lower_pole"]), 1)
        self.assertEqual(ray_winding(path, C["upper_pole"]), -1)
        self.assertLess(
            __import__("math").dist(route[3][1][-1], route[7][1][-1]), 1e-10
        )
        self.assertNotEqual(route[3][0], route[7][0])

    def test_success_at_crossing_cannot_skip_unvisited_segment(self):
        route = segments(C)
        faults = []
        advances = []
        m = N(
            goal_handle=object(),
            state="RUNNING",
            route=route,
            index=3,
            pose=route[3][1][-1],
            progress=0,
            progress_fraction=0.0,
            c=C,
            fault=faults.append,
            next_segment=lambda: advances.append(True),
        )
        Mission.result(m, completed(N(status=4)))
        self.assertFalse(faults)  # Allow a fresh TF to arrive before final checking.
        m.result_at -= 1
        Mission.finish_segment(m)
        self.assertTrue(faults)
        self.assertFalse(advances)

    def test_delayed_pose_and_fractional_progress_finish_short_segment(self):
        route = primitives(C)
        advances = []
        m = N(
            goal_handle=object(),
            state="RUNNING",
            route=route,
            index=1,
            pose=[0.8, 0.70],
            progress_fraction=14.0,
            c=C,
            fault=lambda reason: self.fail(reason),
            next_segment=lambda: advances.append(True),
        )
        Mission.result(m, completed(N(status=4)))
        Mission.finish_segment(m)
        self.assertFalse(advances)
        m.pose = [0.8, 0.7248]
        m.progress_fraction = projected_progress(route[1][1], 14, m.pose)
        Mission.finish_segment(m)
        self.assertEqual(advances, [True])

    def test_cancel_racing_with_completed_segment_does_not_advance(self):
        m = N(
            goal_handle=object(),
            state="CANCELLING",
            get_clock=lambda: N(now=lambda: N(nanoseconds=0)),
        )
        Mission.result(m, completed(N(status=4)))
        self.assertEqual(m.state, "CANCELLED")

    def test_server_abort_does_not_advance(self):
        route = segments(C)
        faults = []
        advances = []
        m = N(
            goal_handle=object(),
            state="RUNNING",
            route=route,
            index=0,
            pose=route[0][1][-1],
            progress=len(route[0][1]) - 1,
            c=C,
            fault=faults.append,
            next_segment=lambda: advances.append(True),
        )
        Mission.result(m, completed(N(status=6)))
        self.assertFalse(faults)  # Await the next published grid before classifying obstruction.
        self.assertIsNotNone(m.abort_at)
        self.assertFalse(advances)

    def test_reset_while_running_or_cancelling_is_rejected(self):
        for state in ("RUNNING", "CANCELLING"):
            r = Mission.reset(
                N(state=state, goal_handle=None, goal_pending=False),
                Trigger.Request(),
                Trigger.Response(),
            )
            self.assertFalse(r.success)

    def test_fault_reset_waits_for_pending_goal_acceptance(self):
        r = Mission.reset(
            N(state="FAULT", goal_handle=None, goal_pending=True),
            Trigger.Request(),
            Trigger.Response(),
        )
        self.assertFalse(r.success)

    def test_start_away_from_parking_is_rejected(self):
        m = N(
            state="IDLE",
            navigator_active=True,
            pose=[3.5, 2.5],
            last_pose=time.monotonic(),
            ready_since=time.monotonic() - 2,
            tf_age=0.0,
            client=N(server_is_ready=lambda: True),
            c=C,
        )
        r = Mission.start(m, Trigger.Request(), Trigger.Response())
        self.assertFalse(r.success)

    def test_cancel_closes_gate_before_action_request(self):
        events = []
        m = N(
            state="RUNNING",
            enabled=lambda value: events.append(("enabled", value)),
            goal_handle=N(cancel_goal_async=lambda: events.append(("cancel", True))),
        )
        r = Mission.cancel(m, Trigger.Request(), Trigger.Response())
        self.assertTrue(r.success)
        self.assertEqual(events, [("enabled", False), ("cancel", True)])
        self.assertEqual(m.state, "CANCELLING")

    def test_cancel_pending_acceptance_cancels_late_handle(self):
        events = []
        handle = N(
            accepted=True,
            get_result_async=lambda: completed(N(status=5)),
            cancel_goal_async=lambda: events.append("cancel"),
        )
        m = N(
            state="CANCELLING",
            result=lambda f: None,
            fault=lambda reason: events.append(reason),
        )
        Mission.accepted(m, completed(handle))
        self.assertEqual(events, ["cancel"])

    def test_completed_cancel_result_does_not_cancel_cleared_handle(self):
        handle = N(accepted=True, get_result_async=lambda: completed(N(status=5)))
        m = N(
            state="CANCELLING",
            goal_handle=None,
            get_clock=lambda: N(now=lambda: N(nanoseconds=0)),
            fault=lambda reason: self.fail(reason),
        )
        m.result = lambda f: Mission.result(m, f)
        Mission.accepted(m, completed(handle))
        self.assertEqual(m.state, "CANCELLED")
        self.assertIsNone(m.goal_handle)

    def test_invalid_tangent_geometry_is_rejected(self):
        c = dict(C)
        c["upper_pole"] = [c["upper_pole"][0] + 0.1, c["upper_pole"][1]]
        with self.assertRaises(ValueError):
            segments(c)


if __name__ == "__main__":
    unittest.main()
