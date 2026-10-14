"""Check final command safety independently of Nav2 behavior."""

import math
import numpy as np
import unittest
from types import SimpleNamespace as N
from unittest.mock import patch
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud2, PointField
from array import array
from intramural_mission.adapter import Adapter


class MotionGuardTests(unittest.TestCase):
    def test_offset_lidar_points_are_checked_in_body_frame(self):
        cloud = PointCloud2(height=1, width=4, point_step=12, row_step=48)
        cloud.fields = [PointField(name=k, offset=i*4, datatype=7, count=1)
                        for i, k in enumerate(("x", "y", "z"))]
        cloud.data = array("B", np.array([[0, 0, 0], [.03, 0, 0], [.95, .3, .1],
                                         [float("nan"), 0, 0]], dtype="<f4").tobytes())
        guard = N(c={"lidar_pose": [.15, -.15, .22, 0, 0, 0], "robot_radius": .29},
                  mode="sensors", touch=lambda key: None)
        Adapter.lidar(guard, cloud)
        np.testing.assert_allclose(guard.raw_near_points, [[1.1, .15]], atol=1e-6)

    def guard(self):
        output = []
        cmd = Twist()
        cmd.linear.x = 1.0
        cmd.linear.y = 1.0
        cmd.angular.z = 1.0
        m = N(
            enabled=True, bypassing=False,
            cmd=cmd,
            last={k: 10.0 for k in ["command", "odom", "lidar", "imu"]},
            output=[0.0, 0.0],
            last_tick=9.975,
            c={"max_speed": 0.35, "max_acceleration": 0.6, "robot_radius": .29, "bypass_speed": .2},
            pub=N(publish=output.append),
            hazard_pub=N(publish=lambda m: None),
            raw_near_points=np.empty((0, 2)),
            last_motion_direction=(0.0, 0.0),
            diagnostic=N(publish=lambda m: None),
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
