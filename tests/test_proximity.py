"""Raw scan protection must detect new boxes without treating tangential poles as a block."""

import unittest
import numpy as np
from intramural_mission.proximity import obstacle_near, lidar_to_body


class ProximityTests(unittest.TestCase):
    def test_braking_capsule_rejects_square_corner_false_positive(self):
        self.assertFalse(obstacle_near(np.array([[.5,.35]]),(1,0),radial=.39,forward=.463,lateral=.39))
        self.assertTrue(obstacle_near(np.array([[.44,0]]),(1,0),radial=.39,forward=.463,lateral=.39))
    def test_mount_translation_and_rotation_are_applied_once(self):
        points = lidar_to_body(np.array([[1., 0., 0.]]), [.15, -.15, .22, 0, 0, np.pi/2])
        np.testing.assert_allclose(points, [[.15, .85, .22]], atol=1e-9)

    def test_box_ahead_is_detected_before_contact(self):
        self.assertTrue(obstacle_near(np.array([[1.1, .15]]), (1, 0)))
        self.assertTrue(obstacle_near(np.array([[.3, -1.05]]), (0, -1)))

    def test_tangential_pole_and_distant_wall_do_not_block(self):
        self.assertFalse(obstacle_near(np.array([[1.35, 0], [2, 0]]), (0, -1)))
        self.assertFalse(obstacle_near(np.array([[2, 0]]), (1, 0)))
        self.assertFalse(obstacle_near(np.array([[1.35, 0], [0, 1.0]]), (1, 0)))

    def test_no_heading_still_blocks_immediate_collision(self):
        self.assertTrue(obstacle_near(np.array([[.7, .1]]), (0, 0)))
        self.assertFalse(obstacle_near(np.empty((0, 2)), (1, 0)))
        self.assertFalse(obstacle_near(np.zeros((1000, 2)), (1, 0)))


if __name__ == "__main__":
    unittest.main()
