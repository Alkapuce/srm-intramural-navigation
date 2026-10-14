"""Ordered-segment obstacle gate and coordinate transforms."""

import unittest

from intramural_mission.hazards import first_blocked


class HazardTests(unittest.TestCase):
    def test_obstacle_ahead_blocks_but_behind_or_other_segment_does_not(self):
        width = height = 60
        resolution = 0.1
        grid = [0] * (width * height)
        grid[20 * width + 20] = 100
        points = [(i / 10, 2.0) for i in range(10, 42)]
        self.assertIsNotNone(first_blocked(points, 0, (1, 2), grid, width, height,
                                           resolution, (0, 0, 0), radius=.2))
        self.assertIsNone(first_blocked(points, 20, (3, 2), grid, width, height,
                                        resolution, (0, 0, 0), radius=.2))
        self.assertIsNone(first_blocked([(i / 10, 4) for i in range(10, 42)], 0,
                                        (1, 4), grid, width, height, resolution,
                                        (0, 0, 0), radius=.2))

    def test_map_to_rolling_odom_frame(self):
        width = height = 60
        grid = [0] * (width * height)
        grid[20 * width + 30] = 100
        # Map path x=2 becomes odom x=3 after map->odom +1 m.
        self.assertIsNotNone(first_blocked([(2, 2)], 0, (1.9, 2), grid, width,
                                            height, .1, (0, 0, 0), (1, 0, 0),
                                            radius=.1))

    def test_invalid_grid_rejected(self):
        with self.assertRaises(ValueError):
            first_blocked([], 0, (0, 0), [], 10, 10, .1, (0, 0, 0))


if __name__ == "__main__":
    unittest.main()
