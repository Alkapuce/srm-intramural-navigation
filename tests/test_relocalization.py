"""Global scan matching must recover a changed position AND heading, without a pose seed."""
from pathlib import Path
import sys
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts/internal'))
from relocalize import apply, match


class RelocalizationTest(unittest.TestCase):
    def test_partial_asymmetric_scene_with_changed_position_heading_and_outliers(self):
        rng = np.random.default_rng(3)
        # Unequal walls and several objects prevent the 180-degree ambiguity of an empty rectangle.
        walls = []
        for a, b in [([-4., -3.], [5., -3.]), ([5., -3.], [5., 4.]),
                     ([5., 4.], [-2., 4.]), ([-4., -3.], [-4., 1.])]:
            xy = np.asarray(a)+rng.random((1500, 1))*(np.asarray(b)-a)
            walls.append(np.c_[xy, rng.uniform(.1, 1.7, len(xy))])
        for x, y in [(-1., .8), (2., 2.), (3.5, -1.)]:
            angle = rng.uniform(-np.pi, np.pi, 800)
            walls.append(np.c_[x+.15*np.cos(angle), y+.15*np.sin(angle), rng.uniform(.1, 1.4, len(angle))])
        target = np.concatenate(walls)
        for yaw, position in [(-1.47, [-.1, .05, .08]), (.64, [2., -.7, -.04])]:
            expected = np.eye(4)
            expected[:3, :3] = Rotation.from_euler('xyz', [.015, -.01, yaw]).as_matrix()
            expected[:3, 3] = position
            visible = target[np.linalg.norm(target[:, :2]-position[:2], axis=1) < 6.5]
            source = apply(visible, np.linalg.inv(expected))+rng.normal(0, .006, visible.shape)
            source = np.vstack([source, rng.uniform([-4, -4, .2], [4, 4, 1.5], (150, 3))])
            normal = expected[:3, :3].T @ [0., 0., 1.]
            plane = [-normal[0]/normal[2], -normal[1]/normal[2], -position[2]/normal[2]]
            actual, report = match(target, source, [0., 0., 0.], plane)
            self.assertTrue(report['passed'], report)
            self.assertLess(np.linalg.norm(actual[:3, 3]-expected[:3, 3]), .03)
            self.assertLess(Rotation.from_matrix(actual[:3, :3]@expected[:3, :3].T).magnitude(), .015)

    def test_empty_scene_cannot_produce_a_pose(self):
        with self.assertRaises(ValueError):
            match(np.zeros((0, 3)), np.zeros((0, 3)), [0., 0., 0.], [0., 0., 0.])


if __name__ == '__main__':
    unittest.main()
