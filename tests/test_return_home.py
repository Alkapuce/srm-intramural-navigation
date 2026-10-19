"""Recorded recovery reverses actual poses and rejects gaps before starting a launch."""
import importlib.util
from pathlib import Path
import unittest

spec=importlib.util.spec_from_file_location('prepare_return',Path(__file__).resolve().parents[1]/'scripts/internal/prepare-return.py')
module=importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class ReturnHomeTests(unittest.TestCase):
    def test_reverse_recorded_run_and_reject_missing_pose_intervals(self):
        points=[[0.,0.],[.1,0.],[.2,.1]]
        records=[{'mission':{'run':1,'state':'RUNNING','pose':p}} for p in points]
        path=module.recorded_path(records,[0.,0.])
        self.assertEqual(path[:3],list(reversed(points)))
        self.assertEqual(path[-1],[0.,0.])
        records[-1]['mission']['pose']=[1.,1.]
        with self.assertRaises(ValueError):module.recorded_path(records,[0.,0.])


if __name__=='__main__':unittest.main()
