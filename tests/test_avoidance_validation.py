"""Check the independent truth oracle with skewed clocks and synthetic teleports."""
import importlib.util
from pathlib import Path
import unittest

source=Path(__file__).resolve().parents[1]/'scripts/internal/avoidance_gazebo.py'
spec=importlib.util.spec_from_file_location('avoidance_physical_oracle',source)
oracle=importlib.util.module_from_spec(spec);spec.loader.exec_module(oracle)


class PhysicalOracleTests(unittest.TestCase):
    config=dict(robot_radius=.29,obstacle_size=[.5,.5,1.])

    def test_speed_loss_uses_physical_source_time(self):
        summary=oracle.motion_timing([[0.,0.,0.],[1.,0.,0.],[1.5,.05,0.],[2.,1.05,0.]])
        self.assertAlmostEqual(summary['stopped_seconds'],1.)
        self.assertAlmostEqual(summary['below_point_two_mps_seconds'],1.5)
        self.assertAlmostEqual(summary['peak_measured_speed_mps'],2.)
        self.assertAlmostEqual(summary['en_route_stopped_seconds'],0.)
        summary=oracle.motion_timing([[0.,0.,0.],[1.,0.,0.],[1.5,.5,0.],[1.75,.5,0.],[2.25,1.,0.],[3.25,1.,0.]])
        self.assertAlmostEqual(summary['en_route_stopped_seconds'],.25)
        self.assertAlmostEqual(summary['stopped_seconds'],2.25)
        self.assertEqual(oracle.motion_timing([[0.,0.,0.],[1.,0.,0.]])['en_route_stopped_seconds'],0.)

    def test_asynchronous_truth_is_interpolated_at_the_same_source_time(self):
        robot=[[0.,0.,0.],[1.,1.,0.]]
        actor=[[.1,.1,1.],[.9,.9,1.]]
        self.assertAlmostEqual(oracle.aligned_clearance(robot,actor,self.config),.46,delta=.001)

    def test_contact_between_truth_samples_is_detected(self):
        self.assertLess(oracle.aligned_clearance([[0.,-1.,0.],[1.,1.,0.]],
            [[0.,0.,0.],[1.,0.,0.]],self.config),0.)

    def test_scene_teleport_does_not_invent_a_sweep_through_the_robot(self):
        self.assertGreater(oracle.aligned_clearance([[0.,0.,0.],[1.,0.,0.]],
            [[0.,2.,0.],[.49,2.,0.],[.5,-2.,0.],[1.,-2.,0.]],self.config),1.4)
