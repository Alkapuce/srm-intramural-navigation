"""Check the independent truth oracle with skewed clocks and synthetic teleports."""
import importlib.util
from pathlib import Path
import unittest

source=Path(__file__).resolve().parents[1]/'scripts/internal/avoidance_gazebo.py'
spec=importlib.util.spec_from_file_location('avoidance_physical_oracle',source)
oracle=importlib.util.module_from_spec(spec);spec.loader.exec_module(oracle)


class PhysicalOracleTests(unittest.TestCase):
    config=dict(robot_radius=.29,obstacle_size=[.5,.5,1.])

    def test_physical_matrix_selection_preserves_formal_order_and_subsets(self):
        self.assertEqual(oracle.matrix_indices(60),[47,*range(47),*range(48,60)])
        self.assertEqual(oracle.matrix_indices(10,baseline=True),list(range(10)))
        self.assertEqual(oracle.matrix_indices(60,selected=[10,40,56]),[10,40,56])
        for selected in ([],[10,10],[-1],[62]):
            with self.assertRaises(ValueError):oracle.matrix_indices(60,selected=selected)
        with self.assertRaises(ValueError):oracle.matrix_indices(60,baseline=True,selected=[10])

    def test_entry_and_return_must_use_the_same_ordered_channel(self):
        config=dict(parking=[0.,0.],parking_tolerance=.08,max_tracking_error=.3)
        trace=[[0.,0.,0.],[1.,1.,1.],[2.,2.,1.],[3.,2.,1.],[4.,1.,1.],[5.,0.,0.]]
        history=[dict(sim_time=0.)for _ in range(10)];history[1]['sim_time']=2.;history[9]['sim_time']=3.
        self.assertTrue(oracle.channel_agreement(trace,history,config)['passed'])
        trace[4][2]=-1.
        self.assertFalse(oracle.channel_agreement(trace,history,config)['passed'])

    def test_loop_order_rejects_swapped_loops_with_the_same_final_winding(self):
        import numpy as np
        lo=np.column_stack((np.cos(np.linspace(np.pi/2,5*np.pi/2,5)),
            -1+np.sin(np.linspace(np.pi/2,5*np.pi/2,5))))
        hi=np.column_stack((np.cos(np.linspace(-np.pi/2,-5*np.pi/2,5)),
            1+np.sin(np.linspace(-np.pi/2,-5*np.pi/2,5))))
        points=np.vstack((lo,hi[1:]));swapped=np.vstack((hi,lo[1:]))
        nominal=dict(points=points.tolist(),gates=[['stage',min(i+1,8)] for i in range(10)])
        history=[dict(segment=i,sim_time=float(i)) for i in range(10)]
        config=dict(lower_pole=[0.,-1.],upper_pole=[0.,1.])
        self.assertTrue(oracle.ordered_turns(np.column_stack((np.arange(9),points)),history,nominal,config)['passed'])
        self.assertFalse(oracle.ordered_turns(np.column_stack((np.arange(9),swapped)),history,nominal,config)['passed'])

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

    def test_actual_box_orientation_and_rotation_sweep(self):
        import math
        actor=[[0.,0.,0.,math.pi/4],[1.,0.,0.,math.pi/4]]
        self.assertAlmostEqual(oracle.aligned_clearance([[0.,.4,.4],[1.,.4,.4]],actor,self.config),.025685,delta=.001)
        self.assertLess(oracle.aligned_clearance([[0.,.55,0.],[1.,.55,0.]],actor,self.config),0.)
        self.assertLess(oracle.aligned_clearance([[0.,.6,0.],[1.,.6,0.]],
            [[0.,0.,0.,0.],[1.,0.,0.,math.pi/2]],self.config),0.)

    def test_scene_teleport_does_not_invent_a_sweep_through_the_robot(self):
        self.assertGreater(oracle.aligned_clearance([[0.,0.,0.],[1.,0.,0.]],
            [[0.,2.,0.],[.49,2.,0.],[.5,-2.,0.],[1.,-2.,0.]],self.config),1.4)
