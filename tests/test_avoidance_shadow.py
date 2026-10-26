"""Recorded-component frame checks; raw replay uses live localization TF instead."""
import importlib.util
import math
from pathlib import Path
import unittest
from nav_msgs.msg import Odometry

source=Path(__file__).resolve().parents[1]/'scripts/internal/avoidance_shadow.py'
spec=importlib.util.spec_from_file_location('avoidance_shadow',source)
shadow=importlib.util.module_from_spec(spec);spec.loader.exec_module(shadow)


class RecordedBodyPoseTests(unittest.TestCase):
    def test_mount_conjugates_recorded_lio_pose(self):
        message=Odometry();message.header.frame_id='camera_init';message.child_frame_id='body'
        message.pose.pose.orientation.w=1.
        self.assertEqual(shadow.recorded_body_pose(message,[.15,-.15,.22,0.,0.,0.]),[0.,0.,0.])
        message.pose.pose.position.x=1.;message.pose.pose.position.y=2.
        message.pose.pose.orientation.w=math.sqrt(.5);message.pose.pose.orientation.z=math.sqrt(.5)
        result=shadow.recorded_body_pose(message,[.15,-.15,.22,0.,0.,0.])
        for actual,expected in zip(result,[1.,1.7,math.pi/2]):self.assertAlmostEqual(actual,expected)
        message.child_frame_id='base_footprint'
        with self.assertRaises(ValueError):shadow.recorded_body_pose(message,[0.]*6)
