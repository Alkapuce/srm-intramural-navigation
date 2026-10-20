"""Real raw conversion must preserve geometry and reject unusable scans."""
import copy
import unittest
import numpy as np
from livox_ros_driver2.msg import CustomMsg, CustomPoint
from intramural_mission.livox_adapter import convert


class LivoxAdapterTest(unittest.TestCase):
    def test_preserve_raw_frame_time_and_points_and_reject_malformed_input(self):
        msg=CustomMsg()
        msg.header.frame_id="livox_frame"
        msg.header.stamp.sec=123
        msg.header.stamp.nanosec=456
        msg.points=[CustomPoint(x=1.0,y=-2.0,z=.5,reflectivity=20,tag=16,offset_time=100)]
        msg.point_num=1
        out=convert(msg)
        self.assertEqual(out.header,msg.header)
        self.assertEqual(out.row_step,24)
        np.testing.assert_array_equal(np.frombuffer(out.data,dtype="<f4",count=4),[1,-2,.5,20])
        self.assertEqual(out.fields[-1].name,"offset_time")
        self.assertEqual(np.frombuffer(out.data,dtype='<u4',count=1,offset=20)[0],100)
        self.assertEqual(out.data[16],16)
        for field,value in [("point_num",2),("point_num",0),("frame_id","camera_init"),("x",float("nan"))]:
            bad=copy.deepcopy(msg)
            if field=="frame_id":bad.header.frame_id=value
            elif field=="x":bad.points[0].x=value
            else:setattr(bad,field,value)
            with self.subTest(field=field,value=value), self.assertRaises(ValueError):convert(bad)
