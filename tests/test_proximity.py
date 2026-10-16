"""Raw scan protection must detect new boxes without treating tangential poles as a block."""

import unittest
import time
from types import SimpleNamespace as N
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import Path as RosPath
from geometry_msgs.msg import PoseStamped
from array import array
from intramural_mission.adapter import Adapter
from intramural_base.transport import CommandPolicy
from intramural_mission.proximity import obstacle_near, obstacle_on_path, lidar_to_body


class ProximityTests(unittest.TestCase):
    def test_field_rejects_isolated_scan_hits_but_stops_for_supported_obstacle(self):
        route=[[0.,0.],[.15,0.],[.3,0.],[.5,0.]]
        cloud=np.array([[.308,-.059],[0.,.8],[-.8,0.]])
        self.assertFalse(obstacle_on_path(cloud,route,.4,.29,min_points=3))
        self.assertFalse(obstacle_near(cloud,(1.,0.),radial=.31,forward=.71,lateral=.29,immediate=.29,min_points=3))
        cloud=np.array([[.4,-.05],[.4,0.],[.4,.05]])
        self.assertTrue(obstacle_on_path(cloud,route,.4,.29,min_points=3))
        self.assertTrue(obstacle_near(cloud,(1.,0.),radial=.31,forward=.71,lateral=.29,immediate=.29,min_points=3))

    def test_controller_zero_stops_both_limiters_before_a_corner(self):
        now=time.monotonic()
        policy=CommandPolicy([[1.,0.],[0.,1.]],.4,.2,.4)
        policy.enabled=True;policy.previous=(.3,0.);policy.last_tick=now-.01
        sent=[]
        def transmit(msg):
            policy.receive(msg.linear.x,msg.linear.y,msg.angular.z,now)
            sent.append(policy.tick(now)[0])
        m=N(c=dict(robot_radius=.29,max_speed=.4,max_acceleration=.2,bypass_speed=.2),
            cmd=Twist(),last={k:now for k in ['command','odom','lidar','imu']},last_tick=now-.025,
            motion_permitted=True,enabled=True,bypassing=False,output=[.3,0.],
            last_motion_direction=(.3,0.),raw_near_points=np.empty((0,2)),mode='sensors',
            pub=N(publish=transmit),hazard_pub=N(publish=lambda v:None),diagnostic=N(publish=lambda v:None))
        Adapter.tick(m)
        self.assertEqual(sent[-1],(0.,0.,0.))
        m.cmd.linear.y=.3;m.last_tick=now-.025;policy.last_tick=now-.01
        Adapter.tick(m)
        self.assertEqual(sent[-1][0],0.)
        self.assertGreater(sent[-1][1],0.)

    def test_ordered_curve_excludes_side_rear_and_straight_extension_but_stops_on_route(self):
        route=[[0.,0.],[.15,0.],[.3,.15],[.3,.5]]
        for point,expected in [([.65,0.],False),([0.,-.5],False),([-.5,0.],False),([.3,.3],True)]:
            self.assertEqual(obstacle_on_path([point],route,.5,.29),expected)
        self.assertTrue(obstacle_near(np.array([[.65,0.]]),(.4,0.),radial=.31,forward=.79,lateral=.29,immediate=.29))
        self.assertFalse(obstacle_on_path([[-.5,0.]],[[-1.,0.],[-.5,0.],[0.,0.],[.5,0.]],.5,.29))
        self.assertTrue(obstacle_on_path([[-.6,0.]],[[0.,0.],[-.5,0.]],.5,.29))
        self.assertFalse(obstacle_on_path([],route,.5,.29))

    def test_adapter_uses_transformed_future_curve_and_person_removal_restores_output(self):
        path=RosPath();path.header.frame_id='map'
        for x,y in [[0.,0.],[.15,0.],[.3,.15],[.3,.5]]:
            p=PoseStamped();p.pose.position.x=x;p.pose.position.y=y;path.poses.append(p)
        tf=N(header=N(stamp=N(sec=0,nanosec=0)),transform=N(translation=N(x=0.,y=0.),rotation=N(w=1.,x=0.,y=0.,z=0.)))
        c=dict(robot_radius=.29,minimum_clearance=.02,max_speed=.4,max_acceleration=.2,bypass_speed=.2,raw_collision_follow_path=True)
        cmd=Twist();cmd.linear.x=.4;commands=[]
        m=N(c=c,cmd=cmd,mode='sensors',enabled=True,motion_permitted=True,bypassing=False,
            output=[.4,0.],last_motion_direction=(.4,0.),collision_path=path,path_tf=N(lookup_transform=lambda *a:tf),
            get_clock=lambda:N(now=lambda:N(nanoseconds=0)),pub=N(publish=commands.append),
            hazard_pub=N(publish=lambda v:None),diagnostic=N(publish=lambda v:None))
        m.body_path=lambda:Adapter.body_path(m)
        for point,blocked in [([.65,0.],False),([.3,.3],True),([.65,0.],False)]:
            now=time.monotonic();m.last={k:now for k in ['command','odom','lidar','imu']};m.last_tick=now-.025
            m.raw_near_points=np.array([point]);Adapter.tick(m)
            self.assertEqual(m.hazard,blocked)
            self.assertEqual(m.guard_mode,'ordered_path')
            self.assertEqual(commands[-1].linear.x==0.,blocked)
        c['raw_collision_min_points']=3
        for cloud,blocked in [([[.3,.3]],False),([[.3,.3],[.32,.3],[.3,.32]],True),([[.3,.3]],False)]:
            now=time.monotonic();m.last={k:now for k in ['command','odom','lidar','imu']};m.last_tick=now-.025
            m.raw_near_points=np.array(cloud);Adapter.tick(m)
            self.assertEqual(m.hazard,blocked)
            self.assertEqual(commands[-1].linear.x==0.,blocked)

    def test_field_directional_sweep_keeps_front_and_rear_motion_protection(self):
        kwargs=dict(radial=.31,forward=.79,lateral=.29,immediate=.29)
        for point,direction,expected in [([.02,.30],(.4,0.),False),([-.30,0.],(.4,0.),False),
                                          ([.70,0.],(.4,0.),True),([0.,-.70],(0.,-.4),True),
                                          ([.28,0.],(0.,0.),True)]:
            self.assertEqual(obstacle_near(np.array([point]),direction,**kwargs),expected)

    def test_real_near_zero_strength_return_is_filtered_without_hiding_far_dark_points(self):
        msg=PointCloud2();msg.height=1;msg.width=3;msg.point_step=16;msg.row_step=48
        msg.fields=[PointField(name=k,offset=i*4,datatype=7,count=1) for i,k in enumerate(('x','y','z','intensity'))]
        msg.data=array('B',np.array([[-.004,-.1144,.039,0.],[-.004,-.1144,.039,1.],[.5,0.,.3,0.]],dtype='<f4').tobytes())
        c=dict(lidar_pose=[.15,-.15,.22,0.,0.,0.],robot_radius=.29,raw_zero_intensity_min_range_m=.15)
        m=N(c=c,mode='sensors',touch=lambda k:None)
        Adapter.lidar(m,msg)
        np.testing.assert_allclose(m.raw_near_points,[[.146,-.2644],[.65,-.15]],atol=1e-6)
        del c['raw_zero_intensity_min_range_m']
        Adapter.lidar(m,msg)
        self.assertEqual(len(m.raw_near_points),3)

    def test_field_guard_shares_clearance_but_preserves_directional_braking(self):
        for minimum,point,expected in [(.02,[-.154,-.336],False),(.02,[.43,0.],True),
                                        (.02,[.30,0.],True),(None,[-.154,-.336],True)]:
            now=time.monotonic();commands=[];hazards=[]
            c=dict(robot_radius=.29,max_speed=.2,bypass_speed=.2,max_acceleration=.2)
            if minimum is not None:c['minimum_clearance']=minimum
            cmd=Twist();cmd.linear.x=.2
            m=N(c=c,cmd=cmd,last={k:now for k in ['command','odom','lidar','imu']},
                last_tick=now-.025,motion_permitted=True,enabled=True,bypassing=False,
                output=[.2,0.],last_motion_direction=(.2,0.),raw_near_points=np.array([point]),mode='sensors',
                pub=N(publish=commands.append),hazard_pub=N(publish=hazards.append),diagnostic=N(publish=lambda msg:None))
            Adapter.tick(m)
            self.assertEqual(hazards[-1].data,expected)
            self.assertEqual(commands[-1].linear.x,0. if expected else .2)

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
