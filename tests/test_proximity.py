"""Raw scan protection must detect new boxes without treating tangential poles as a block."""

import unittest
import time
from pathlib import Path
import rclpy
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
from intramural_mission.proximity import surface_support, outside_self
from intramural_mission.route import curve_speed_limit, arc, line


class ProximityTests(unittest.TestCase):
    def test_height_band_filters_overhang_before_guard_and_planner_share_xy(self):
        from intramural_mission.proximity import collision_height
        cloud=PointCloud2(height=1,width=4,point_step=12,row_step=48)
        cloud.fields=[PointField(name=k,offset=i*4,datatype=7,count=1)for i,k in enumerate(('x','y','z'))]
        cloud.data=np.array([[.5,0.,.04],[.5,.01,.20],[.5,.02,.43],[.5,.03,.46]],dtype='<f4').tobytes()
        m=N(atomic_command=True,c=dict(lidar_pose=[0.]*6,robot_radius=.29,raw_collision_z_bounds_m=[.19,.44]),
            mode='sensors',touch=lambda k:None,get_clock=lambda:N(now=lambda:N(nanoseconds=0)))
        Adapter.lidar(m,cloud)
        np.testing.assert_allclose(m.raw_near_points,[[.5,.01],[.5,.02]],atol=1e-6)
        for bounds in ([.4,.2],[0.,float('nan')],[.3]):
            with self.assertRaises(ValueError):collision_height(np.empty((0,3)),bounds)

    def test_stopped_guard_uses_current_speed_without_latching_pre_stop_speed(self):
        now=time.monotonic();diagnostics=[]
        c=dict(robot_radius=.29,minimum_clearance=.02,max_speed=2.,max_acceleration=3.5,
            navigation_engine='trajectory',raw_collision_follow_path=True,raw_collision_min_points=3,
            response_delay=.1,response_tau=.09)
        m=N(get_clock=lambda:N(now=lambda:N(nanoseconds=10_000_000_000)),atomic_command=False,c=c,
            cmd=Twist(),last={k:now for k in ['command','odom','lidar','imu']},last_tick=now-.025,
            motion_permitted=True,enabled=True,bypassing=False,output=[0.,0.],body_velocity=[0.,0.],
            mission_state='BLOCKED',held_speed=1.6,last_motion_direction=(1.6,0.),
            raw_near_points=np.array([[.8,-.02],[.8,0.],[.8,.02]]),
            body_path=lambda:line([0.,0.],[3.,0.],.05),mode='sensors',
            pub=N(publish=lambda v:None),hazard_pub=N(publish=lambda v:None),
            diagnostic=N(publish=lambda v:diagnostics.append(__import__('json').loads(v.data))))
        Adapter.tick(m)
        self.assertFalse(m.hazard)
        self.assertAlmostEqual(diagnostics[-1]['collision_reach_m'],.37)
        m.body_velocity=[1.6,0.];m.last_tick=time.monotonic()-.025;Adapter.tick(m)
        self.assertTrue(m.hazard,'Actual momentum still requires the longer stopping sweep')
        m.body_velocity=[0.,0.];m.raw_near_points=np.array([[.5,-.02],[.5,0.],[.5,.02]])
        m.last_tick=time.monotonic()-.025;Adapter.tick(m)
        self.assertTrue(m.hazard,'A real nearby obstacle must still block a stopped vehicle')

    def test_self_filter_is_local_to_the_confirmed_three_dimensional_part(self):
        points=np.array([[.16,-.30,.24],[.16,-.38,.24],[.16,-.30,.31],[.21,-.30,.24]])
        box=[[[.14,-.36,.20],[.19,-.24,.28]]]
        np.testing.assert_array_equal(outside_self(points,box),[False,True,True,True])
        with self.assertRaises(ValueError):outside_self(points,[[[0,0,0],[float('inf'),1,1]]])

    def test_isolated_scan_returns_are_removed_without_delaying_a_near_surface(self):
        points=np.array([[.30,0.,.3],[.31,.01,.3],[.30,.02,.3],
                         [.4,.4,.3],[.4,-.4,.3],[.4,0.,1.]])
        kept=surface_support(points,3)
        np.testing.assert_array_equal(kept,[True,True,True,False,False,False])
        self.assertTrue(obstacle_near(points[kept,:2],(1.,0.),radial=.31,forward=.5,lateral=.31,min_points=3))
        self.assertTrue(surface_support(points,1).all())

    def test_sweep_pruning_matches_independent_point_segment_distances(self):
        rng=np.random.default_rng(20261026)
        for _ in range(100):
            route=np.vstack(([0.,0.],np.cumsum(rng.normal(0.,.15,(15,2)),axis=0)))
            points=rng.uniform(-2.,2.,(150 if _%2 else 600,2));reach=rng.uniform(.1,1.5);radius=.29
            points=points[np.linalg.norm(points,axis=1)>radius+.01]
            hits=np.linalg.norm(points,axis=1)<radius;travelled=0.
            for a,b in zip(route,route[1:]):
                length=np.linalg.norm(b-a)
                if travelled>=reach:break
                d=(b-a)*min(1.,(reach-travelled)/length)
                for i,p in enumerate(points):
                    t=max(0.,min(1.,float(np.dot(p-a,d)/np.dot(d,d))))
                    hits[i] |= np.linalg.norm(p-a-t*d)<radius
                travelled+=length
            for support in (1,3):
                self.assertEqual(obstacle_on_path(points,route,reach,radius,support),np.count_nonzero(hits)>=support)

    def test_old_scan_is_compensated_and_checks_the_full_future_body(self):
        from livox_ros_driver2.msg import CustomMsg, CustomPoint
        from intramural_mission.livox_adapter import convert
        straight=line([0.,0.],[3.,0.],.05)
        bend=line([0.,0.],[.4,0.],.05)+line([.4,0.],[.4,.8],.05)[1:]
        for x,y,path,blocked in ((.82,0.,straight,True),(.82,.5,straight,False),(.655,.5,bend,True)):
            now=time.monotonic();cmd=Twist();cmd.linear.x=1.6;diagnostics=[]
            c=dict(lidar_pose=[0.]*6,robot_radius=.29,minimum_clearance=.02,max_speed=2.,max_acceleration=3.5,
                braking_acceleration=.2,bypass_speed=.2,raw_collision_min_points=3,
                navigation_engine='trajectory',raw_collision_follow_path=True,response_delay=.1,response_tau=.09)
            m=N(atomic_command=False,c=c,cmd=cmd,last={k:now for k in ['command','odom','lidar','imu']},last_tick=now-.025,
                motion_permitted=True,enabled=True,bypassing=False,output=[1.6,0.],body_velocity=[1.7,0.],
                odom_history=[(9.9,-.17,0.,0.),(10.,0.,0.,0.),(10.1,.17,0.,0.),(10.2,.34,0.,0.)],
                collision_pose_stamp=10.2,last_motion_direction=(1.6,0.),body_path=lambda:path,mode='sensors',
                touch=lambda k:None,get_clock=lambda:N(now=lambda:N(nanoseconds=10200000000)),
                pub=N(publish=lambda v:None),hazard_pub=N(publish=lambda v:None),diagnostic=N(publish=lambda v:diagnostics.append(v.data)))
            msg=CustomMsg();msg.header.frame_id='livox_frame';msg.header.stamp.sec=10
            msg.points=[CustomPoint(x=float(x),y=float(y+d),z=.5,reflectivity=20,tag=0,offset_time=50000000)for d in (-.01,0.,.01)]
            msg.point_num=3
            Adapter.lidar(m,convert(msg));Adapter.tick(m)
            with self.subTest(x=x,y=y,blocked=blocked):
                self.assertEqual(m.hazard,blocked)
                self.assertEqual(m.output==[0.,0.],blocked)
                self.assertTrue(__import__('json').loads(diagnostics[-1])['raw_motion_compensated'])

    def test_trajectory_stop_ignores_later_route_but_preserves_near_obstacle_stop(self):
        now=time.monotonic();cmd=Twist();cmd.linear.x=1.6
        c=dict(robot_radius=.29,minimum_clearance=.02,max_speed=2.,max_acceleration=3.5,
            braking_acceleration=.2,bypass_speed=.2,raw_collision_min_points=3,
            navigation_engine='trajectory',raw_collision_follow_path=True,
            response_delay=.1,response_tau=.09)
        m=N(get_clock=lambda:N(now=lambda:N(nanoseconds=10_000_000_000)), atomic_command=False,c=c,cmd=cmd,last={k:now for k in ['command','odom','lidar','imu']},last_tick=now-.025,
            motion_permitted=True,enabled=True,bypassing=False,output=[1.6,0.],body_velocity=[1.6,0.],
            last_motion_direction=(1.6,0.),raw_near_points=np.array([[2.,-.02],[2.,0.],[2.,.02]]),
            body_path=lambda:line([0.,0.],[3.,0.],.05),mode='sensors',
            pub=N(publish=lambda v:None),hazard_pub=N(publish=lambda v:None),diagnostic=N(publish=lambda v:None))
        Adapter.tick(m)
        self.assertFalse(m.hazard,'An obstacle 2 m away must not stop the current short braking path')
        self.assertGreater(m.output[0],0.)
        m.raw_near_points=np.array([[.5,-.02],[.5,0.],[.5,.02]])
        m.last_tick=time.monotonic()-.025
        Adapter.tick(m)
        self.assertTrue(m.hazard)
        self.assertEqual(m.output,[0.,0.])

    def test_mid360_confidence_reaches_guard_without_hiding_contact_or_normal_returns(self):
        from livox_ros_driver2.msg import CustomMsg, CustomPoint
        from intramural_mission.livox_adapter import convert
        path=[[0.,0.],[.124,-.158],[.105,-.182],[.130,-.222],[.159,-.261],[.190,-.297],[.225,-.330],[.261,-.361]]
        for tag,contact,blocked in ((16,False,False),(0,False,True),(128,False,True),(16,True,True)):
            now=time.monotonic();cmd=Twist();cmd.linear.x=.6;cmd.linear.y=-.8
            c=dict(lidar_pose=[0.]*6,robot_radius=.29,minimum_clearance=.02,max_speed=1.7,
                max_acceleration=3.,braking_acceleration=.2,bypass_speed=.2,
                raw_collision_min_points=3,raw_collision_follow_path=True)
            m=N(atomic_command=False,c=c,cmd=cmd,last={k:now for k in ['command','odom','lidar','imu']},last_tick=now-.025,
                motion_permitted=True,enabled=True,bypassing=False,output=[.6,-.8],
                last_motion_direction=(.6,-.8),body_path=lambda:path,mode='sensors',
                touch=lambda k:None,get_clock=lambda:N(now=lambda:N(nanoseconds=0)),
                pub=N(publish=lambda v:None),hazard_pub=N(publish=lambda v:None),diagnostic=N(publish=lambda v:None))
            msg=CustomMsg();msg.header.frame_id='livox_frame'
            xyz=[(.381,-.092,.371),(.392,-.077,.422),(.416,-.120,.251)]
            if contact:xyz=[(.30,-.01,.3),(.30,0.,.3),(.30,.01,.3)]
            msg.points=[CustomPoint(x=x,y=y,z=z,reflectivity=1,tag=tag)for x,y,z in xyz];msg.point_num=3
            Adapter.lidar(m,convert(msg));Adapter.tick(m)
            with self.subTest(tag=tag,contact=contact):
                self.assertEqual(m.hazard,blocked)
                self.assertEqual(m.output==[0.,0.],blocked)
                if not blocked:
                    m.last_tick=time.monotonic()-.025;Adapter.tick(m)
                    self.assertFalse(m.hazard)  # Rechecking the same scan is not confirmation.
                    msg.header.stamp.nanosec=100000000
                    Adapter.lidar(m,convert(msg));m.last_tick=time.monotonic()-.025;Adapter.tick(m)
                    self.assertTrue(m.hazard)
                    self.assertEqual(m.output,[0.,0.])

    def test_long_horizon_does_not_select_a_later_crossing_visit(self):
        path=[[.1,.05],[.5,.05],[1.,0.],[1.,1.],[0.,1.],[0.,0.],[0.,-1.]]
        self.assertTrue(obstacle_on_path([[.3,.05]],path,.4,.1))
        self.assertFalse(obstacle_on_path([[0.,-.3]],path,.4,.1))

    def test_high_cruise_limit_checks_scheduled_motion_then_slows_at_final_parking(self):
        now=time.monotonic();cmd=Twist();cmd.linear.x=1.5
        c=dict(robot_radius=.29,minimum_clearance=.02,max_speed=1.5,max_acceleration=.6,
            braking_acceleration=.2,bypass_speed=.2,raw_collision_min_points=3,
            raw_collision_follow_path=True,continuous_corners=True,lateral_acceleration=.5,
            parking_tolerance=.12)
        path=line([0.,0.],[6.,0.],.05)
        m=N(get_clock=lambda:N(now=lambda:N(nanoseconds=10_000_000_000)), atomic_command=False,c=c,cmd=cmd,last={k:now for k in ['command','odom','lidar','imu']},last_tick=now-.025,
            motion_permitted=True,enabled=True,bypassing=False,output=[0.,0.],
            last_motion_direction=(0.,0.),raw_near_points=np.array([[.8,-.04],[.8,0.],[.8,.04]]),
            body_path=lambda:path,mode='sensors',pub=N(publish=lambda v:None),
            hazard_pub=N(publish=lambda v:None),diagnostic=N(publish=lambda v:None))
        Adapter.tick(m)
        self.assertFalse(m.hazard)
        self.assertGreater(m.output[0],0.)
        m.output=[.5,0.];m.last_tick=time.monotonic()-.025;Adapter.tick(m)
        self.assertTrue(m.hazard)
        self.assertEqual(m.output,[0.,0.])
        m.raw_near_points=np.empty((0,2));m.mission_phase='RETURN';path=line([0.,0.],[.13,0.],.02)
        m.output=[.2,0.];m.last_tick=time.monotonic()-.025;Adapter.tick(m)
        self.assertLess(m.output[0],.2)
        path=line([0.,0.],[.11,0.],.02);m.last_tick=time.monotonic()-.025;Adapter.tick(m)
        self.assertLess(m.output[0],.2)
        self.assertGreater(m.output[0],0.)
        Adapter.controller_command(m,Twist())
        self.assertEqual(m.output,[0.,0.])

    def test_corner_speed_envelope_brakes_before_turn_without_zero_and_leaves_straight_free(self):
        straight=line([0.,0.],[1.5,0.],.05)
        self.assertEqual(curve_speed_limit(straight,.55,.4,.2),.55)
        # mission/collision_path repeats each shared segment endpoint.
        path=line([0.,0.],[.8,0.],.05)+line([.8,0.],[.8,1.],.05)
        speeds=[curve_speed_limit([[x-offset,y] for x,y in path],.55,.4,.2) for offset in (0.,.3,.65)]
        self.assertGreater(speeds[0],speeds[1])
        self.assertGreater(speeds[1],speeds[2])
        self.assertGreater(speeds[2],0.)
        self.assertLess(speeds[2],.2)
        for radius in (.55,.623,1.,1.5):
            curve=arc([0.,radius],radius,-np.pi/2,0.,.05)
            self.assertAlmostEqual(curve_speed_limit(curve,.55,.4,.2),min(.55,np.sqrt(.4*radius)),places=5)
            rotated=[[x*np.cos(.7)-y*np.sin(.7),x*np.sin(.7)+y*np.cos(.7)] for x,y in curve]
            self.assertAlmostEqual(curve_speed_limit(rotated,.55,.4,.2),curve_speed_limit(curve,.55,.4,.2),places=5)
            self.assertAlmostEqual(curve_speed_limit(curve,.6,.5,.2),min(.6,np.sqrt(.5*radius)),places=5)

    def test_planned_corner_speed_reaches_guard_and_transport_without_disabling_person_stop(self):
        now=time.monotonic();cmd=Twist();cmd.linear.x=.55;sent=[]
        policy=CommandPolicy([[1.,0.],[0.,1.]],.55,.6,.55)
        policy.enabled=True;policy.last_tick=now-.025
        def transmit(msg):
            policy.receive(msg.linear.x,msg.linear.y,msg.angular.z,time.monotonic())
            sent.append(policy.tick(time.monotonic())[0])
        c=dict(robot_radius=.29,minimum_clearance=.02,max_speed=.55,max_acceleration=.6,
            braking_acceleration=.2,bypass_speed=.2,raw_collision_min_points=3,
            raw_collision_follow_path=True,continuous_corners=True,lateral_acceleration=.4)
        route=line([0.,0.],[.15,0.],.05)+line([.15,0.],[.15,.7],.05)[1:]
        m=N(get_clock=lambda:N(now=lambda:N(nanoseconds=10_000_000_000)), atomic_command=False,c=c,cmd=cmd,last={k:now for k in ['command','odom','lidar','imu']},last_tick=now-.025,
            motion_permitted=True,enabled=True,bypassing=False,output=[0.,0.],
            last_motion_direction=(.55,0.),raw_near_points=np.empty((0,2)),body_path=lambda:route,
            mode='sensors',pub=N(publish=transmit),hazard_pub=N(publish=lambda v:None),
            diagnostic=N(publish=lambda v:None))
        for _ in range(30):
            m.last_tick=time.monotonic()-.025;policy.last_tick=time.monotonic()-.025
            Adapter.tick(m)
        self.assertGreater(sent[-1][0],.05)
        self.assertLess(sent[-1][0],.2)
        m.raw_near_points=np.array([[.30,.10],[.30,.12],[.31,.10]])
        Adapter.tick(m)
        self.assertTrue(m.hazard)
        self.assertEqual(sent[-1],(0.,0.,0.))
        m.raw_near_points=np.empty((0,2));m.last_tick=time.monotonic()-.025
        policy.last_tick=time.monotonic()-.025;Adapter.tick(m)
        self.assertFalse(m.hazard)
        self.assertGreater(sent[-1][0],0.)

    def test_faster_tracking_does_not_shorten_conservative_braking_sweep(self):
        now=time.monotonic();cmd=Twist();cmd.linear.x=.5
        m=N(get_clock=lambda:N(now=lambda:N(nanoseconds=10_000_000_000)), atomic_command=False,c=dict(robot_radius=.29,minimum_clearance=.02,max_speed=.5,max_acceleration=.6,
                   braking_acceleration=.2,bypass_speed=.2,raw_collision_min_points=3),
            cmd=cmd,last={k:now for k in ['command','odom','lidar','imu']},last_tick=now-.025,
            motion_permitted=True,enabled=True,bypassing=False,output=[.5,0.],
            last_motion_direction=(.5,0.),raw_near_points=np.array([[.8,-.04],[.8,0.],[.8,.04]]),
            mode='sensors',pub=N(publish=lambda v:None),hazard_pub=N(publish=lambda v:None),
            diagnostic=N(publish=lambda v:None))
        Adapter.tick(m)
        self.assertTrue(m.hazard)
        self.assertEqual(m.output,[0.,0.])

    def test_raw_controller_zero_ros_topic_reaches_final_stop_without_a_timer_tick(self):
        import json, tempfile
        original=Path(__file__).resolve().parents[1]/'config/arena.json'
        config_file=tempfile.NamedTemporaryFile(mode='w',suffix='.json',dir=original.parent.parent/'cache')
        config=Path(config_file.name)
        config_file.write(json.dumps(dict(json.loads(original.read_text()),navigation_engine='legacy')));config_file.flush()
        rclpy.init(domain_id=78,args=['--ros-args','-p',f'arena_config:={config}','-p','localization:=sensors'])
        guard=Adapter();peer=rclpy.create_node('controller_zero_observer')
        for timer in guard.timers:timer.cancel()
        guard.output=[.3,0.];received=[]
        peer.create_subscription(Twist,'cmd_vel',received.append,10)
        pub=peer.create_publisher(Twist,'cmd_vel_controller',10)
        try:
            deadline=time.monotonic()+5
            while not received and time.monotonic()<deadline:
                pub.publish(Twist())
                rclpy.spin_once(guard,timeout_sec=.025)
                rclpy.spin_once(peer,timeout_sec=.025)
            self.assertTrue(received)
            self.assertEqual(guard.output,[0.,0.])
            self.assertEqual((received[-1].linear.x,received[-1].linear.y),(0.,0.))
        finally:
            peer.destroy_node();guard.destroy_node();rclpy.shutdown();config_file.close()

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
        m=N(get_clock=lambda:N(now=lambda:N(nanoseconds=10_000_000_000)), atomic_command=False,c=dict(robot_radius=.29,max_speed=.4,max_acceleration=.2,bypass_speed=.2),
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

    def test_raw_controller_stop_clears_a_smoothed_old_direction_immediately(self):
        policy=CommandPolicy([[1.,0.],[0.,1.]],.4,.2,.4)
        policy.enabled=True;policy.previous=(.3,0.);policy.last_tick=0.
        m=N(get_clock=lambda:N(now=lambda:N(nanoseconds=10_000_000_000)), atomic_command=False,cmd=Twist(),output=[.3,0.],pub=N(publish=lambda msg:policy.receive(msg.linear.x,msg.linear.y,0.,.01)))
        Adapter.controller_command(m,Twist())
        self.assertEqual(m.output,[0.,0.])
        self.assertEqual(policy.previous,(0.,0.))
        policy.receive(0.,.3,0.,.015)
        values,_=policy.tick(.02)
        self.assertEqual(values[0],0.)
        self.assertGreater(values[1],0.)

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
        tf=N(header=N(stamp=N(sec=0,nanosec=0)),transform=N(atomic_command=False,translation=N(x=0.,y=0.),rotation=N(w=1.,x=0.,y=0.,z=0.)))
        c=dict(robot_radius=.29,minimum_clearance=.02,max_speed=.4,max_acceleration=.2,bypass_speed=.2,raw_collision_follow_path=True)
        cmd=Twist();cmd.linear.x=.4;commands=[]
        m=N(atomic_command=False,c=c,cmd=cmd,mode='sensors',enabled=True,motion_permitted=True,bypassing=False,
            output=[.4,0.],last_motion_direction=(.4,0.),collision_path=path,path_tf=N(lookup_transform=lambda *a:tf),
            get_clock=lambda:N(now=lambda:N(nanoseconds=0)),pub=N(publish=commands.append),
            hazard_pub=N(publish=lambda v:None),diagnostic=N(publish=lambda v:None))
        m.body_path=lambda:Adapter.body_path(m)
        for point,blocked in [([.65,0.],False),([.3,.3],True),([.65,0.],False)]:
            now=time.monotonic();m.last={k:now for k in ['command','odom','lidar','imu']};m.last_tick=now-.025
            m.output=[.4,0.]
            m.raw_near_points=np.array([point]);Adapter.tick(m)
            self.assertEqual(m.hazard,blocked)
            self.assertEqual(m.guard_mode,'ordered_path')
            self.assertEqual(commands[-1].linear.x==0.,blocked)
        c['raw_collision_min_points']=3
        for cloud,blocked in [([[.3,.3]],False),([[.3,.3],[.32,.3],[.3,.32]],True),([[.3,.3]],False)]:
            now=time.monotonic();m.last={k:now for k in ['command','odom','lidar','imu']};m.last_tick=now-.025
            m.output=[.4,0.]
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
        m=N(atomic_command=False,c=c,mode='sensors',touch=lambda k:None,get_clock=lambda:N(now=lambda:N(nanoseconds=0)))
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
            m=N(get_clock=lambda:N(now=lambda:N(nanoseconds=10_000_000_000)), atomic_command=False,c=c,cmd=cmd,last={k:now for k in ['command','odom','lidar','imu']},
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
