"""Safety regressions for crossings, busy requests and asynchronous cancellation."""

import json, math, unittest, time
import tempfile
from pathlib import Path
from types import SimpleNamespace as N
from concurrent.futures import Future
from unittest.mock import patch, Mock
from std_srvs.srv import Trigger
from nav_msgs.msg import Path as RosPath, OccupancyGrid
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from intramural_mission.route import segments, projected_progress, primitives, controller_segment
from intramural_mission.mission import Mission
from intramural_mission.bypass import plan, winding, entry_path
import numpy as np
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy

_root = Path(__file__).resolve().parents[1]
_config_path = Path("/work/config/arena.json") if Path("/work/config/arena.json").exists() else _root / "config/arena.json"
C = json.loads(_config_path.read_text())


def ray_winding(path, p):
    result = 0
    for a, b in zip(path, path[1:]):
        cross = (b[0] - a[0]) * (p[1] - a[1]) - (p[0] - a[0]) * (b[1] - a[1])
        if a[1] <= p[1] < b[1] and cross > 0:
            result += 1
        if b[1] <= p[1] < a[1] and cross < 0:
            result -= 1
    return result


def completed(value):
    f = Future()
    f.set_result(value)
    return f


class MissionTests(unittest.TestCase):
    def test_entry_exit_spread_turn_without_curvature_spikes(self):
        for radius, heading, side in ((.55, 0., 0.), (.623, .8, 0.), (1.5, -.6, math.pi)):
            def rotate(p):
                return [p[0]*math.cos(heading)-p[1]*math.sin(heading),
                        p[0]*math.sin(heading)+p[1]*math.cos(heading)]
            sign = 1 if side == 0 else -1
            c = dict(C, loop_radius=radius, lower_pole=[0., 0.], upper_pole=rotate([0., 2*radius]),
                     parking=rotate([sign*2.2*radius, sign*.8*radius]), entry_angle=side,
                     smooth_transitions=True)
            route = segments(c)
            points = np.array([p for _, path in route for p in path])
            points = points[np.r_[True, np.linalg.norm(np.diff(points, axis=0), axis=1) > 1e-8]]
            a, b = np.diff(points, axis=0)[:-1], np.diff(points, axis=0)[1:]
            curvature = 2*np.abs(np.cross(a, b))/(np.linalg.norm(a, axis=1)*np.linalg.norm(b, axis=1)
                *np.linalg.norm(points[2:]-points[:-2], axis=1))
            self.assertLess(float(curvature.max()), 2/radius)
            self.assertLessEqual(float(np.linalg.norm(np.diff(points, axis=0), axis=1).max()), c['path_spacing']*1.001)
            self.assertEqual([ray_winding(points, c[k]) for k in ('lower_pole', 'upper_pole')], [1, -1])

    def test_smoothed_entry_exit_keep_order_winding_and_tangent_handoffs_for_varied_spacing(self):
        for radius in (.55,.623,1.5):
            c=dict(C,loop_radius=radius,lower_pole=[0.,0.],upper_pole=[0.,2*radius],
                parking=[2*radius,-radius],entry_angle=0.,smooth_transitions=True)
            route=segments(c);points=[p for _,path in route for p in path]
            self.assertEqual([winding(points,c[k])for k in ('lower_pole','upper_pole')],[1,-1])
            self.assertEqual(len(route),10)
            for index in (0,8):
                left,right=route[index][1],route[index+1][1]
                self.assertEqual(left[-1],right[0])
                a=np.array(left[-1])-left[-2];b=np.array(right[1])-right[0]
                self.assertGreater(np.dot(a,b)/np.linalg.norm(a)/np.linalg.norm(b),.95)
            self.assertEqual(route[0][1][0],c['parking'])
            self.assertEqual(route[-1][1][-1],c['parking'])

    def test_open_map_entry_uses_straight_connector_without_grid_staircase(self):
        grid=OccupancyGrid();grid.header.frame_id='map';grid.info.resolution=.05
        grid.info.width=grid.info.height=200;grid.info.origin.position.x=grid.info.origin.position.y=-3.
        grid.info.origin.orientation.w=1.;grid.data=[0]*40000
        c=dict(C,pole_clearance_from_map=True)
        path=entry_path(c,[.013,-.017],[.941,.213],grid)
        a,b=np.array(path[0]),np.array(path[-1])
        self.assertTrue(all(abs(np.cross(b-a,np.array(p)-a))<1e-10 for p in path))

    def test_obstacle_removal_waits_for_extended_goal_inflation_then_resumes_same_segment(self):
        c=dict(C,continuous_segments=True,minimum_clearance=.02,allow_bypass=False)
        root=Path(__file__).resolve().parents[1]
        with tempfile.TemporaryDirectory(dir=root/'cache') as tmp:
            config=Path(tmp)/'arena.json';config.write_text(json.dumps(c))
            rclpy.init(domain_id=79,args=['--ros-args','-p',f'arena_config:={config}'])
            m=Mission()
            try:
                for timer in m.timers:timer.cancel()
                m.index=4;m.phase='UPPER';m.state='BLOCKED';m.blocked_count=1
                current=m.route[4][1];extended,_=controller_segment(m.route,4)
                m.progress=len(current)-3;pose=current[m.progress];m.pose=pose
                def lookup(target,source,stamp):
                    t=TransformStamped();t.header.stamp=m.get_clock().now().to_msg();t.transform.rotation.w=1.
                    if source=='base_footprint':t.transform.translation.x,t.transform.translation.y=pose
                    return t
                m.tf=N(lookup_transform=lookup);m.navigator_active=True
                m.clear_since=time.monotonic()-1
                grid=OccupancyGrid();grid.header.frame_id='map';grid.info.width=grid.info.height=200
                grid.info.resolution=.05;grid.info.origin.position.x=grid.info.origin.position.y=-2.
                grid.info.origin.orientation.w=1.;data=[0]*(200*200)
                x,y=extended[-1];res=grid.info.resolution
                cell=math.floor((y+2)/res)*200+math.floor((x+2)/res)
                data[cell]=99;grid.data=data;m.receive_costmap(grid)
                goals=[];result=Future();cancel=[]
                handle=N(accepted=True,get_result_async=lambda:result,cancel_goal_async=lambda:cancel.append(True))
                def send(goal):goals.append(goal);return completed(handle)
                m.client=N(send_goal_async=send)
                blocked=m.obstacle_index()
                self.assertIsNotNone(blocked)
                self.assertGreaterEqual(blocked,len(current))
                m.tick()
                self.assertEqual(m.state,'BLOCKED',m.reason)
                self.assertFalse(goals)
                data[cell]=0;grid.data=data;m.receive_costmap(grid);m.tick()
                self.assertEqual(m.state,'BLOCKED')
                clock=time.monotonic()+.6
                with patch('intramural_mission.mission.time.monotonic',return_value=clock):
                    m.receive_costmap(grid);m.tick()
                self.assertEqual(m.state,'RUNNING',m.reason)
                self.assertEqual(m.index,4)
                self.assertEqual(len(goals),1)
                self.assertEqual(len(goals[0].path.poses),len(extended)-m.progress)
                # A new actual occupancy in the extension still stops the resumed goal.
                data[cell]=100;grid.data=data;m.receive_costmap(grid);m.tick()
                self.assertEqual(m.state,'BLOCKED')
                self.assertTrue(cancel)
            finally:
                m.destroy_node();rclpy.shutdown()

    def test_start_here_uses_measured_map_instead_of_an_oversized_ideal_pole(self):
        c=dict(C,robot_radius=.29,pole_radius=.15,minimum_clearance=.02,
               lower_pole=[0.,0.],upper_pole=[0.,1.2],loop_radius=.6,
               bounds=[-3.,3.,-3.,4.],entry_angle=0.,parking=[2.,-1.])
        grid=OccupancyGrid();grid.header.frame_id='map';grid.info.width=240;grid.info.height=280
        grid.info.resolution=.025;grid.info.origin.position.x=grid.info.origin.position.y=-3.
        grid.info.origin.orientation.w=1.
        yy,xx=np.mgrid[:280,:240];x=-3+(xx+.5)*.025;y=-3+(yy+.5)*.025
        data=np.zeros((280,240),dtype=np.int8)
        for px,py in (c['lower_pole'],c['upper_pole']):data[np.hypot(x-px,y-py)<.1]=100
        grid.data=data.ravel().tolist();pose=[-.28,.85];goal=segments(c)[0][1][-1]
        self.assertIsNone(entry_path(c,pose,goal,grid))
        c['pole_clearance_from_map']=True
        now=time.monotonic();gate=[]
        m=N(start_permitted=True,state='IDLE',navigator_active=True,pose=pose,last_pose=now,
            ready_since=now-3,tf_age=0.,variant='figure8',c=c,run=0,static_map=grid,
            costmap=grid,costmap_at=now,bypass_pub=Mock(),enabled=gate.append,
            get_clock=lambda:N(now=lambda:N(nanoseconds=0,to_msg=lambda:Time())),
            path_pub=Mock(),accepted=lambda f,**kw:None,
            client=N(server_is_ready=lambda:True,send_goal_async=lambda g:completed(None)))
        m.make_path=lambda points:Mission.make_path(m,points)
        m.next_segment=lambda:Mission.next_segment(m)
        m.start=lambda req,res,**kw:Mission.start(m,req,res,**kw)
        result=Mission.start_here(m,Trigger.Request(),Trigger.Response())
        self.assertTrue(result.success,result.message)
        self.assertEqual(len(m.route),10)
        self.assertEqual(m.route[-1][1][-1],c['parking'])
        self.assertEqual(gate,[True])
        # Starting above the first gate must approach in the arc direction,
        # rather than producing a hairpin whose far endpoint is already nearby.
        m.state='IDLE';m.pose=[.6,1.6]
        result=Mission.start_here(m,Trigger.Request(),Trigger.Response())
        self.assertTrue(result.success,result.message)
        incoming=np.array(m.route[0][1][-1])-m.route[0][1][-2]
        outgoing=np.array(m.route[1][1][1])-m.route[1][1][0]
        self.assertGreater(np.dot(incoming,outgoing)/np.linalg.norm(incoming)/np.linalg.norm(outgoing),.95)
        m.pose=pose
        # Measured occupancy and unknown space still reject before enabling motion.
        m.state='IDLE';gate.clear()
        row,col=int((pose[1]+3)/.025),int((pose[0]+3)/.025)
        for value in (100,-1):
            data[row,col]=value;grid.data=data.ravel().tolist()
            self.assertFalse(Mission.start_here(m,Trigger.Request(),Trigger.Response()).success)
            self.assertFalse(gate)
        m.static_map=None
        self.assertFalse(Mission.start_here(m,Trigger.Request(),Trigger.Response()).success)
        # Start through the real planner and one-time curve footprint validation.
        data[data<0]=0;grid.data=data.ravel().tolist();m.static_map=grid
        c['smooth_transitions']=True;m.pose=list(c['parking']);m.state='IDLE'
        m.costmap_at=time.monotonic()
        result=Mission.start_here(m,Trigger.Request(),Trigger.Response())
        self.assertTrue(result.success,result.message)
        left,right=m.route[0][1],m.route[1][1]
        a=np.array(left[-1])-left[-2];b=np.array(right[1])-right[0]
        self.assertGreater(np.dot(a,b)/np.linalg.norm(a)/np.linalg.norm(b),.95)

    def test_real_mission_map_guard_remains_active_without_ideal_circle_veto(self):
        c=dict(C,pole_clearance_from_map=True,robot_radius=.29,pole_radius=.15,
               minimum_clearance=.02,lower_pole=[0.,0.],upper_pole=[0.,1.246],
               loop_radius=.623,entry_angle=0.)
        root=Path(__file__).resolve().parents[1]
        with tempfile.TemporaryDirectory(dir=root/'cache') as tmp:
            config=Path(tmp)/'arena.json';config.write_text(json.dumps(c))
            rclpy.init(domain_id=79,args=['--ros-args','-p',f'arena_config:={config}'])
            m=Mission()
            try:
                for timer in m.timers:timer.cancel()
                pose=[-.45,1.246]
                def lookup(target,source,stamp):
                    t=TransformStamped();t.transform.rotation.w=1.
                    t.header.stamp=m.get_clock().now().to_msg()
                    if source=='base_footprint':t.transform.translation.x,t.transform.translation.y=pose
                    return t
                m.tf=N(lookup_transform=lookup)
                grid=OccupancyGrid();grid.header.frame_id='map';grid.info.width=grid.info.height=160
                grid.info.resolution=.05;grid.info.origin.position.x=grid.info.origin.position.y=-2.
                grid.info.origin.orientation.w=1.;grid.data=[0]*(160*160)
                m.receive_map(grid);m.receive_costmap(grid)
                m.state='RUNNING';m.index=4;m.phase='UPPER';m.progress=len(m.route[4][1])-1
                m.resume_at=time.monotonic();m.navigator_active=True
                m.tick()
                self.assertEqual(m.state,'RUNNING',m.reason)
                x,y=m.route[4][1][-1];data=list(grid.data)
                data[int((y+2)/.05)*160+int((x+2)/.05)]=100
                grid.data=data;m.receive_costmap(grid);m.tick()
                self.assertEqual(m.state,'BLOCKED')
                self.assertEqual(m.blocked_count,1)
            finally:
                m.destroy_node();rclpy.shutdown()

    def test_continuous_handoff_preserves_lateral_tolerance_and_ordered_progress(self):
        c=dict(C,continuous_segments=True,checkpoint_tolerance=.15)
        route=segments(c);points=route[3][1]
        a,b=points[-2:];dx,dy=b[0]-a[0],b[1]-a[1];length=math.hypot(dx,dy)
        advances=[]
        m=N(c=c,continuous_goal=True,goal_pending=False,segment_result=None,route=route,index=3,
            pose=[b[0]-.053*dy/length,b[1]+.053*dx/length],progress_fraction=len(points)-1-.4,
            next_segment=lambda:advances.append(True))
        # A 5.3 cm lateral error missed the former 5 cm endpoint disk and left
        # the mission on its old segment while the controller followed the next.
        Mission.finish_segment(m)
        self.assertEqual(advances,[True])
        advances.clear();m.progress_fraction=len(points)-1-2.
        Mission.finish_segment(m)
        self.assertEqual(advances,[])

    def test_continuous_tick_hands_off_within_tracking_corridor_without_skipping_progress(self):
        c=dict(C,continuous_segments=True,continuous_corners=True,pole_clearance_from_map=True,
               lower_pole=[0.,0.],upper_pole=[0.,1.246],loop_radius=.623,entry_angle=0.)
        with tempfile.TemporaryDirectory(dir=_root/'cache') as tmp:
            config=Path(tmp)/'arena.json';config.write_text(json.dumps(c))
            rclpy.init(domain_id=79,args=['--ros-args','-p',f'arena_config:={config}'])
            m=Mission()
            try:
                for timer in m.timers:timer.cancel()
                # TF and the action transport are the external boundaries; tick and handoff stay real.
                goals=[]
                def send(goal):goals.append(goal);return Future()
                m.client=N(send_goal_async=send)
                points=m.route[2][1];a,b=points[-2:]
                dx,dy=b[0]-a[0],b[1]-a[1];length=math.hypot(dx,dy)
                pose=[]
                def lookup(*args):
                    t=TransformStamped();t.header.stamp=m.get_clock().now().to_msg()
                    t.transform.rotation.w=1.;t.transform.translation.x,t.transform.translation.y=pose
                    return t
                m.tf=N(lookup_transform=lookup)
                for lateral,remaining,index,state in ((.17,.02,3,'RUNNING'),(.17,.20,2,'RUNNING'),(.34,.02,2,'FAULT')):
                    pose[:]=[b[0]-remaining*dx/length-lateral*dy/length,
                             b[1]-remaining*dy/length+lateral*dx/length]
                    m.state='RUNNING';m.index=2;m.phase='LOWER';m.reason=''
                    m.progress=max(0,len(points)-1-math.ceil(remaining/length))
                    m.progress_fraction=float(m.progress)
                    m.continuous_goal=True;m.goal_pending=False;m.segment_result=None
                    m.resume_at=m.last_check=time.monotonic();m.navigator_active=True
                    m.tick()
                    self.assertEqual((m.index,m.state),(index,state),m.reason)
                    if index==3:self.assertEqual(goals[-1].goal_checker_id,'continuous_goal_checker')
                m.index=8;m.next_segment()
                self.assertEqual(goals[-1].goal_checker_id,'general_goal_checker')
            finally:
                m.destroy_node();rclpy.shutdown()

    def test_tangent_goal_overlap_preserves_order_and_keeps_sharp_corner_stops(self):
        c=dict(C,entry_angle=0.)
        route=segments(c)
        for index in range(10):
            path,continuous=controller_segment(route,index)
            self.assertEqual(continuous,1<=index<=7)
            self.assertEqual(path[:len(route[index][1])],route[index][1])
            if continuous:self.assertGreater(len(path),len(route[index][1]))
        advances=[]
        points=route[1][1]
        m=N(c=dict(c,continuous_segments=True),continuous_goal=True,goal_pending=False,
            segment_result=None,route=route,index=1,pose=points[-1],progress_fraction=len(points)-1,
            next_segment=lambda:advances.append(True))
        Mission.finish_segment(m)
        self.assertEqual(advances,[True])
        m.index=2;m.goal_handle=object();handle=m.goal_handle
        Mission.result(m,completed(N(status=6)),segment=1)
        self.assertIs(m.goal_handle,handle)

    def test_speed_planned_corners_overlap_entry_exit_but_preserve_route_and_stop_at_parking(self):
        route=segments(dict(C,entry_angle=0.))
        for index in range(10):
            path,continuous=controller_segment(route,index,allow_corners=True)
            self.assertEqual(continuous,index<9)
            self.assertEqual(path[:len(route[index][1])],route[index][1])
            if continuous:self.assertGreater(len(path),len(route[index][1]))
        reversal=[('ENTER',[[0.,0.],[1.,0.]]),('RETURN',[[1.,0.],[0.,0.]])]
        self.assertFalse(controller_segment(reversal,0,allow_corners=True)[1])
        advances=[];points=route[0][1];length=math.dist(points[-2],points[-1])
        m=N(c=dict(C,continuous_segments=True,continuous_corners=True),route=route,index=0,
            continuous_goal=True,goal_pending=False,segment_result=None,
            pose=[points[-1][0]+.03,points[-1][1]-.10],
            progress_fraction=len(points)-1-.1/length,next_segment=lambda:advances.append(True))
        Mission.finish_segment(m)
        self.assertEqual(advances,[True])
        m.progress_fraction=len(points)-1-.2/length;advances.clear()
        Mission.finish_segment(m)
        self.assertFalse(advances)

    def test_collision_path_ros_topic_keeps_return_after_current_segment(self):
        rclpy.init(domain_id=79,args=['--ros-args','-p',f'arena_config:={_config_path}'])
        mission, observer = Mission(), rclpy.create_node('collision_path_observer')
        received=[]
        observer.create_subscription(RosPath,'mission/collision_path',received.append,
            QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        try:
            mission.state, mission.index = 'BLOCKED', 8
            mission.progress = len(mission.route[8][1])-2
            mission.pose = mission.route[8][1][mission.progress]
            mission.publish_status()
            deadline=time.monotonic()+5
            while not received and time.monotonic()<deadline:rclpy.spin_once(observer,timeout_sec=.1)
            self.assertTrue(received)
            points=[[p.pose.position.x,p.pose.position.y] for p in received[-1].poses]
            self.assertEqual(points[0],mission.pose)
            self.assertIn(mission.route[8][1][-1],points)
            self.assertGreater(len(points),2)
            self.assertLess(__import__('math').dist(points[-1],mission.c['parking']),
                            __import__('math').dist(mission.route[8][1][-1],mission.c['parking']))
        finally:
            observer.destroy_node();mission.destroy_node();rclpy.shutdown()

    def test_field_two_cm_clearance_allows_small_gap_but_stops_inside_margin(self):
        c=dict(C,minimum_clearance=.02,loop_radius=.623,lower_pole=[0.,0.],upper_pole=[0.,1.246],entry_angle=0.)
        route=segments(c)
        for distance, expected in [(.47,'RUNNING'),(.45,'FAULT'),(.43,'FAULT')]:
            pose=[-distance,c['upper_pole'][1]]
            transform=N(header=N(stamp=N(sec=0,nanosec=0)),transform=N(translation=N(x=pose[0],y=pose[1])))
            now=time.monotonic()
            m=N(c=c,route=route,index=4,progress=len(route[4][1])-1,progress_fraction=0.,
                state='RUNNING',variant='figure8',segment_result=None,resume_at=now,last_check=now,
                navigator_active=True,ready_since=now,costmap=None,costmap_at=0.,abort_at=None,
                tf=N(lookup_transform=lambda *args:transform),get_clock=lambda:N(now=lambda:N(nanoseconds=0)),
                enabled=lambda value:None,goal_handle=None,publish_status=lambda:None)
            m.obstacle_index=lambda:Mission.obstacle_index(m)
            m.fault=lambda reason:Mission.fault(m,reason)
            m.finish_segment=lambda:Mission.finish_segment(m)
            Mission.tick(m)
            self.assertEqual(m.state,expected)

    def test_entry_detour_closes_loop_and_returns_along_entered_path(self):
        route = segments(C)
        blocked = [(x,y) for x in np.arange(1.25,1.751,.05) for y in np.arange(.5,1.001,.05)]
        result = plan(C,route,0,4,route[0][1][4],blocked)
        self.assertIsNotNone(result)
        route[:len(result)] = [(result[0][0],route[0][1][:4]+result[0][1])] + result[1:]
        entered = route[0][1]
        self.assertGreater(np.linalg.norm(np.asarray(entered[-1])-np.asarray(segments(C)[0][1][-1])),.1)
        m = N(index=0,variant='figure8',bypassing=True,start_from_here=False,route=route,c=C,history=[],
            get_clock=lambda:N(now=lambda:N(nanoseconds=0)),make_path=lambda points:RosPath(),
            path_pub=Mock(),client=Mock(),accepted=lambda future,**kw:None)
        m.client.send_goal_async.return_value = completed(None)
        Mission.next_segment(m)
        self.assertEqual(m.route[-1][1],list(reversed(entered)))
        self.assertEqual(m.route[-2][1][-1],entered[-1])
        points = [p for _,path in m.route for p in path]
        self.assertEqual([winding(points,C[k]) for k in ('lower_pole','upper_pole')],[1,-1])

    def test_blocked_and_aborted_paths_keep_far_obstacle_under_observation(self):
        grid = N(header=N(frame_id='odom'), data=[0], info=N(width=1,height=1,resolution=.05,
            origin=N(position=N(x=0,y=0),orientation=N(w=1,x=0,y=0,z=0))))
        transform = N(transform=N(translation=N(x=0,y=0),rotation=N(w=1,x=0,y=0,z=0)))
        for state, abort, reach in [('RUNNING',None,1.2),('BLOCKED',None,5.0),('RUNNING',1,5.0)]:
            m = N(costmap=grid,costmap_at=time.monotonic(),tf=N(lookup_transform=lambda *a:transform),
                route=segments(C),index=0,progress=0,pose=[0,0],c=C,state=state,abort_at=abort)
            with patch('intramural_mission.mission.first_blocked') as check:
                Mission.obstacle_index(m)
                self.assertEqual(check.call_args.kwargs['lookahead'],reach)
        m.state = 'BLOCKED'
        with patch('intramural_mission.mission.first_blocked', side_effect=[None,17]) as check:
            self.assertEqual(Mission.obstacle_index(m),17)
            self.assertEqual(check.call_args.kwargs['threshold'],99)
        m.c=dict(C,minimum_clearance=.02)
        with patch('intramural_mission.mission.first_blocked', side_effect=[None,17]) as check:
            Mission.obstacle_index(m)
            self.assertAlmostEqual(check.call_args_list[0].kwargs['radius'],.31)
            self.assertEqual(check.call_args_list[1].kwargs['radius'],.02)

    def test_obstacle_pause_closes_gate_and_preserves_segment(self):
        gate = []
        cancel = []
        handle = N(cancel_goal_async=lambda: cancel.append(True))
        m = N(state="RUNNING", index=3, blocked_count=0, goal_handle=handle,
              enabled=gate.append, clear_since=1, abort_at=2)
        Mission.pause_for_obstacle(m, 17)
        self.assertEqual((m.state, m.index, m.blocked_index), ("BLOCKED", 3, 17))
        self.assertEqual(gate, [False])
        self.assertEqual(cancel, [True])
        self.assertIsNone(m.abort_at)

    def test_cancel_from_blocked_without_action_is_terminal(self):
        gate = []
        m = N(state="BLOCKED", goal_handle=None, goal_pending=False,
              segment_result=None, enabled=gate.append,
              get_clock=lambda: N(now=lambda: N(nanoseconds=0)))
        result = Mission.cancel(m, Trigger.Request(), Trigger.Response())
        self.assertTrue(result.success)
        self.assertEqual(m.state, "CANCELLED")
        self.assertEqual(gate, [False])

    def test_reference_winding_and_distinct_crossing_visits(self):
        route = segments(C)
        path = [p for _, points in route for p in points]
        self.assertEqual(ray_winding(path, C["lower_pole"]), 1)
        self.assertEqual(ray_winding(path, C["upper_pole"]), -1)
        self.assertLess(
            __import__("math").dist(route[3][1][-1], route[7][1][-1]), 1e-10
        )
        self.assertNotEqual(route[3][0], route[7][0])

    def test_success_at_crossing_cannot_skip_unvisited_segment(self):
        route = segments(C)
        faults = []
        advances = []
        m = N(
            goal_handle=object(),
            state="RUNNING",
            route=route,
            index=3,
            pose=route[3][1][-1],
            progress=0,
            progress_fraction=0.0,
            c=C,
            fault=faults.append,
            next_segment=lambda: advances.append(True),
        )
        Mission.result(m, completed(N(status=4)))
        self.assertFalse(faults)  # Allow a fresh TF to arrive before final checking.
        m.result_at -= 1
        Mission.finish_segment(m)
        self.assertTrue(faults)
        self.assertFalse(advances)

    def test_delayed_pose_and_fractional_progress_finish_short_segment(self):
        route = primitives(C)
        advances = []
        m = N(
            goal_handle=object(),
            state="RUNNING",
            route=route,
            index=1,
            pose=[0.8, 0.64],
            progress_fraction=12.8,
            c=C,
            fault=lambda reason: self.fail(reason),
            next_segment=lambda: advances.append(True),
        )
        Mission.result(m, completed(N(status=4)))
        Mission.finish_segment(m)
        self.assertFalse(advances)
        m.pose = [0.8, 0.7248]
        m.progress_fraction = projected_progress(route[1][1], 14, m.pose)
        Mission.finish_segment(m)
        self.assertEqual(advances, [True])

    def test_real_short_entry_accepts_nav2_success_inside_metric_tolerance(self):
        from intramural_mission.route import line
        points = line([0., 0.], [-.8968332, -.00208], .05)
        advances = []
        m = N(segment_result=4, route=[('ENTER', points)], index=0, c=C,
              pose=[-.7941357, -.0127109], progress_fraction=15.93930324,
              next_segment=lambda: advances.append(True), result_at=time.monotonic()-1,
              fault=lambda reason: self.fail(reason))
        Mission.finish_segment(m)
        self.assertEqual(advances, [True])

    def test_return_keeps_parking_tolerance_and_rejects_untraversed_loop(self):
        from intramural_mission.route import line
        points = line([1., 0.], [0., 0.], .05)
        advances, faults = [], []
        m = N(segment_result=4, route=[('RETURN', points)], index=0, c=C,
              pose=[.14, 0.], progress_fraction=17.2,
              next_segment=lambda: advances.append(True), result_at=time.monotonic()-1,
              fault=faults.append)
        Mission.finish_segment(m)
        self.assertTrue(faults)
        self.assertFalse(advances)
        m.pose, m.progress_fraction = [.10, 0.], 18.
        Mission.finish_segment(m)
        self.assertEqual(advances, [True])

    def test_cancel_racing_with_completed_segment_does_not_advance(self):
        m = N(
            goal_handle=object(),
            state="CANCELLING",
            get_clock=lambda: N(now=lambda: N(nanoseconds=0)),
        )
        Mission.result(m, completed(N(status=4)))
        self.assertEqual(m.state, "CANCELLED")

    def test_server_abort_does_not_advance(self):
        route = segments(C)
        faults = []
        advances = []
        m = N(
            goal_handle=object(),
            state="RUNNING",
            route=route,
            index=0,
            pose=route[0][1][-1],
            progress=len(route[0][1]) - 1,
            c=C,
            fault=faults.append,
            next_segment=lambda: advances.append(True),
        )
        Mission.result(m, completed(N(status=6)))
        self.assertFalse(faults)  # Await the next published grid before classifying obstruction.
        self.assertIsNotNone(m.abort_at)
        self.assertFalse(advances)

    def test_reset_while_running_or_cancelling_is_rejected(self):
        for state in ("RUNNING", "CANCELLING"):
            r = Mission.reset(
                N(state=state, goal_handle=None, goal_pending=False),
                Trigger.Request(),
                Trigger.Response(),
            )
            self.assertFalse(r.success)

    def test_fault_reset_waits_for_pending_goal_acceptance(self):
        r = Mission.reset(
            N(state="FAULT", goal_handle=None, goal_pending=True),
            Trigger.Request(),
            Trigger.Response(),
        )
        self.assertFalse(r.success)

    def test_start_away_from_parking_is_rejected(self):
        m = N(
            start_permitted=True,
            variant='figure8',
            state="IDLE",
            navigator_active=True,
            pose=[3.5, 2.5],
            last_pose=time.monotonic(),
            ready_since=time.monotonic() - 2,
            tf_age=0.0,
            client=N(server_is_ready=lambda: True),
            c=C,
        )
        r = Mission.start(m, Trigger.Request(), Trigger.Response())
        self.assertFalse(r.success)

    def test_start_here_plans_around_wall_keeps_all_loops_and_original_parking(self):
        grid = OccupancyGrid()
        grid.header.frame_id = 'map'
        grid.info.width = grid.info.height = 160
        grid.info.resolution = .05
        grid.info.origin.position.x = grid.info.origin.position.y = -2.
        grid.info.origin.orientation.w = 1.
        data = np.zeros((160, 160), dtype=np.int8)
        data[:78, 55:58] = 100  # Wall separates start from entry; go around its top.
        grid.data = data.ravel().tolist()
        now = time.monotonic()
        paths, gate = [], []
        m = N(start_permitted=True,state='IDLE',navigator_active=True,pose=[-.5,0.],
            last_pose=now,ready_since=now-3,tf_age=0.,variant='figure8',c=C,run=0,
            static_map=grid,bypass_pub=Mock(),enabled=gate.append,
            get_clock=lambda:N(now=lambda:N(nanoseconds=0,to_msg=lambda:Time())),
            path_pub=N(publish=paths.append),accepted=lambda f,**kw:None,
            client=N(server_is_ready=lambda:True,send_goal_async=lambda g:completed(None)))
        m.make_path = lambda points: Mission.make_path(m, points)
        m.next_segment = lambda: Mission.next_segment(m)
        m.start = lambda req,res,**kw: Mission.start(m,req,res,**kw)
        result = Mission.start_here(m,Trigger.Request(),Trigger.Response())
        self.assertTrue(result.success, result.message)
        canonical = segments(C)
        self.assertEqual(m.route[0][1][0],m.pose)
        self.assertEqual(m.route[0][1][-1],canonical[0][1][-1])
        self.assertEqual(m.route[1:],canonical[1:])
        self.assertEqual(m.route[-1][1][-1],C['parking'])
        self.assertGreater(max(p[1] for p in m.route[0][1]),2.1)
        self.assertEqual(gate,[True])
        self.assertEqual(len(paths),1)
        m.bypassing = True
        Mission.next_segment(m)
        self.assertEqual(m.route[-1][1][-1],C['parking'])
        # Unknown cells sealing the gap must reject before opening the motion gate.
        data[78:,55:58] = -1
        grid.data = data.ravel().tolist()
        m.state, m.pose = 'IDLE', [-.5,0.]
        gate.clear()
        self.assertFalse(Mission.start_here(m,Trigger.Request(),Trigger.Response()).success)
        self.assertFalse(gate)
        self.assertIsNone(entry_path(C, m.pose, canonical[0][1][-1], None))

    def test_vehicle_standby_rejects_start_before_accessing_navigation(self):
        result = Mission.start(N(start_permitted=False), Trigger.Request(), Trigger.Response())
        self.assertFalse(result.success)

    def test_stop_and_wait_policy_never_enters_bypass_planner(self):
        Mission.try_bypass(N(c={"allow_bypass":False}))

    def test_cancel_closes_gate_before_action_request(self):
        events = []
        m = N(
            state="RUNNING",
            enabled=lambda value: events.append(("enabled", value)),
            goal_handle=N(cancel_goal_async=lambda: events.append(("cancel", True))),
        )
        r = Mission.cancel(m, Trigger.Request(), Trigger.Response())
        self.assertTrue(r.success)
        self.assertEqual(events, [("enabled", False), ("cancel", True)])
        self.assertEqual(m.state, "CANCELLING")

    def test_cancel_pending_acceptance_cancels_late_handle(self):
        events = []
        handle = N(
            accepted=True,
            get_result_async=lambda: completed(N(status=5)),
            cancel_goal_async=lambda: events.append("cancel"),
        )
        m = N(
            state="CANCELLING",
            result=lambda f,**kw: None,
            fault=lambda reason: events.append(reason),
        )
        Mission.accepted(m, completed(handle))
        self.assertEqual(events, ["cancel"])

    def test_completed_cancel_result_does_not_cancel_cleared_handle(self):
        handle = N(accepted=True, get_result_async=lambda: completed(N(status=5)))
        m = N(
            state="CANCELLING",
            goal_handle=None,
            get_clock=lambda: N(now=lambda: N(nanoseconds=0)),
            fault=lambda reason: self.fail(reason),
        )
        m.result = lambda f,**kw: Mission.result(m, f,**kw)
        Mission.accepted(m, completed(handle))
        self.assertEqual(m.state, "CANCELLED")
        self.assertIsNone(m.goal_handle)

    def test_invalid_tangent_geometry_is_rejected(self):
        c = dict(C)
        c["upper_pole"] = [c["upper_pole"][0] + 0.1, c["upper_pole"][1]]
        with self.assertRaises(ValueError):
            segments(c)


if __name__ == "__main__":
    unittest.main()
