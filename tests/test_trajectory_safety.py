"""Guard input contracts on an isolated ROS graph, without any serial or simulator."""

from array import array
import json
from pathlib import Path
import tempfile
import shutil
import time
from types import SimpleNamespace as N
import unittest
import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from std_srvs.srv import Trigger
from nav_msgs.msg import OccupancyGrid
from intramural_mission.adapter import Adapter
from intramural_mission.trajectory_mission import TrajectoryMission
from intramural_mission.trajectory import RoutePlan, OmniTracker
from intramural_mission.route import segments

ROOT = Path(__file__).resolve().parents[1]


class TrajectorySafetyTests(unittest.TestCase):
    def test_two_fresh_feasible_observations_resume_but_latest_raw_obstacle_still_stops(self):
        from collections import deque
        from intramural_mission.planning import GridMap
        c=json.loads((ROOT/'config/arena.json').read_text());c.update(allow_bypass=False,mission_variant='return',parking=[2.,0.])
        with tempfile.TemporaryDirectory(dir=ROOT/'cache') as folder:
            config=Path(folder)/'arena.json';config.write_text(json.dumps(c))
            rclpy.init(domain_id=89,args=['--ros-args','-p',f'arena_config:={config}'])
            node=TrajectoryMission();replies=deque()
            node.worker=N(poll=lambda:replies.popleft() if replies else None)
            node.plan=RoutePlan.build([('RETURN',[[float(x),0.] for x in np.linspace(0.,2.,41)])],c)
            node.tracker=OmniTracker(node.plan,c);node.run_id='current'
            def transform(*args):
                t=TransformStamped();t.header.stamp=node.get_clock().now().to_msg();t.transform.rotation.w=1.
                return t
            node.tf=N(lookup_transform=transform)
            grid=OccupancyGrid();grid.header.frame_id='map';grid.info.resolution=.05
            grid.info.width=100;grid.info.height=40;grid.info.origin.position.x=grid.info.origin.position.y=-1.
            grid.info.origin.orientation.w=1.;grid.data=[0]*4000
            node.receive_map(grid);node.measured=GridMap(grid,c)
            try:
                for raw in (False,True):
                    node.stop('BLOCKED');node.raw_hazard=raw;node.avoidance={}
                    node.receive_costmap(grid)
                    node.receive_guard(String(data=json.dumps(dict(sensors_fresh=True,route_id=node.plan.route_id))))
                    replies.append(dict(stamp=node.get_clock().now().nanoseconds/1e9,route_id=node.plan.route_id,
                        run_id='current',plan=None,blocked=False,execution_failure=False,imminent=False,
                        reason='',seconds=.01,tracks=[]))
                    node.tick()
                    self.assertEqual(node.state,'BLOCKED')
                    replies.append(dict(node.avoidance,stamp=node.get_clock().now().nanoseconds/1e9,plan=None))
                    node.tick()
                    self.assertEqual(node.state,'BLOCKED' if raw else 'RUNNING')
                    if raw:np.testing.assert_array_equal(node.tracker.command,[0.,0.])
                    else:self.assertGreater(np.linalg.norm(node.tracker.command),0.)
            finally:
                node.worker=None;node.destroy_node();rclpy.shutdown()

    def test_checked_escape_snapshot_accepts_once_but_latest_raw_point_and_cancel_still_win(self):
        from collections import deque
        from intramural_mission.avoidance import DetourPlanner
        c=json.loads((ROOT/'config/arena.json').read_text())
        c.update(json.loads((ROOT/'config/navigation-profiles.json').read_text())['candidate'])
        c.update(allow_bypass=False,mission_variant='return',bounds=[-2.,6.,-3.,3.],
            lower_pole=[-10.,-10.],upper_pole=[10.,10.],pole_clearance_from_map=True)
        c['drive_acceleration_limit']=c['max_acceleration']
        with tempfile.TemporaryDirectory(dir=ROOT/'cache') as folder:
            config=Path(folder)/'arena.json';config.write_text(json.dumps(c))
            rclpy.init(domain_id=89,args=['--ros-args','-p',f'arena_config:={config}'])
            node=TrajectoryMission();replies=deque()
            node.worker=N(poll=lambda:replies.popleft()if replies else None)
            try:
                grid=OccupancyGrid();grid.header.frame_id='map';grid.info.resolution=.05
                grid.info.width=160;grid.info.height=120
                grid.info.origin.position.x=-2.;grid.info.origin.position.y=-3.
                grid.info.origin.orientation.w=1.;grid.data=[0]*19200
                node.plan=RoutePlan.build([('RETURN',[[float(x),0.]for x in np.linspace(-1.,5.,241)])],c)
                node.tracker=OmniTracker(node.plan,c);node.tracker.progress.s=1.
                node.tracker.velocity=np.array([1.9,0.]);node.tracker.command=np.array([1.9,0.])
                node.pose=[0.,0.];node.run_id='escape';node.state='RUNNING'
                now=node.get_clock().now().nanoseconds/1e9;planner=DetourPlanner(c,grid)
                for i in range(16):
                    position=np.array([.8,-1.8])+np.array([0.,1.5])*(i-15)*.1
                    points=[position+[dx,dy]for dx in(-.1,0.,.1)for dy in(-.1,0.,.1)]
                    planner.observations(points,now+(i-15)*.1)
                result=planner.solve(dict(run_id=node.run_id,plan=node.plan,progress=1.,pose=node.pose,
                    velocity=[1.9,0.],command=[1.9,0.],acceleration=[0.,0.],points=points,stamp=now))
                self.assertEqual(result['reason'],'ESCAPE_READY',result)
                saved=dict(result);replies.append(result);node.update_avoidance()
                self.assertTrue(node.avoidance['escape_accepted'])
                deadline=node.escape_until
                self.assertAlmostEqual(deadline,now+.8,places=5)
                self.assertNotIn('escape_predictions',node.avoidance)
                # A far raw point is reconsidered next scan; it cannot freeze the entire two-second route.
                now=node.get_clock().now().nanoseconds/1e9
                node.observation=dict(stamp=now,points=[[2.5,0.]])
                node.submitted_stamp=now
                replies.append(dict(saved));node.update_avoidance()
                self.assertTrue(node.avoidance['escape_accepted'])
                self.assertEqual(node.escape_until,deadline)
                # A newly visible raw point overrides the otherwise certified moving-target escape.
                now=node.get_clock().now().nanoseconds/1e9
                node.observation=dict(stamp=now,points=[[.2,0.]])
                node.submitted_stamp=now
                replies.append(saved);node.update_avoidance()
                self.assertFalse(node.avoidance.get('escape_accepted',False))
                self.assertEqual(node.escape_until,deadline)
                self.assertTrue(node.cancel(None,Trigger.Response()).success)
                self.assertEqual(node.state,'CANCELLING')
                np.testing.assert_array_equal(node.tracker.command,[0.,0.])
            finally:
                node.worker=None;node.destroy_node();rclpy.shutdown()

    def test_real_worker_speed_restore_does_not_jump_to_the_returning_branch(self):
        from dataclasses import replace
        c=json.loads((ROOT/'config/arena.json').read_text())
        c.update(allow_bypass=True,mission_variant='return',parking=[0.,.6],
            lower_pole=[-10.,-10.],upper_pole=[10.,10.],response_models=[[0.,0.]],
            response_delay=0.,response_tau=0.,require_stress_models=False)
        with tempfile.TemporaryDirectory(dir=ROOT/'cache') as folder:
            config=Path(folder)/'arena.json';config.write_text(json.dumps(c))
            rclpy.init(domain_id=89,args=['--ros-args','-p',f'arena_config:={config}'])
            node=TrajectoryMission()
            grid=OccupancyGrid();grid.header.frame_id='map';grid.info.resolution=.05
            grid.info.width=grid.info.height=80;grid.info.origin.position.x=grid.info.origin.position.y=-2.
            grid.info.origin.orientation.w=1.;grid.data=[0]*6400
            node.receive_map(grid)
            try:
                self.assertTrue(node.worker.ready.wait(5.))
                points=([[float(x),0.]for x in np.linspace(0.,.35,15)]+
                    [[.35,float(y)]for y in np.linspace(.01,.04,4)]+
                    [[float(x),.04]for x in np.linspace(.325,0.,14)]+
                    [[0.,float(y)]for y in np.linspace(.08,.6,14)])
                fast=RoutePlan.build([('RETURN',points)],c)
                slow=replace(fast,route_id='slow',speed=tuple(v*.3 for v in fast.speed))
                node.plan=slow;node.tracker=OmniTracker(slow,c);node.tracker.progress.s=.16
                node.pose=[.16,.025];node.run_id='switch';node.state='RUNNING'
                node.speed_restore=fast
                for _ in range(2):
                    stamp=node.get_clock().now().nanoseconds/1e9
                    node.worker.submit(dict(run_id='switch',plan=slow,restore_plan=fast,progress=.15,
                        pose=[.15,.025],velocity=[0.,0.],command=[0.,0.],acceleration=[0.,0.],
                        points=[],stamp=stamp))
                    deadline=time.monotonic()+5.
                    while node.avoidance.get('stamp',0.)<stamp and time.monotonic()<deadline:
                        node.update_avoidance();time.sleep(.005)
                    self.assertGreaterEqual(node.avoidance.get('stamp',0.),stamp)
                self.assertNotEqual(node.plan.route_id,slow.route_id)
                self.assertAlmostEqual(node.tracker.progress.s,.16)
                self.assertEqual(node.tracker.progress.phase,0)
                node.plan=slow;node.tracker.plan=slow;node.pose=[0.,.5]
                node.state='BLOCKED';node.execution_blocked=True
                stamp=node.get_clock().now().nanoseconds/1e9
                node.worker.submit(dict(run_id='switch',plan=slow,restore_plan=fast,progress=.15,
                    pose=[.15,.025],velocity=[0.,0.],command=[0.,0.],acceleration=[0.,0.],
                    points=[],stamp=stamp))
                deadline=time.monotonic()+5.
                while node.avoidance.get('stamp',0.)<stamp and time.monotonic()<deadline:
                    node.update_avoidance();time.sleep(.005)
                self.assertGreaterEqual(node.avoidance.get('stamp',0.),stamp)
                self.assertEqual(node.plan.route_id,slow.route_id)
                self.assertTrue(node.execution_blocked)
                self.assertEqual(node.state,'BLOCKED')
            finally:
                if node.worker:node.worker.close();node.worker=None
                node.destroy_node();rclpy.shutdown()

    def test_missing_tf_lookup_keeps_only_the_still_fresh_snapshot(self):
        from tf2_ros import LookupException
        c=json.loads((ROOT/'config/arena.json').read_text());c['allow_bypass']=False
        with tempfile.TemporaryDirectory(dir=ROOT/'cache') as folder:
            config=Path(folder)/'arena.json';config.write_text(json.dumps(c))
            rclpy.init(domain_id=89,args=['--ros-args','-p',f'arena_config:={config}'])
            node=TrajectoryMission()
            try:
                t=TransformStamped();t.header.stamp=node.get_clock().now().to_msg();t.transform.rotation.w=1.
                node.tf=N(lookup_transform=lambda *args:t);node.tick();pose=node.pose.copy()
                def missing(*args):raise LookupException('Injected missing transform')
                node.tf=N(lookup_transform=missing);node.tick()
                self.assertEqual(node.pose,pose);self.assertLess(node.pose_age,.25)
                node.pose_stamp-=.3;node.tick()
                self.assertIsNone(node.pose);self.assertGreaterEqual(node.pose_age,.25)
                self.assertIn('Injected missing',node.pose_problem)
            finally:node.destroy_node();rclpy.shutdown()

    def test_old_planner_results_and_expired_continuation_cannot_keep_motion(self):
        from collections import deque
        c=json.loads((ROOT/'config/arena.json').read_text());c['allow_bypass']=False
        with tempfile.TemporaryDirectory(dir=ROOT/'cache') as folder:
            config=Path(folder)/'arena.json';config.write_text(json.dumps(c))
            rclpy.init(domain_id=89,args=['--ros-args','-p',f'arena_config:={config}'])
            node=TrajectoryMission();replies=deque()
            node.worker=N(poll=lambda:replies.popleft() if replies else None)
            node.plan=RoutePlan.build([('RETURN',[[float(x),0.] for x in np.linspace(0.,2.,41)])],c)
            node.tracker=OmniTracker(node.plan,c);node.tracker.command=np.array([.4,0.])
            node.pose=[0.,0.];node.run_id='current';node.state='RUNNING'
            now=node.get_clock().now().nanoseconds/1e9
            base=dict(stamp=now,route_id=node.plan.route_id,run_id='current',plan=None,
                blocked=True,imminent=False,reason='PREDICTED_CONFLICT',seconds=.01,tracks=[])
            try:
                for changes in (dict(run_id='previous'),dict(stamp=now-1.),dict(route_id='previous')):
                    replies.append(dict(base,**changes));node.update_avoidance()
                    self.assertEqual(node.state,'RUNNING');self.assertEqual(node.avoidance,{})
                replies.append(dict(base,continuation_until=now+1.));node.update_avoidance()
                self.assertEqual(node.state,'RUNNING');self.assertFalse(node.predictive_stop)
                node.continuation_until=now-1.;node.update_avoidance()
                self.assertEqual(node.state,'BLOCKED');np.testing.assert_array_equal(node.tracker.command,[0.,0.])
                node.state='RUNNING';node.continuation_until=0.
                replies.append(dict(base,plan=node.plan,seconds=.09,continuation_until=now+10.))
                node.update_avoidance()
                self.assertEqual(node.state,'BLOCKED');self.assertEqual(node.avoidance['reason'],'PLANNING_TIMEOUT')
                node.state='RUNNING';node.continuation_until=now-1.
                replies.append(dict(base,blocked=False,seconds=.09));node.update_avoidance()
                self.assertEqual(node.state,'BLOCKED')
            finally:
                node.worker=None;node.destroy_node();rclpy.shutdown()

    def test_parking_window_completes_without_restarting_correction(self):
        from intramural_mission.planning import GridMap
        c=json.loads((ROOT/'config/arena.json').read_text())
        c.update(json.loads((ROOT/'config/navigation-profiles.json').read_text())['candidate'])
        c.update(parking=[2.,0.],lower_pole=[-5.,-5.],upper_pole=[-5.,5.],mission_variant='return')
        with tempfile.TemporaryDirectory(dir=ROOT/'cache') as folder:
            config=Path(folder)/'arena.json';config.write_text(json.dumps(c))
            rclpy.init(domain_id=88,args=['--ros-args','-p',f'arena_config:={config}'])
            node=TrajectoryMission();position=[1.95,0.]
            def transform(*args):
                t=TransformStamped();t.header.stamp=node.get_clock().now().to_msg();t.transform.rotation.w=1.
                t.transform.translation.x,t.transform.translation.y=position
                return t
            node.tf=N(lookup_transform=transform)
            grid=OccupancyGrid();grid.header.frame_id='map';grid.info.resolution=.05
            grid.info.width=100;grid.info.height=40;grid.info.origin.position.x=grid.info.origin.position.y=-1.
            grid.info.origin.orientation.w=1.;grid.data=[0]*4000
            try:
                node.plan=RoutePlan.build([('RETURN',[[float(x),0.] for x in np.linspace(0.,2.,81)])],c)
                node.tracker=OmniTracker(node.plan,c);node.tracker.progress.s=1.95
                node.measured=GridMap(grid,c);node.state='RUNNING';node.index=0
                node.receive_map(grid)
                for x in (1.95,1.93,1.95,1.96,1.94):
                    position[0]=x;node.receive_costmap(grid)
                    node.receive_guard(String(data='{"sensors_fresh":true}'));node.tick()
                    np.testing.assert_array_equal(node.tracker.command,[0.,0.])
                    time.sleep(.1)
                self.assertEqual(node.state,'DONE',node.detail)
            finally:
                node.destroy_node();rclpy.shutdown()

    def test_prepare_stays_disabled_and_start_reuses_it_or_rejects_changed_pose(self):
        from std_msgs.msg import Bool
        c=json.loads((ROOT/'config/arena.json').read_text());c['optimize_route']=False
        with tempfile.TemporaryDirectory(dir=ROOT/'cache') as folder:
            config=Path(folder)/'arena.json';config.write_text(json.dumps(c))
            rclpy.init(domain_id=87,args=['--ros-args','-p',f'arena_config:={config}'])
            node=TrajectoryMission();position=list(c['parking']);enabled=[]
            node.create_subscription(Bool,'mission/enabled',lambda m:enabled.append(m.data),10)
            def transform(*args):
                t=TransformStamped();t.header.stamp=node.get_clock().now().to_msg();t.transform.rotation.w=1.
                if args[1]=='base_footprint':
                    t.transform.translation.x,t.transform.translation.y=position
                return t
            node.tf=N(lookup_transform=transform)
            grid=OccupancyGrid();grid.header.frame_id='map';grid.info.resolution=.05
            grid.info.width=grid.info.height=200;grid.info.origin.position.x=grid.info.origin.position.y=-2.
            grid.info.origin.orientation.w=1.;grid.data=[0]*40000
            def fresh():
                node.receive_map(grid);node.receive_costmap(grid)
                node.receive_guard(String(data='{"sensors_fresh":true}'));node.tick()
            def call(name):
                client=node.create_client(Trigger,'mission/'+name)
                try:
                    self.assertTrue(client.wait_for_service(timeout_sec=2.))
                    future=client.call_async(Trigger.Request());rclpy.spin_until_future_complete(node,future,timeout_sec=10.)
                    self.assertTrue(future.done());return future.result()
                finally:node.destroy_client(client)
            try:
                fresh();response=call('prepare');self.assertTrue(response.success,response.message)
                self.assertEqual(node.state,'PREPARED');self.assertEqual(node.run,0)
                self.assertEqual(node.tracker.command.tolist(),[0.,0.]);self.assertNotIn(True,enabled)
                position[0]+=.2;fresh();response=call('start')
                self.assertFalse(response.success);self.assertEqual(node.state,'PREPARED');self.assertEqual(node.run,0)
                position[:]=c['parking'];grid.data[0]=-1;fresh();response=call('start')
                self.assertFalse(response.success);self.assertEqual(node.run,0)
                grid.data[0]=0
                position[:]=c['parking'];fresh();response=call('prepare');self.assertTrue(response.success,response.message)
                prepared=node.plan;fresh();response=call('start')
                self.assertTrue(response.success,response.message);self.assertIs(node.plan,prepared)
                self.assertIn(node.state,('STARTING','RUNNING'));self.assertEqual(node.run,1)
                self.assertTrue(node.report['reused_prepared_start']);self.assertEqual(node.report['planning_seconds'],0.)
            finally:
                if hasattr(node,'log'):node.log.close()
                if node.run_id:shutil.rmtree(ROOT/'artifacts/navigation-runs'/node.run_id)
                node.destroy_node();rclpy.shutdown()

    def test_odometry_coasting_still_blocks_after_zero_command_and_off_route_turn(self):
        from nav_msgs.msg import Odometry
        from geometry_msgs.msg import Twist
        c=json.loads((ROOT/'config/arena.json').read_text())
        c.update(navigation_engine='trajectory',raw_collision_follow_path=True,
                 lidar_pose=[0.]*6,max_speed=2.,response_delay=.2,response_tau=.15,
                 raw_collision_min_points=3)
        with tempfile.TemporaryDirectory(dir=ROOT/'cache') as folder:
            config=Path(folder)/'arena.json';config.write_text(json.dumps(c))
            rclpy.init(domain_id=86,args=['--ros-args','-p',f'arena_config:={config}','-p','localization:=sensors'])
            guard=Adapter();peer=rclpy.create_node('coasting_odometry_fixture')
            pub=peer.create_publisher(Odometry,'odometry',10)
            try:
                for x,offset in ((0.,.2),(.08,.1)):
                    m=Odometry();m.header.frame_id='odom';m.child_frame_id='base_footprint'
                    now=guard.get_clock().now().nanoseconds/1e9
                    stamp=now-offset;m.header.stamp.sec=int(stamp);m.header.stamp.nanosec=int((stamp-int(stamp))*1e9)
                    m.pose.pose.position.x=x;m.pose.pose.orientation.z=np.sin(np.pi/4);m.pose.pose.orientation.w=np.cos(np.pi/4)
                    deadline=time.monotonic()+2
                    while guard.previous_odometry is None or guard.previous_odometry[0]<stamp-1e-8:
                        self.assertLess(time.monotonic(),deadline)
                        pub.publish(m);rclpy.spin_once(guard,timeout_sec=.005)
                self.assertLess(guard.body_velocity[1],-.5)
                context=dict(phase='ENTER',state='RUNNING',run_id='coast',route_id='route')
                guard.receive_mission_status(String(data=json.dumps(context)))
                stamp=guard.get_clock().now().nanoseconds/1e9
                guard.receive_trajectory(String(data=json.dumps(dict(run_id='coast',route_id='route',stamp=stamp,
                    pose=[0.,0.,0.],pose_stamp=stamp,points=[[0.,0.],[1.,0.],[2.,0.]]))))
                cloud=PointCloud2(height=1,width=3,point_step=12,row_step=36)
                cloud.header.stamp=guard.get_clock().now().to_msg()
                cloud.fields=[PointField(name=k,offset=4*i,datatype=7,count=1) for i,k in enumerate('xyz')]
                cloud.data=array('B',np.array([[0.,-.48,.3],[.01,-.48,.3],[-.01,-.48,.3]],dtype='<f4').tobytes())
                guard.lidar(cloud);guard.touch('imu');guard.touch('command');guard.enabled=True
                guard.controller_command(Twist())
                guard.tick()
                self.assertTrue(guard.hazard)
                self.assertEqual(guard.output,[0.,0.])
                self.assertLess(guard.body_velocity[1],-.5,'Zero command must not erase physical momentum')
            finally:
                peer.destroy_node();guard.destroy_node();rclpy.shutdown()

    def test_planning_waits_for_new_localization_before_enabling_motion(self):
        c=json.loads((ROOT/'config/arena.json').read_text())
        c.update(optimize_route=False)
        with tempfile.TemporaryDirectory(dir=ROOT/'cache') as folder:
            config=Path(folder)/'arena.json';config.write_text(json.dumps(c))
            rclpy.init(domain_id=79,args=['--ros-args','-p',f'arena_config:={config}'])
            node=TrajectoryMission();age=[0.]
            try:
                def transform(*args):
                    t=TransformStamped();stamp=node.get_clock().now().nanoseconds/1e9-age[0]
                    t.header.stamp.sec=int(stamp);t.header.stamp.nanosec=int((stamp-int(stamp))*1e9)
                    t.transform.rotation.w=1.;return t
                node.tf=N(lookup_transform=transform)
                grid=OccupancyGrid();grid.header.frame_id='map';grid.info.resolution=.05
                grid.info.width=grid.info.height=200;grid.info.origin.position.x=grid.info.origin.position.y=-2.
                grid.info.origin.orientation.w=1.;grid.data=[0]*40000
                node.receive_map(grid);node.receive_costmap(grid)
                node.receive_guard(String(data='{"sensors_fresh":true}'));node.tick()
                response=node.start(Trigger.Request(),Trigger.Response())
                self.assertTrue(response.success,response.message)
                age[0]=.261;node.tick()
                self.assertEqual(node.state,'STARTING')
                self.assertEqual(node.tracker.command.tolist(),[0.,0.])
                self.assertFalse(node.reset(Trigger.Request(),Trigger.Response()).success)
                age[0]=0.;node.receive_costmap(grid)
                node.receive_guard(String(data='{"sensors_fresh":true}'));node.tick()
                self.assertEqual(node.state,'RUNNING')
                # Once moving, the original 0.25 s localization limit still stops the run.
                age[0]=.261;node.tick()
                self.assertEqual(node.state,'FAULT')
                self.assertEqual(node.reason,'LOCALIZATION_OR_SENSOR_STALE')
            finally:
                if hasattr(node,'log'):node.log.close()
                if node.run_id:shutil.rmtree(ROOT/'artifacts/navigation-runs'/node.run_id)
                node.destroy_node();rclpy.shutdown()

    def test_locked_standby_never_reports_ready_with_healthy_inputs(self):
        with tempfile.TemporaryDirectory(dir=ROOT / "cache") as folder:
            config = Path(folder) / "arena.json"
            config.write_bytes((ROOT / "config/arena.json").read_bytes())
            rclpy.init(domain_id=77, args=["--ros-args", "-p", f"arena_config:={config}",
                                         "-p", "start_permitted:=false"])
            node = TrajectoryMission()
            statuses = []
            node.create_subscription(String, "mission/status",
                                     lambda msg: statuses.append(json.loads(msg.data)), 10)
            try:
                def transform(*args):
                    t = TransformStamped()
                    t.header.stamp = node.get_clock().now().to_msg()
                    t.transform.rotation.w = 1.0
                    return t

                node.tf = N(lookup_transform=transform)
                node.receive_map(OccupancyGrid())
                node.receive_costmap(OccupancyGrid())
                node.receive_guard(String(data='{"sensors_fresh": true}'))
                node.tick()
                deadline = time.monotonic() + 2
                while not statuses and time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=.02)
                self.assertTrue(statuses)
                self.assertIsNotNone(statuses[-1]["pose"])
                self.assertFalse(statuses[-1]["start_permitted"])
                self.assertFalse(statuses[-1]["ready"])
                response = node.start(Trigger.Request(), Trigger.Response())
                self.assertFalse(response.success)
                self.assertEqual(node.state, "IDLE")
                self.assertIsNone(node.tracker)
            finally:
                node.destroy_node()
                rclpy.shutdown()

    def test_missing_localization_latches_fault_and_clears_motion(self):
        c = json.loads((ROOT / "config/arena.json").read_text())
        c.update(json.loads((ROOT / "config/navigation-profiles.json").read_text())["safe"])
        with tempfile.TemporaryDirectory(dir=ROOT / "cache") as folder:
            config = Path(folder) / "arena.json"
            config.write_text(json.dumps(c))
            rclpy.init(
                domain_id=79, args=["--ros-args", "-p", f"arena_config:={config}"]
            )
            node = TrajectoryMission()
            try:
                node.plan = RoutePlan.build(segments(c), c)
                node.tracker = OmniTracker(node.plan, c)
                node.tracker.command[:] = [0.4, 0.0]
                node.state = "RUNNING"
                node.tick()
                self.assertEqual(node.state, "FAULT")
                self.assertEqual(node.reason, "LOCALIZATION_OR_SENSOR_STALE")
                self.assertEqual(node.tracker.command.tolist(), [0.0, 0.0])
                # The fault cannot silently restart when a later tick receives data.
                node.tick()
                self.assertEqual(node.state, "FAULT")
            finally:
                node.destroy_node()
                rclpy.shutdown()

    def test_old_run_path_and_empty_sensor_heartbeat_cannot_enable_motion(self):
        c = json.loads((ROOT / "config/arena.json").read_text())
        c.update(json.loads((ROOT / "config/navigation-profiles.json").read_text())["safe"])
        c.update(navigation_engine="trajectory", raw_collision_follow_path=True)
        with tempfile.TemporaryDirectory(dir=ROOT / "cache") as folder:
            config = Path(folder) / "arena.json"
            config.write_text(json.dumps(c))
            rclpy.init(
                domain_id=78,
                args=[
                    "--ros-args",
                    "-p",
                    f"arena_config:={config}",
                    "-p",
                    "localization:=sensors",
                ],
            )
            a = Adapter()
            try:

                def transform(*args):
                    t = TransformStamped()
                    t.header.stamp = a.get_clock().now().to_msg()
                    t.transform.rotation.w = 1.0
                    return t

                a.path_tf = N(lookup_transform=transform)
                a.receive_mission_status(
                    String(
                        data=json.dumps(
                            dict(
                                phase="ENTER",
                                state="RUNNING",
                                run_id="new",
                                route_id="route",
                            )
                        )
                    )
                )
                path = dict(
                    run_id="old",
                    route_id="route",
                    stamp=a.get_clock().now().nanoseconds / 1e9,
                    pose=[2.,3.,np.pi/2],
                    pose_stamp=a.get_clock().now().nanoseconds / 1e9,
                    points=[[2.0, 3.0], [2.0, 4.0]],
                )
                a.receive_trajectory(String(data=json.dumps(path)))
                self.assertIsNone(a.body_path())
                path["run_id"] = "new"
                def unavailable(*args):
                    raise AssertionError('A separate TF lookup must not interrupt the coherent trajectory')
                a.path_tf = N(lookup_transform=unavailable)
                a.receive_trajectory(String(data=json.dumps(path)))
                np.testing.assert_allclose(a.body_path(),[[0.,0.],[1.,0.]],atol=1e-12)
                path['pose_stamp']-=1.
                path['stamp']+=.0001
                a.receive_trajectory(String(data=json.dumps(path)))
                self.assertIsNone(a.body_path())
                path['pose_stamp']+=1.
                path["stamp"] -= 1.0
                a.receive_trajectory(String(data=json.dumps(path)))
                self.assertIsNone(a.body_path())
                cloud = PointCloud2()
                cloud.header.stamp = a.get_clock().now().to_msg()
                cloud.height = 1
                cloud.width = 1
                cloud.point_step = 12
                cloud.row_step = 12
                cloud.fields = [
                    PointField(name=name, offset=4 * i, datatype=7, count=1)
                    for i, name in enumerate("xyz")
                ]
                cloud.data = array("B", np.zeros((1, 3), dtype="<f4").tobytes())
                a.lidar(cloud)
                self.assertNotIn("lidar", a.last)
                cloud.data = array(
                    "B", np.array([[1.0, 0.0, 0.5]], dtype="<f4").tobytes()
                )
                a.lidar(cloud)
                self.assertIn("lidar", a.last)
                a.enabled = True
                a.tick()
                self.assertEqual(a.output, [0.0, 0.0])
            finally:
                a.destroy_node()
                rclpy.shutdown()


if __name__ == "__main__":
    unittest.main()
