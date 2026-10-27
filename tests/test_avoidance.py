"""Failure cases: association, stale scans, crossing, topology and moving splice."""
import unittest
import numpy as np
from intramural_mission.avoidance import ObstacleTracker, collision, splice
from intramural_mission.trajectory import RoutePlan


class AvoidanceTests(unittest.TestCase):
    def test_safe_crawl_still_searches_a_faster_detour_when_recovery_is_blocked(self):
        from dataclasses import replace
        from intramural_mission.avoidance import DetourPlanner,preview
        c,grid,fast,points=self.case()
        slow=replace(fast,speed=tuple(min(.1,v)if s<2. else v for s,v in zip(fast.s,fast.speed)),route_id='crawl')
        planner=DetourPlanner(c,grid)
        for i in range(6):planner.observations(points,i*.1)
        current,times=preview(slow,0.,.1)
        self.assertFalse(collision(current,times,planner.observations(points,.6),c['robot_radius']))
        result=None
        for i in range(8):
            result=planner.solve(dict(plan=slow,restore_plan=fast,progress=0.,pose=[0.,0.],
                velocity=[.1,0.],command=[.1,0.],acceleration=[0.,0.],points=points,stamp=.7+i*.1))
            if result['plan'] is not None:break
        self.assertIsNotNone(result['plan'],result)
        self.assertEqual(result['reason'],'DETOUR_READY')
        self.assertFalse(result['blocked'])
        self.assertNotEqual(result['plan'].points,slow.points)

    def test_return_cannot_change_sides_around_an_already_avoided_obstacle(self):
        from intramural_mission.avoidance import route_topology
        from intramural_mission.bypass import winding
        c,_,_,_=self.case()
        route=[('ENTER',[[0.,0.],[1.,1.],[2.,0.]])]
        previous=[2.,0.]
        for n,angle in enumerate(np.linspace(np.pi,3*np.pi,9)[1:]):
            point=[2.4+.4*np.cos(angle),.4*np.sin(angle)]
            route.append((str(n),[previous,point]));previous=point
        route.append(('RETURN',[[2.,0.],[1.,1.],[0.,0.]]))
        reference=RoutePlan.build(route,c)
        wrong=RoutePlan.build(route[:-1]+[('RETURN',[[2.,0.],[1.,-1.],[0.,0.]])],c)
        obstacle=dict(center=[1.,0.],extent=[.15,.15],radius=.22,
            covariance=np.zeros((4,4)).tolist(),age=0.,moving=False,shared_margin=.02)
        constraints=route_topology(reference,c,[obstacle],9)
        self.assertEqual(len(constraints),3)
        self.assertEqual(len(route_topology(reference,c,[obstacle],8)),3)
        self.assertTrue(all(winding(reference.points,p)==v for p,v in constraints))
        self.assertFalse(all(winding(wrong.points,p)==v for p,v in constraints))
        self.assertTrue(all(winding(wrong.points,c[k])==winding(reference.points,c[k])
                            for k in ('lower_pole','upper_pole')))
        obstacle['center']=[1.,1.]
        self.assertEqual(len(route_topology(reference,c,[obstacle],9)),2)
        self.assertEqual(len(route_topology(reference,c,[obstacle],0)),2)

    def test_splice_assigns_joint_locations_in_order_after_a_long_detour(self):
        from intramural_mission.route import line
        c,_,_,_=self.case()
        names=['ENTER','LOWER','LOWER','LOWER','UPPER','UPPER','UPPER','UPPER','LOWER_EXIT','RETURN']
        plan=RoutePlan.build([(name,line([i,0.],[i+1.,0.],.025))for i,name in enumerate(names)],c)
        vertices=[[6.5,0.],[6.55,.9],[7.,.9],[8.,0.],[9.5,0.]]
        bridge=[]
        for a,b in zip(vertices,vertices[1:]):bridge.extend(line(a,b,.025)[1 if bridge else 0:])
        updated,_=splice(plan,6.5,bridge,9.5,c,lambda p:np.ones(len(p)),0.)
        self.assertEqual([updated.points[i]for _,i in updated.gates[:6]],
                         [plan.points[i]for _,i in plan.gates[:6]])
        self.assertLess(np.linalg.norm(np.asarray(updated.points[updated.gates[7][1]])-[8.,0.]),.026)
        self.assertTrue(all(a[1]<b[1]for a,b in zip(updated.gates,updated.gates[1:])))

    def test_blocked_next_crossing_can_rejoin_after_the_joint_without_skipping_gates(self):
        from intramural_mission.avoidance import DetourPlanner
        c,grid,_,_=self.case()
        names=['ENTER','LOWER','LOWER','LOWER','UPPER','UPPER','UPPER','UPPER','LOWER_EXIT','RETURN']
        route=[(name,[[x,0.]for x in np.linspace(i*.5,(i+1)*.5,11)])for i,name in enumerate(names)]
        plan=RoutePlan.build(route,c);points=[[2.+x,y]for x in(-.1,0.,.1)for y in(-.1,0.,.1)]
        planner=DetourPlanner(c,grid)
        for i in range(6):planner.observations(points,i*.1)
        result=None
        for i in range(8):
            result=planner.solve(dict(plan=plan,original_plan=plan,phase=2,progress=1.1,pose=[1.1,0.],
                velocity=[.3,0.],command=[.3,0.],acceleration=[0.,0.],points=points,stamp=.6+i*.1))
            if result['plan'] is not None:break
        self.assertIsNotNone(result['plan'],result)
        selected=result['plan']
        self.assertEqual(selected.gates[:2],plan.gates[:2])
        self.assertEqual([name for name,_ in selected.gates],names)
        self.assertGreater(result['join_s'],plan.s[plan.gates[3][1]])
        self.assertEqual(selected.points[-1],plan.points[-1])

    def test_revealing_an_orthogonal_face_is_not_a_velocity_jump(self):
        tracker=ObstacleTracker()
        top=[[2.+x,.25] for x in np.linspace(-.25,.25,11)]
        side=[[2.25,y] for y in np.linspace(-.25,.25,11)]
        for i in range(10):tracker.update(top,i*.1,sensor_position=[0.,1.])
        track=tracker.update(side,1.,sensor_position=[3.,0.])[0]
        self.assertLess(np.linalg.norm(track['velocity']),.1)
        np.testing.assert_allclose(track['center'],[2.,0.],atol=.04)

    def test_real_motion_remains_observable_while_another_face_is_revealed(self):
        tracker=ObstacleTracker()
        top=[[2.+x,.25] for x in np.linspace(-.25,.25,11)]
        for i in range(10):tracker.update(top,i*.1,sensor_position=[0.,1.])
        side=[[2.45,y] for y in np.linspace(-.25,.25,11)]
        track=tracker.update(side,1.,sensor_position=[3.,0.])[0]
        self.assertAlmostEqual(track['velocity'][0],2.,delta=.15)
        self.assertLess(abs(track['velocity'][1]),.1)

    def test_noisy_static_observations_keep_bounded_position_and_velocity_estimates(self):
        tracker=ObstacleTracker();rng=np.random.default_rng(31);errors=[];speeds=[]
        for i in range(200):
            center=np.array([2.,0.])+rng.normal(0.,.015,2)
            cloud=center+np.array([[x,y] for x in (-.25,0.,.25) for y in (-.25,0.,.25)])
            track=tracker.update(cloud,i*.05)[0]
            if i>20:
                errors.append(np.linalg.norm(np.asarray(track['center'])-[2.,0.]))
                speeds.append(np.linalg.norm(track['velocity']))
        self.assertLess(np.mean(errors),.04)
        self.assertLess(np.percentile(speeds,95),.4)

    def test_association_jump_keeps_occupancy_but_cannot_authorize_escape(self):
        tracker=ObstacleTracker()
        for i in range(20):
            tracker.update([[2.+dx,dy]for dx in(-.1,0.,.1)for dy in(-.1,0.,.1)],i*.05)
        shifted=[[2.+dx,.4+dy]for dx in(-.1,0.,.1)for dy in(-.1,0.,.1)]
        track=tracker.update(shifted,1.)[0]
        self.assertTrue(track['velocity_observed'])
        self.assertGreater(np.linalg.norm(track['velocity']),1.)
        self.assertLess(np.linalg.norm(np.asarray(track['center'])-[2.,.4]),.1)
        self.assertFalse(track['motion_confident'])
        for i in range(1,6):
            cloud=[[2.+dx,.4+dy+i*.05]for dx in(-.1,0.,.1)for dy in(-.1,0.,.1)]
            track=tracker.update(cloud,1.+i*.05)[0]
        self.assertTrue(track['motion_confident'])

    def test_slow_fragment_motion_on_a_static_face_does_not_authorize_escape(self):
        tracker=ObstacleTracker()
        for i in range(24):
            y=max(0,i-19)*.02
            cloud=[[1.75,y+dy]for dy in(-.05,0.,.05)]
            track=tracker.update(cloud,i*.1)[0]
        self.assertTrue(track['velocity_observed'])
        self.assertGreater(np.linalg.norm(track['velocity']),.08)
        self.assertFalse(track['motion_confident'])

    def test_unobserved_acceleration_does_not_inflate_a_static_face_across_the_map(self):
        tracker=ObstacleTracker()
        for i in range(24):
            y=max(0,i-19)*.02
            track=tracker.update([[1.75,y+dy]for dy in(-.05,0.,.05)],i*.1)[0]
        self.assertFalse(track.get('acceleration_observed',False))
        center=np.asarray(track['center'])
        self.assertTrue(collision([center,center],[1.95,2.],[track],.29))
        self.assertFalse(collision([center+[2.,0.],center+[2.,0.]],[1.95,2.],[track],.29))

    def test_moving_stop_conflict_allows_only_checked_escape_and_does_not_extend_deadline(self):
        import json
        from pathlib import Path
        from intramural_mission.avoidance import DetourPlanner,stopped_collision,collision_segments
        from intramural_mission.response import predict_motion
        c,grid,_,_=self.case()
        c.update(json.loads((Path(__file__).resolve().parents[1]/'config/navigation-profiles.json').read_text())['candidate'])
        c['drive_acceleration_limit']=c['max_acceleration']
        plan=RoutePlan.build([('RETURN',[[float(x),0.]for x in np.linspace(-1.,5.,241)])],c)
        planner=DetourPlanner(c,grid)
        for i in range(16):
            position=np.array([.8,-1.8])+np.array([0.,1.5])*(i-15)*.1
            points=[position+[dx,dy]for dx in(-.1,0.,.1)for dy in(-.1,0.,.1)]
            planner.observations(points,i*.1)
        request=dict(plan=plan,progress=1.,pose=[0.,0.],velocity=[1.9,0.],command=[1.9,0.],
            acceleration=[0.,0.],points=points,stamp=1.5)
        result=planner.solve(request)
        self.assertEqual(result['reason'],'ESCAPE_READY',result)
        self.assertTrue(result['escape_active'])
        self.assertLessEqual(result['escape_until']-request['stamp'],.8+1e-9)
        self.assertEqual(result['plan'].gates,plan.gates)
        self.assertEqual(len(result['escape_predictions']),len(c['response_models']))
        radius=c['robot_radius']+c['minimum_clearance']
        initial=predict_motion(plan,c,[.152,0.],[1.9,0.],[1.9,0.],[0.,0.],1.,horizon=0.)
        truth=[dict(t,covariance=np.zeros((6,6)).tolist())for t in result['tracks']]
        self.assertTrue(all(collision(b[0],bt[0],truth,radius)
            or stopped_collision(b,bt,truth,radius)for _,_,b,bt,_ in initial))
        execution_tracks=[dict(t,prediction_age=t['prediction_age']+.08)for t in result['tracks']]
        for trajectory in result['escape_predictions']:
            self.assertFalse(collision(trajectory['points'],trajectory['times'],execution_tracks,radius))
        for _,timing,brakes,brake_times,_ in predict_motion(plan,c,[.152,0.],[1.9,0.],[1.9,0.],[0.,0.],1.):
            eligible=timing>=.7
            branches,stamps=brakes[eligible],brake_times[eligible]
            self.assertFalse(collision_segments(branches[:,:-1].reshape(-1,2),branches[:,1:].reshape(-1,2),
                stamps[:,:-1].ravel(),stamps[:,1:].ravel(),execution_tracks,radius))
            self.assertFalse(stopped_collision(branches,stamps,execution_tracks,radius))
        # Replanning keeps the original deadline even while stopping remains unsafe.
        expired=planner.solve(dict(request,escape_until=request['stamp']+.05))
        self.assertIsNone(expired['plan'],expired)
        self.assertEqual(expired['reason'],'ESCAPE_DEADLINE_EXPIRED')
        self.assertTrue(expired['blocked'])

    def test_stopping_place_remains_occupied_when_an_actor_arrives_after_rest(self):
        from intramural_mission.avoidance import collision_segments,stopped_collision
        brakes=np.array([[[0.,0.],[.2,0.]]]);times=np.array([[0.,.3]])
        actor=dict(center=[.2,-1.],velocity=[0.,1.],radius=.05,
            covariance=np.zeros((4,4)).tolist(),age=0.,moving=True)
        self.assertFalse(collision_segments(brakes[:,0],brakes[:,1],times[:,0],times[:,1],[actor],.29))
        self.assertTrue(stopped_collision(brakes,times,[actor],.29))
        actor['velocity']=[0.,-1.]
        self.assertFalse(stopped_collision(brakes,times,[actor],.29))

    def test_position_error_and_static_margin_share_one_budget(self):
        track=dict(center=[0.,0.],velocity=[0.,0.],extent=[.25,.25],radius=.36,
            covariance=np.diag([.02**2,.02**2,0.,0.]).tolist(),age=0.,moving=False,shared_margin=.02)
        self.assertFalse(collision([[.585,0.],[.585,0.]],[0.,0.],[track],.29+.02))
        self.assertTrue(collision([[.575,0.],[.575,0.]],[0.,0.],[track],.29+.02))
        track['covariance']=np.diag([.005**2,.005**2,0.,0.]).tolist()
        self.assertTrue(collision([[.555,0.],[.555,0.]],[0.,0.],[track],.29+.02))

    def test_stationary_scan_jitter_does_not_extrapolate_unobserved_acceleration(self):
        from intramural_mission.avoidance import obstacle_positions
        tracker=ObstacleTracker();rng=np.random.default_rng(17)
        moving=[]
        for i in range(100):
            center=np.array([2.,0.])+rng.normal(0.,.005,2)
            cloud=center+np.array([[x,y] for x in (-.25,0.,.25) for y in (-.25,0.,.25)])
            track=tracker.update(cloud,i*.1+.04,point_times=np.full(9,i*.1))[0]
            if i>10:moving.append(track['moving'])
        self.assertFalse(any(moving))
        track.update(moving=False,velocity=[.077,-.022],acceleration=[.519,0.])
        np.testing.assert_allclose(obstacle_positions(track,[0.,1.,2.]),
            np.asarray(track['center'])+np.arange(3)[:,None]*track['velocity'])

    def test_cruise_checks_future_braking_tails_when_the_ideal_line_is_clear(self):
        from intramural_mission.avoidance import DetourPlanner,preview
        c,grid,plan,_=self.case()
        c.update(response_models=[[2.,.5]],response_delay=2.,response_tau=.5,drive_acceleration_limit=2.)
        planner=DetourPlanner(c,grid)
        points=[[3.+x,y] for x in (-.1,0.,.1) for y in (-.1,0.,.1)]
        for i in range(7):
            tracks=planner.observations(points,i*.1)
        samples,timing=preview(plan,0.,1.)
        self.assertFalse(collision(samples,timing,tracks,.31))
        result=planner.solve(dict(plan=plan,progress=0.,pose=[0.,0.],velocity=[1.,0.],
            command=[1.,0.],acceleration=[0.,0.],points=points,stamp=.7))
        self.assertTrue(result['blocked'],result)
        self.assertNotEqual(result['reason'],'')

    def test_delayed_accelerating_observation_predicts_the_short_term_position(self):
        from intramural_mission.avoidance import obstacle_positions
        tracker=ObstacleTracker()
        for i in range(31):
            source=i*.05;moving=max(0.,source-1.)
            cloud=[[2.+moving*moving+x,y] for x in (-.1,0.,.1) for y in (-.1,0.,.1)]
            track=tracker.update(cloud,source+.1,sensor_position=[0.,0.],point_times=np.full(9,source))[0]
        actual=2.+.75**2+1.5*(.9-.75)
        linear=track['center'][0]+track['velocity'][0]*.3
        forecast=obstacle_positions(track,.3)[0]
        self.assertLess(abs(forecast-actual),.12)
        self.assertLess(abs(forecast-actual),.8*abs(linear-actual))
        self.assertGreater(track['acceleration'][0],.5)

    def test_accelerating_target_sweep_covers_a_curve_between_endpoints(self):
        track=dict(center=[-1.,0.],velocity=[2.,0.],acceleration=[0.,8.],radius=.01,
                   covariance=np.zeros((6,6)).tolist(),age=0.,moving=True)
        # Independent fine integration of da/dt=-a/.4 reaches the robot between samples.
        p=np.array(track['center']);v=np.array(track['velocity']);a=np.array(track['acceleration'])
        for _ in range(5000):
            p+=v*.0001;v+=a*.0001;a-=a*.0001/.4
        self.assertTrue(collision([p,p],[0.,1.],[track],.05))

    def test_speed_restoration_needs_two_distinct_clear_updates(self):
        from dataclasses import replace
        from intramural_mission.avoidance import DetourPlanner
        c,grid,fast,_=self.case()
        slow=replace(fast,speed=tuple(.5*v if .2<=s<=1.2 else v for s,v in zip(fast.s,fast.speed)),route_id='slowed')
        planner=DetourPlanner(c,grid)
        request=dict(plan=slow,restore_plan=fast,progress=.2,pose=[.2,0.],velocity=[.3,0.],
            command=[.3,0.],acceleration=[0.,0.],points=[],stamp=1.)
        self.assertIsNone(planner.solve(request)['plan'])
        self.assertIsNone(planner.solve(request)['plan'])
        result=planner.solve(dict(request,stamp=1.1))
        self.assertEqual(result['reason'],'RESTORE_READY')
        np.testing.assert_array_equal(result['plan'].speed[5:],fast.speed[5:])
        self.assertAlmostEqual(result['plan'].sample(.2)[2],.3)

    def test_entry_detour_has_a_continuous_return_joint_and_preserves_winding(self):
        import json
        from pathlib import Path
        from intramural_mission.route import segments
        from intramural_mission.bypass import winding
        c=json.loads((Path(__file__).resolve().parents[1]/'config/arena.json').read_text())
        route=segments(c)
        route[0][1][0]=[c['parking'][0]+.04,c['parking'][1]]
        plan=RoutePlan.build(route,c)
        bridge=np.asarray(plan.points[2:26]).copy()
        bridge[:,1]+=.08*np.sin(np.linspace(0.,np.pi,len(bridge)))
        updated,_=splice(plan,plan.s[2],bridge,plan.s[25],c,lambda p:np.ones(len(p)),.4)
        self.assertLess(max(np.linalg.norm(np.diff(updated.points,axis=0),axis=1)),.08)
        self.assertEqual(len(updated.gates),10)
        self.assertEqual(updated.points[-1],plan.points[-1])
        for key in ('lower_pole','upper_pole'):
            self.assertEqual(winding(updated.points,c[key]),winding(plan.points,c[key]))

    def test_repeated_retiming_uses_nominal_profile_and_recovers_an_intermediate_speed(self):
        from dataclasses import replace
        from intramural_mission.avoidance import DetourPlanner
        c,grid,_,_=self.case()
        c.update(max_speed=2.,max_acceleration=6.,lateral_acceleration=6.,max_tracking_error=.04,
                 drive_acceleration_limit=1.,response_models=[[.09,.05]],response_delay=.09,response_tau=.05)
        angle=np.linspace(0.,np.pi/2,61)
        nominal=RoutePlan.build([('RETURN',np.c_[.3*np.sin(angle),.3*(1-np.cos(angle))])],c)
        planner=DetourPlanner(c,grid)
        request=dict(plan=nominal,alternative_route=True,progress=nominal.s[10],pose=nominal.points[10],
            velocity=[0.,0.],command=[0.,0.],acceleration=[0.,0.],points=[],stamp=1.)
        result=planner.solve(request)
        self.assertEqual(result['reason'],'RETIME_READY')
        capped=result['plan']
        # A slightly faster current curve is unsafe here; repeated solves must select the same cap.
        faster=replace(capped,speed=tuple(v*1.2 for v in capped.speed))
        for i in range(3):
            result=planner.solve(dict(request,plan=faster,restore_plan=nominal,stamp=1.1+i*.1))
            self.assertEqual(result['reason'],'RETIME_READY')
            np.testing.assert_allclose(result['plan'].speed[11:],capped.speed[11:])
        slow=replace(capped,speed=tuple(v*.5 for v in capped.speed),route_id='slow')
        for i in range(3):
            result=planner.solve(dict(request,plan=slow,restore_plan=nominal,stamp=2.+i*.1))
            if result['plan'] is not None:break
        self.assertEqual(result['reason'],'RETIME_READY')
        self.assertEqual(result['scale'],.8)
        np.testing.assert_allclose(result['plan'].speed[11:],capped.speed[11:])

    def test_source_time_observation_predicts_current_position_without_sensor_lag(self):
        tracker=ObstacleTracker()
        for i in range(12):
            source=i*.1;reference=source+.1
            cloud=[[2.+source+x,y] for x in (-.03,0.,.03) for y in (-.03,0.,.03)]
            tracks=tracker.update(cloud,reference,sensor_position=[0.,0.],point_times=np.full(9,source))
            if i==1:self.assertGreater(tracks[0]['velocity'][0],.3)
        self.assertAlmostEqual(tracks[0]['center'][0],2.+reference,delta=.04)
        self.assertAlmostEqual(tracks[0]['velocity'][0],1.,delta=.15)

    def test_motion_start_is_not_hidden_by_a_confident_stationary_filter(self):
        tracker=ObstacleTracker()
        for i in range(24):
            x=2.+max(0,i-19)*.05
            cloud=[[x+dx,y] for dx in (-.1,0.,.1) for y in (-.1,0.,.1)]
            tracks=tracker.update(cloud,i*.05,sensor_position=[0.,0.])
        self.assertAlmostEqual(tracks[0]['center'][0],x,delta=.05)
        self.assertAlmostEqual(tracks[0]['velocity'][0],1.,delta=.25)

    def test_accelerating_motion_is_detected_before_the_braking_window_closes(self):
        tracker=ObstacleTracker()
        for i in range(25):
            moving=max(0.,(i-19)*.05)
            x=2.+moving*moving
            cloud=[[x+dx,y] for dx in (-.1,0.,.1) for y in (-.1,0.,.1)]
            tracks=tracker.update(cloud,i*.05,sensor_position=[0.,0.])
            if i==23:
                self.assertTrue(tracks[0]['moving'])
                self.assertGreater(tracks[0]['velocity'][0],.15)
        self.assertAlmostEqual(tracks[0]['center'][0],x,delta=.04)

    def test_partial_faces_stay_one_stationary_footprint(self):
        tracker=ObstacleTracker()
        full=[[2.+x,y] for x in (-.2,0.,.2) for y in (-.2,0.,.2)]
        for i in range(10):tracker.update(full,i*.1)
        for i in range(10,20):tracks=tracker.update([[1.8,y] for y in (-.2,0.,.2)],i*.1)
        self.assertEqual(len(tracks),1)
        self.assertAlmostEqual(tracks[0]['center'][0],2.,delta=.03)
        self.assertLess(np.linalg.norm(tracks[0]['velocity']),.03)
        self.assertGreater(tracks[0]['extent'][0],.2)

    def test_each_response_model_preserves_moving_boundary_and_real_braking_queue(self):
        from intramural_mission.response import predict_motion
        from intramural_mission.safety import stopping_horizon,response_horizon
        c,grid,plan,points=self.case()
        c.update(response_models=[[.09,.09],[.08,.03]],response_tau=.05,response_delay=.1,
                 drive_acceleration_limit=2.)
        self.assertGreater(stopping_horizon(c,1.),response_horizon(c))
        trajectories=predict_motion(plan,c,[.3,0.],[1.,0.],[1.,0.],[0.,0.],.3,horizon=.3)
        current=predict_motion(plan,c,[.3,0.],[1.,0.],[1.,0.],[0.,0.],.3,horizon=0.)
        self.assertEqual(len(trajectories),2)
        for (actual,timing,braking,brake_times,error),prefix in zip(trajectories,current):
            np.testing.assert_array_equal(actual[0],[.3,0.])
            np.testing.assert_array_equal(braking[:,0],actual)
            self.assertEqual(braking.shape[:2],brake_times.shape)
            self.assertGreater(braking[0,-1,0]-.3,.3)
            np.testing.assert_allclose(braking[0],prefix[2][0])
            np.testing.assert_array_equal(brake_times[0],prefix[3][0])

    def test_contour_keeps_free_square_corners_and_sweeps_between_samples(self):
        t=dict(center=[0.,0.],velocity=[0.,0.],radius=.354,extent=[.25,.25],
               covariance=np.zeros((4,4)).tolist(),age=0.,moving=False)
        self.assertFalse(collision([[.5,.5],[.6,.5]],[0.,.05],[t],.29))
        self.assertTrue(collision([[-1.,0.],[1.,0.]],[0.,.05],[t],.29))
        self.assertFalse(collision([[-1.,.6],[1.,.6]],[0.,.05],[t],.29))

    def test_voxel_compression_preserves_real_point_support(self):
        tracker = ObstacleTracker()
        self.assertEqual(len(tracker.update([[1.,0.]],1.,[3])),1)

    def case(self):
        import json
        from pathlib import Path
        from nav_msgs.msg import OccupancyGrid
        from intramural_mission.avoidance import DetourPlanner
        c = json.loads((Path(__file__).resolve().parents[1]/'config/arena.json').read_text())
        c.update(mission_variant='return', max_speed=1., max_acceleration=2., lateral_acceleration=1.,
                 terminal_deceleration=1., bounds=[-2.,6.,-3.,3.], pole_clearance_from_map=True,
                 lower_pole=[-10.,-10.], upper_pole=[10.,10.])
        grid = OccupancyGrid(); grid.header.frame_id='map'; grid.info.resolution=.05
        grid.info.width=160;grid.info.height=120;grid.info.origin.position.x=-2.;grid.info.origin.position.y=-3.
        grid.info.origin.orientation.w=1.;grid.data=[0]*19200
        plan = RoutePlan.build([('RETURN', [[x,0.] for x in np.linspace(0,4,81)])],c)
        points = [[1.2+x,y] for x in (-.1,0.,.1) for y in (-.1,0.,.1)]
        return c,grid,plan,points

    def test_worker_keeps_only_latest_pending_observation_and_closes(self):
        import time
        from intramural_mission.avoidance import PlanningWorker
        c,grid,plan,points=self.case();worker=PlanningWorker(c,grid);results=[]
        motion={key:np.zeros(2)for key in ('pose','velocity','command','acceleration')}
        try:
            for stamp in (1.,2.,3.):
                worker.submit(dict(plan=plan,progress=0.,points=points,stamp=stamp,**motion))
            for vector in motion.values():vector[:]=99.
            deadline=time.monotonic()+5.
            while len(results)<2 and time.monotonic()<deadline:
                result=worker.poll()
                if result is not None:results.append(result)
                time.sleep(.005)
            self.assertEqual([r['stamp']for r in results],[1.,3.])
            self.assertGreater(results[-1].get('continuation_until',0.),results[-1]['stamp'])
            self.assertFalse(results[-1].get('execution_failure',True))
        finally:worker.close()
        self.assertFalse(worker.process.is_alive())

    def test_worker_finishes_current_brake_failure_without_redundant_candidate_search(self):
        import time
        from intramural_mission.avoidance import PlanningWorker
        c,grid,plan,points=self.case();c['drive_acceleration_limit']=2.
        worker=PlanningWorker(c,grid)
        def receive():
            deadline=time.monotonic()+5.
            while time.monotonic()<deadline:
                result=worker.poll()
                if result is not None:return result
                time.sleep(.002)
            self.fail('Worker result did not arrive')
        try:
            worker.submit(dict(plan=plan,progress=0.,pose=[0.,0.],velocity=[0.,0.],
                command=[0.,0.],acceleration=[0.,0.],points=points,stamp=1.))
            receive()
            worker.submit(dict(plan=plan,progress=.3,pose=[.3,0.],velocity=[1.,0.],
                command=[1.,0.],acceleration=[0.,0.],points=np.asarray(points)-[.15,0.],stamp=1.1))
            alert=receive()
            self.assertEqual(alert['reason'],'BRAKING_ENVELOPE_CONFLICT')
            self.assertTrue(alert['blocked']);self.assertIsNone(alert['plan'])
            self.assertFalse(worker.busy)
            self.assertIsNone(worker.poll())
        finally:worker.close()

    def test_stopped_worker_with_large_queued_input_closes_without_feeder_hang(self):
        import os
        import signal
        import time
        from intramural_mission.avoidance import PlanningWorker
        c,grid,_,_=self.case();worker=PlanningWorker(c,grid)
        try:
            self.assertTrue(worker.ready.wait(5.))
            os.kill(worker.process.pid,signal.SIGSTOP)
            worker.submit(dict(payload=b'x'*2_000_000))
            began=time.monotonic();worker.close()
            self.assertLess(time.monotonic()-began,2.5)
            self.assertFalse(worker.process.is_alive())
        finally:
            if worker.process.is_alive():worker.process.kill();worker.process.join()

    def test_persistent_obstacle_produces_executable_detour(self):
        from intramural_mission.avoidance import DetourPlanner
        c,grid,plan,points=self.case();planner=DetourPlanner(c,grid)
        for i in range(6):
            planner.observations(points,i*.1)
        result = planner.solve(dict(plan=plan,progress=0.,pose=[0.,0.],velocity=[0.,0.],
            command=[0.,0.],acceleration=[0.,0.],points=points,stamp=.6))
        self.assertIsNotNone(result['plan'], result)
        self.assertGreater(max(abs(y) for x,y in result['plan'].points), .4)

    def test_far_conflict_can_continue_only_a_checked_prefix_with_a_stop_tail(self):
        from intramural_mission.avoidance import DetourPlanner
        c,grid,plan,_=self.case();c['drive_acceleration_limit']=2.
        points=[[1.8+x,y] for x in (-.1,0.,.1) for y in np.arange(-3.,3.01,.1)]
        planner=DetourPlanner(c,grid)
        for i in range(8):planner.observations(points,i*.1)
        request=dict(plan=plan,progress=0.,pose=[0.,0.],velocity=[1.,0.],
            command=[1.,0.],acceleration=[0.,0.],points=points,stamp=.8)
        result=planner.solve(request)
        self.assertTrue(result['blocked']);self.assertIsNone(result['plan'])
        self.assertGreater(result['continuation_until'],request['stamp'])
        result=planner.solve(dict(request,pose=[1.4,0.],progress=1.4,stamp=.9))
        self.assertTrue(result['imminent']);self.assertNotIn('continuation_until',result)

    def test_raw_blocked_path_triggers_replanning_beyond_a_slow_two_second_preview(self):
        from dataclasses import replace
        from intramural_mission.avoidance import DetourPlanner
        c,grid,plan,_=self.case();plan=replace(plan,speed=tuple(.05 for _ in plan.speed))
        points=[[2.5+x,y] for x in (-.1,0.,.1) for y in np.arange(-3.,3.01,.1)]
        planner=DetourPlanner(c,grid)
        for i in range(8):planner.observations(points,i*.1)
        request=dict(plan=plan,progress=1.85,pose=[1.85,0.],velocity=[0.,0.],
            command=[0.,0.],acceleration=[0.,0.],points=points,stamp=.8)
        self.assertFalse(planner.solve(request)['blocked'])
        self.assertTrue(planner.solve(dict(request,stamp=.9,path_blocked=True))['blocked'])

    def test_clear_stopped_pose_can_leave_a_cell_with_a_tighter_center(self):
        import json
        from pathlib import Path
        from intramural_mission.avoidance import DetourPlanner
        c,grid,_,points=self.case()
        c.update(json.loads((Path(__file__).resolve().parents[1]/'config/navigation-profiles.json').read_text())['candidate'])
        plan=RoutePlan.build([('RETURN',[[x,0.] for x in np.linspace(0.,4.,81)])],c)
        planner=DetourPlanner(c,grid)
        for i in range(6):planner.observations(points,i*.1)
        for i in range(6):
            result=planner.solve(dict(plan=plan,progress=.71,pose=[.71,0.],velocity=[0.,0.],
                command=[0.,0.],acceleration=[0.,0.],points=points,stamp=.6+i*.05))
            if result['plan'] is not None:break
        self.assertIsNotNone(result['plan'],result)

    def test_motion_and_missing_observation_keep_predicted_occupancy(self):
        tracker = ObstacleTracker()
        for i in range(12):
            cloud = np.array([[i*.1+x, y] for x in (-.03, 0, .03) for y in (-.03, 0, .03)])
            tracks = tracker.update(cloud, i*.1)
        self.assertEqual(len(tracks), 1)
        self.assertAlmostEqual(tracks[0]['velocity'][0], 1., delta=.15)
        self.assertTrue(tracker.update([], 1.2))
        self.assertEqual(tracker.update([], 1.2), tracker.snapshot(1.2))
        self.assertEqual(tracker.update([], 2.1), [])

    def test_crossing_checks_arrival_time_and_intermediate_sweep(self):
        track = dict(center=[.5, -.5], velocity=[0., 1.], radius=.06,
                     covariance=np.zeros((4, 4)).tolist(), age=0.)
        self.assertTrue(collision([[0, 0], [1, 0]], [0., 1.], [track], .1))
        track['velocity'] = [0., -1.]
        self.assertFalse(collision([[0, 0], [1, 0]], [0., 1.], [track], .1))

    def test_invalid_or_old_scan_does_not_advance_tracking_clock(self):
        tracker = ObstacleTracker()
        cloud = np.array([[0., 0.], [.02, 0], [0, .02]])
        tracker.update(cloud, 1.)
        with self.assertRaises(ValueError):
            tracker.update([[float('nan'), 0]], 1.1)
        self.assertEqual(tracker.stamp, 1.)
        self.assertEqual(tracker.update(cloud, .9), tracker.snapshot(1.))

    def test_splice_keeps_completed_gates_and_nonzero_boundary_speed(self):
        limits = dict(max_speed=1., max_acceleration=2., lateral_acceleration=1.,
                      braking_acceleration=.5, terminal_deceleration=1., path_spacing=.05)
        route = [('ENTER', [[x, 0.] for x in np.linspace(0, 1, 21)]),
                 ('RETURN', [[x, 0.] for x in np.linspace(1, 3, 41)])]
        plan = RoutePlan.build(route, limits)
        bridge = [[x, .15*np.sin((x-1.2)*np.pi/.8)**2] for x in np.linspace(1.2, 2., 25)]
        updated, start_s = splice(plan, 1.2, bridge, 2., limits, lambda p: np.ones(len(p)), .5)
        self.assertEqual(updated.gates[0], plan.gates[0])
        self.assertEqual(updated.points[-1], plan.points[-1])
        self.assertAlmostEqual(updated.sample(start_s)[2], .5, delta=.08)
        self.assertGreater(start_s, updated.s[updated.gates[0][1]])


if __name__ == '__main__':
    unittest.main()
