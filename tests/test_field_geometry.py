"""Run offline field detection/configuration against synthetic maps; never open devices."""
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import threading
import time

import numpy as np
from PIL import Image
import yaml
from intramural_mission.route import segments
from intramural_mission.bypass import winding


class FieldGeometryTest(unittest.TestCase):
    def run_field(self, ambiguous=False, blocked=False, short=False, parking=None, response=False, wrong_map=False,bypass=None):
        root=Path(__file__).resolve().parents[1]
        with tempfile.TemporaryDirectory(dir=root/'cache') as temp:
            source=Path(temp)/'map';source.mkdir();out=Path(temp)/'configured'
            poles=[[0.,2.],[1.,3.5]] + ([[3.,1.]] if ambiguous else [])
            points=[]
            for cx,cy in poles:
                for h in np.linspace(.15,.3 if short else 1.,10):
                    for a in np.linspace(0,2*math.pi,12,endpoint=False):
                        points.append([cx+.07*math.cos(a),cy+.07*math.sin(a),h])
            raw=np.asarray(points,dtype='<f4').tobytes()
            (source/'prior.pcd').write_bytes(b'FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nDATA binary\n'+raw)
            (source/'occupancy.json').write_text(json.dumps({'floor_plane':{'z_equals_ax_by_c':[0.,0.,0.]}}))
            config=json.loads((root/'config/arena.json').read_text());config['lidar_pose']=[0.]*6
            if bypass is not None:config['allow_bypass']=not bypass
            config['optimized_route']=dict(shape=[20.,20.,.5],entry_angle=0.,transition_scale=1.25)
            (source/'arena.json').write_text(json.dumps(config))
            Image.fromarray(np.full((300,300),205 if blocked else 254,dtype=np.uint8)).save(source/'map.pgm')
            (source/'map.yaml').write_text(yaml.safe_dump(dict(image='map.pgm',resolution=.05,origin=[-5.,-5.,0.])))
            (source/'manifest.json').write_text(json.dumps(dict(motion_permitted=False,start_permitted=False)))
            (source/'nav.yaml').write_text(yaml.safe_dump({
                'small_gicp_relocalization':{'ros__parameters':{}},'map_server':{'ros__parameters':{}},
                'controller_server':{'ros__parameters':{'FollowPath':{},'general_goal_checker':{}}},'velocity_smoother':{'ros__parameters':{}}}))
            command=[sys.executable,str(root/'scripts/internal/field_geometry.py'),
                '--map-config',str(source),'--output',str(out),'--profile','safe']
            if parking is not None:command += ['--parking-here']
            if bypass is not None:command += ['--allow-bypass' if bypass else '--no-bypass']
            if response:
                model=Path(temp)/'response.json'
                model.write_text(json.dumps(dict(eligible_for_model=True,clock='source',response_delay=.09,response_tau=.08)))
                command += ['--response-model',str(model)]
            result=subprocess.run(command,capture_output=True,text=True,timeout=25)
            if wrong_map:
                self.assertNotEqual(result.returncode,0)
                self.assertIn('selected map',result.stderr)
                self.assertFalse(out.exists())
                return dict(passed=False)
            report=json.loads((out/'field-detection.json').read_text())
            self.assertEqual(result.returncode,0 if report['passed'] else 1,result.stderr)
            self.assertEqual((source/'prior.pcd').read_bytes().split(b'DATA binary\n',1)[1],raw)
            if report['passed']:
                actual=json.loads((out/'arena.json').read_text())
                self.assertEqual(actual['allow_bypass'],False if bypass is None else bypass)
                self.assertEqual(json.loads((out/'manifest.json').read_text())['obstacle_policy'],
                    'predictive_detour' if actual['allow_bypass'] else 'stop_and_wait')
                frozen=json.loads((out/'route-plan.json').read_text())
                route=[];previous=0
                for name,index in frozen['gates']:
                    route.append((name,frozen['points'][previous:index+1]));previous=index
                points=frozen['points']
                self.assertEqual([winding(points,actual[k]) for k in ('lower_pole','upper_pole')],[1,-1])
                self.assertEqual(route[0][1][0],actual['parking'])
                length=math.dist(route[0][1][0],route[0][1][-1])
                stem=[p for p in route[0][1] if math.dist(p,actual['parking'])<.1*length]
                self.assertTrue(stem)
                for p in stem:
                    self.assertLess(min(math.dist(p,q)for q in route[0][1]),1e-8)
                    self.assertLess(min(math.dist(p,q)for q in route[-1][1]),1e-8)
                self.assertEqual(len(route),10)
                for (_,a),(_,b) in zip(route,route[1:]):self.assertLess(math.dist(a[-1],b[0]),1e-8)
                self.assertTrue(np.allclose(actual['lower_pole'],poles[0],atol=.01))
                self.assertTrue(np.allclose(actual['upper_pole'],poles[1],atol=.01))
                manifest=json.loads((out/'manifest.json').read_text())
                self.assertEqual(manifest['route_plan_id'],frozen['route_id'])
                if response:
                    self.assertEqual(actual['response_delay'],.09)
                    self.assertEqual(actual['response_tau'],.08)
                    self.assertIn('response_model_sha256',manifest)
                if parking is not None:
                    np.testing.assert_allclose(actual['parking'],parking)
                    self.assertEqual(manifest['parking_source']['source'],'localized_current_vehicle')
                replay=subprocess.run([sys.executable,str(root/'scripts/validation/verify-map-trajectory.py'),
                    '--map-dir',str(out),'--output',str(out/'reload')],capture_output=True,text=True,timeout=8)
                self.assertEqual(replay.returncode,0,replay.stderr)
                self.assertEqual(json.loads((out/'reload/results.json').read_text())['results'][0]['route_id'],frozen['route_id'])
                self.assertTrue(manifest['field_geometry_verified'])
                self.assertFalse(manifest['motion_permitted'])
                self.assertFalse(manifest['start_permitted'])
                self.assertEqual((out/'prior.pcd').read_bytes(),(source/'prior.pcd').read_bytes())
                self.assertEqual(actual['max_speed'],.65)
                self.assertEqual(actual['max_acceleration'],1.2)
                self.assertEqual(actual['braking_acceleration'],.2)
                self.assertTrue(actual['continuous_corners'])
                self.assertTrue(actual['smooth_transitions'])
                self.assertEqual(actual['lateral_acceleration'],.65)
                self.assertTrue(actual['pole_clearance_from_map'])
                self.assertEqual(actual['minimum_clearance'],.02)
                self.assertTrue(actual['raw_collision_follow_path'])
                self.assertEqual(actual['raw_collision_min_points'],3)
                from intramural_mission.proximity import outside_self
                echoes=np.array([[.16,-.30,.24],[.328,-.011,.422],[.405,.033,.317],
                                 [.60,0.,.30],[.36,.25,.30],[.36,0.,.80],[-.35,0.,.30]])
                np.testing.assert_array_equal(outside_self(echoes,actual['raw_self_filter_boxes']),
                                              [False,False,False,True,True,True,True])
                params=yaml.safe_load((out/'nav.yaml').read_text())
                follow=params['controller_server']['ros__parameters']['FollowPath']
                self.assertEqual(follow['v_linear_max'],.65)
                self.assertEqual(follow['translation_kp'],3.4)
                continuous=params['controller_server']['ros__parameters']['continuous_goal_checker']
                self.assertEqual(continuous['xy_goal_tolerance'],.001)
                self.assertEqual(params['velocity_smoother']['ros__parameters']['max_velocity'],[.65,.65,0.])
                self.assertEqual(params['velocity_smoother']['ros__parameters']['max_accel'],[1.2,1.2,0.])
                self.assertEqual(params['velocity_smoother']['ros__parameters']['max_decel'],[-1.2,-1.2,0.])
                self.assertEqual(follow['lookahead_dist'],.4)
                self.assertFalse(follow['use_velocity_scaled_lookahead_dist'])
                self.assertTrue(follow['use_interpolation'])
                self.assertLessEqual(params['controller_server']['ros__parameters']['general_goal_checker']['xy_goal_tolerance'],actual['parking_tolerance'])
            else:
                self.assertFalse((out/'manifest.json').exists())
            return report

    def test_rotated_pair_complete_ordered_route_and_immutable_source(self):
        self.assertTrue(self.run_field()['passed'])

    def test_explicit_bypass_switch_rebuilds_the_configuration(self):
        self.assertTrue(self.run_field(bypass=True)['passed'])
        self.assertTrue(self.run_field(bypass=False)['passed'])

    def test_ambiguous_three_poles_refuse_automatic_selection(self):
        self.assertFalse(self.run_field(ambiguous=True)['passed'])

    def test_broad_upper_object_is_not_a_third_thin_pole(self):
        sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts/internal'))
        from field_geometry import detect_poles
        points=[]
        for cx,cy in ((0.,2.),(1.,3.5),(3.,1.)):
            for h in np.linspace(.15,1.35,16):
                radius=(.04 if h<.7 else .14) if cx==3. else (.14 if h<.35 else .04)
                for a in np.linspace(0,2*math.pi,20,endpoint=False):
                    points.append([cx+radius*math.cos(a),cy+radius*math.sin(a),h])
        candidates=detect_poles(np.asarray(points),[0.,0.,0.])
        self.assertEqual(len(candidates),2)
        np.testing.assert_allclose(sorted(p['center'] for p in candidates),[[0.,2.],[1.,3.5]],atol=.01)

    def test_unknown_route_is_blocked(self):
        self.assertFalse(self.run_field(blocked=True)['passed'])

    def test_short_clutter_is_not_a_pole(self):
        self.assertFalse(self.run_field(short=True)['passed'])

    def test_current_localized_start_is_frozen_with_the_complete_route(self):
        import rclpy
        from rclpy.context import Context
        from geometry_msgs.msg import TransformStamped
        from tf2_ros import TransformBroadcaster
        from nav_msgs.msg import OccupancyGrid
        from rclpy.qos import QoSProfile,DurabilityPolicy
        context=Context();rclpy.init(context=context)
        node=rclpy.create_node('field_parking_fixture',context=context)
        broadcaster=TransformBroadcaster(node)
        maps=node.create_publisher(OccupancyGrid,'/intramural_robot/map',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        grid=OccupancyGrid();grid.header.frame_id='map';grid.info.width=grid.info.height=300;grid.info.resolution=.05
        grid.info.origin.position.x=grid.info.origin.position.y=-5.;grid.info.origin.orientation.w=1.;grid.data=[0]*90000
        maps.publish(grid)
        stop=threading.Event()
        def publish():
            while not stop.is_set():
                msg=TransformStamped();msg.header.frame_id='map';msg.child_frame_id='base_footprint'
                msg.header.stamp=node.get_clock().now().to_msg()
                msg.transform.translation.x=-.6;msg.transform.translation.y=.2;msg.transform.rotation.w=1.
                broadcaster.sendTransform(msg);time.sleep(.025)
        worker=threading.Thread(target=publish);worker.start()
        try:
            self.assertTrue(self.run_field(parking=[-.6,.2],response=True)['passed'])
            grid.info.origin.position.x=-4.5;maps.publish(grid)
            self.assertFalse(self.run_field(parking=[-.6,.2],wrong_map=True)['passed'])
        finally:
            stop.set();worker.join();node.destroy_node();context.shutdown()


if __name__=='__main__':unittest.main()
