#!/usr/bin/env python3
"""Detect isolated vertical pole candidates and prepare a checked figure-eight map copy."""
import argparse
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path
import shutil

import numpy as np
from PIL import Image
from scipy.ndimage import distance_transform_edt, label
import yaml
from intramural_mission.proximity import lidar_to_body
from intramural_mission.route import segments
from intramural_mission.planning import compile_route
from types import SimpleNamespace as N
from dataclasses import asdict
from response_fit import model_overrides


def current_parking(source):
    import time
    import rclpy
    from tf2_ros import Buffer, TransformListener
    from nav_msgs.msg import OccupancyGrid
    from rclpy.qos import QoSProfile,DurabilityPolicy
    metadata=yaml.safe_load((source/'map.yaml').read_text())
    pixels=np.asarray(Image.open(source/metadata['image']))
    expected=np.where(pixels[::-1]==254,0,np.where(pixels[::-1]==205,-1,100)).ravel()
    rclpy.init()
    node = rclpy.create_node('field_start_position')
    buffer = Buffer(node=node)
    listener = TransformListener(buffer,node)
    matching=False
    def receive_map(message):
        nonlocal matching
        q=message.info.origin.orientation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        matching=(message.header.frame_id=='map' and message.info.width==pixels.shape[1]
            and message.info.height==pixels.shape[0] and math.isclose(message.info.resolution,metadata['resolution'],rel_tol=1e-6)
            and abs(math.atan2(math.sin(yaw-metadata['origin'][2]),math.cos(yaw-metadata['origin'][2])))<1e-8
            and np.allclose([message.info.origin.position.x,message.info.origin.position.y],metadata['origin'][:2],atol=1e-8,rtol=0)
            and np.array_equal(message.data,expected))
    namespace=json.loads((source/'arena.json').read_text())['namespace'].strip('/')
    node.create_subscription(OccupancyGrid,'/'+namespace+'/map',receive_map,
        QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    deadline = time.monotonic()+8
    try:
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.05)
            try:
                pose = buffer.lookup_transform('map','base_footprint',rclpy.time.Time())
            except Exception:
                continue
            stamp = pose.header.stamp.sec+pose.header.stamp.nanosec/1e9
            age = node.get_clock().now().nanoseconds/1e9-stamp
            xy = [pose.transform.translation.x,pose.transform.translation.y]
            if matching and abs(age)<.25 and all(math.isfinite(v) for v in xy):
                return xy,dict(source='localized_current_vehicle',frame='map',stamp=stamp,age=age)
        raise RuntimeError('No fresh pose in the selected map; start matching localized standby before --parking-here')
    finally:
        node.destroy_node()
        rclpy.shutdown()


def detect_poles(points, plane):
    """Geometric candidates only: a cluttered or incomplete pair must not auto-pass."""
    height = points[:, 2] - (points[:, :2] @ np.asarray(plane[:2]) + plane[2])
    selected = (height > .12) & (height < 1.4)
    cloud, height = points[selected], height[selected]
    if len(cloud) < 30:
        return []
    origin = np.floor(cloud[:, :2].min(axis=0))
    cells = np.floor((cloud[:, :2]-origin)/.05).astype(int)
    shape = cells.max(axis=0)+1
    if max(shape) > 2000:
        raise ValueError("Point cloud extent too large for field detector")
    grid = np.zeros(shape[::-1], dtype=bool)
    grid[cells[:, 1], cells[:, 0]] = True
    regions, count = label(grid, np.ones((3, 3)))
    ids = regions[cells[:, 1], cells[:, 0]]
    candidates = []
    for i in range(1, count+1):
        p, h = cloud[ids == i], height[ids == i]
        if len(p) < 30:
            continue
        span = np.ptp(p[:, :2], axis=0)
        levels = np.unique(np.floor(h/.15).astype(int))
        if max(span) > .35 or np.ptp(h) < .5 or len(levels) < 4:
            continue
        # A broad base is allowed; the shaft above the base must remain thin.
        shaft = p[h > .35]
        if not len(shaft) or max(np.ptp(shaft[:, :2], axis=0)) > .16:
            continue
        center = np.median(p[:, :2], axis=0)
        level_centers = [np.median(p[np.floor(h/.15).astype(int) == k, :2], axis=0) for k in levels]
        if max(np.linalg.norm(v-center) for v in level_centers) > .12:
            continue
        candidates.append(dict(center=center.tolist(), points=len(p), xy_span_m=span.tolist(),
            height_span_m=float(np.ptp(h)), radius_bound_m=float(np.linalg.norm(span)/2)))
    return candidates


def route_preview(plan, config, grid, metadata, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.collections import LineCollection
    from matplotlib.patches import Circle
    p=np.asarray(plan.points)
    fig=plt.figure(figsize=(11,7));layout=fig.add_gridspec(2,2)
    ax=fig.add_subplot(layout[:,0]);speed=fig.add_subplot(layout[0,1]);curvature=fig.add_subplot(layout[1,1])
    x,y,_=metadata['origin'];res=metadata['resolution']
    ax.imshow(grid,cmap='gray',vmin=0,vmax=254,extent=[x,x+grid.shape[1]*res,y,y+grid.shape[0]*res],origin='upper')
    collection=LineCollection(np.stack((p[:-1],p[1:]),axis=1),cmap='viridis',linewidths=2.5)
    collection.set_array(np.asarray(plan.speed[:-1]));collection.set_clim(0,config['max_speed']);ax.add_collection(collection)
    fig.colorbar(collection,ax=ax,label='Reference speed (1:1 command scale)')
    for key in ('lower_pole','upper_pole'):
        ax.add_patch(Circle(config[key],config['pole_radius'],color='orange',alpha=.8))
    ax.plot(*config['parking'],'rx',markersize=10,label='start / return');ax.legend()
    ax.set(xlim=(p[:,0].min()-.7,p[:,0].max()+.7),ylim=(p[:,1].min()-.7,p[:,1].max()+.7),
           xlabel='map x (m)',ylabel='map y (m)',title='Ordered full route',aspect='equal')
    speed.plot(plan.s,plan.speed);speed.set(xlabel='Arc length (m)',ylabel='Reference speed',title='Continuous speed profile')
    curvature.plot(plan.s,np.abs(plan.curvature));curvature.set(xlabel='Arc length (m)',ylabel='|curvature| (1/m)',title='Route curvature')
    for _,i in plan.gates[:-1]:
        speed.axvline(plan.s[i],color='gray',alpha=.2);curvature.axvline(plan.s[i],color='gray',alpha=.2)
    speed.grid(alpha=.2);curvature.grid(alpha=.2)
    fig.tight_layout();fig.savefig(output,dpi=150);plt.close(fig)


def check_route(config, grid, metadata, plan_path=None):
    if config.get('navigation_engine')=='trajectory':
        yaw=metadata['origin'][2];res=float(metadata['resolution'])
        msg=N(header=N(frame_id='map'),info=N(resolution=res,width=grid.shape[1],height=grid.shape[0],
            origin=N(position=N(x=metadata['origin'][0],y=metadata['origin'][1]),
                orientation=N(w=math.cos(yaw/2),x=0.,y=0.,z=math.sin(yaw/2)))),
            data=np.where(grid[::-1]==254,0,np.where(grid[::-1]==205,-1,100)).ravel().tolist())
        try:
            plan,route,report=compile_route(config,msg)
            if plan_path:
                plan_path.write_text(json.dumps(asdict(plan),indent=2)+'\n')
                route_preview(plan,config,grid,metadata,plan_path.with_suffix('.png'))
            needed=config['robot_radius']+res/math.sqrt(2)+config['path_spacing']/2+config.get('minimum_clearance',.02)
            return dict(passed=True,minimum_obstacle_distance_m=report['minimum_clearance']+needed,
                required_obstacle_distance_m=needed,blocked_samples=0,examples=[],route_id=plan.route_id,compiler=report)
        except ValueError as exc:
            return dict(passed=False,minimum_obstacle_distance_m=0.,blocked_samples=1,examples=[str(exc)])
    free = np.asarray(grid)[::-1] == 254
    res = float(metadata['resolution'])
    distances = distance_transform_edt(np.pad(free, 1))[1:-1, 1:-1]*res
    # Include pixel boundary uncertainty and half the path sampling interval.
    needed = config['robot_radius'] + res/math.sqrt(2) + config['path_spacing']/2
    blocked = []
    minimum = math.inf
    for phase, points in segments(config):
        for x, y in points:
            col = math.floor((x-metadata['origin'][0])/res)
            row = math.floor((y-metadata['origin'][1])/res)
            clearance = distances[row, col] if 0 <= row < free.shape[0] and 0 <= col < free.shape[1] else 0.
            minimum = min(minimum, clearance)
            if clearance <= needed:
                blocked.append(dict(phase=phase, point=[x,y], clearance_m=float(clearance)))
    return dict(passed=not blocked, minimum_obstacle_distance_m=float(minimum),
        required_obstacle_distance_m=needed, blocked_samples=len(blocked), examples=blocked[:5])


def main():
    root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map-config', required=True)
    parser.add_argument('--output')
    parking = parser.add_mutually_exclusive_group()
    parking.add_argument('--parking', nargs=2, type=float, help='Starting/return position in map coordinates; default mapping origin')
    parking.add_argument('--parking-here',action='store_true',help='Capture current localized vehicle position as start and return position')
    parser.add_argument('--profile',choices=['safe','candidate','gold-reference'],default='candidate')
    parser.add_argument('--response-model',type=Path)
    bypass=parser.add_mutually_exclusive_group()
    bypass.add_argument('--allow-bypass',dest='bypass',action='store_const',const=True,default=None)
    bypass.add_argument('--no-bypass',dest='bypass',action='store_const',const=False)
    args = parser.parse_args()
    profiles=json.loads((Path(__file__).resolve().parents[2]/'config/navigation-profiles.json').read_text())
    profile=profiles[args.profile].copy()
    source = Path(args.map_config).resolve()
    if args.response_model:
        args.response_model = args.response_model if args.response_model.is_absolute() else root/args.response_model
        profile.update(model_overrides(args.response_model))
    parking_source = dict(source='explicit_coordinates' if args.parking else 'mapping_origin',frame='map')
    if args.parking_here:
        args.parking,parking_source = current_parking(source)
    args.parking = args.parking or [0.,0.]
    output=Path(args.output) if args.output else root/'artifacts/navigation'/datetime.now().strftime('%Y%m%d-%H%M%S-%f')
    out=(output if output.is_absolute()else root/output).resolve()
    if out.exists() or not any(out.is_relative_to(root/k) and out != root/k for k in ('cache','artifacts')):
        parser.error('Choose a new directory under project cache/ or artifacts/')
    if not all(math.isfinite(v) for v in args.parking):
        parser.error('Parking coordinates must be finite')
    out.mkdir(parents=True)
    raw = (source/'prior.pcd').read_bytes()
    header, binary = raw.split(b'DATA binary\n',1)
    lines = header.decode().splitlines()
    if not all(item in lines for item in ('FIELDS x y z','SIZE 4 4 4','TYPE F F F','COUNT 1 1 1')) or len(binary)%12:
        raise ValueError('Expected project-exported binary XYZ map')
    config = json.loads((source/'arena.json').read_text())
    cloud = lidar_to_body(np.frombuffer(binary,dtype='<f4').reshape(-1,3), config['lidar_pose'])
    if not np.isfinite(cloud).all():
        raise ValueError('Nonfinite point cloud')
    plane = json.loads((source/'occupancy.json').read_text())['floor_plane']['z_equals_ax_by_c']
    candidates = detect_poles(cloud,plane)
    report = dict(source_map=str(source), source_pcd_sha256=hashlib.sha256(raw).hexdigest(),
        candidates=candidates, detector_scope='Isolated thin vertical objects, observed height span >=0.5m; geometric identity, no semantic classifier',
        parking=args.parking,parking_source=parking_source, passed=False, physical_motion_commanded=False)
    if len(candidates) != 2:
        report['error'] = f'Expected exactly two unambiguous candidates, found {len(candidates)}; inspect candidates instead of auto-selecting'
    else:
        # The first pole is the one closest to the confirmed starting parking position.
        poles = sorted(candidates, key=lambda p:math.dist(p['center'],args.parking))
        lower, upper = [p['center'] for p in poles]
        metadata = yaml.safe_load((source/'map.yaml').read_text())
        grid = np.asarray(Image.open(source/metadata['image']))
        if grid.ndim != 2 or not set(np.unique(grid)) <= {0,205,254}:
            raise ValueError('Unexpected occupancy encoding')
        config.update(lower_pole=lower, upper_pole=upper, parking=args.parking,mission_variant='figure8',
            loop_radius=math.dist(lower,upper)/2, pole_radius=max(config['pole_radius'],*(p['radius_bound_m'] for p in poles)),
            continuous_corners=True,
            bypass_speed=.2, allow_bypass=config.get('allow_bypass',False) if args.bypass is None else args.bypass, minimum_clearance=.02,
            raw_zero_intensity_min_range_m=.15,
            raw_collision_follow_path=True,
            raw_collision_min_points=3,
            # Right lower chassis structure confirmed on the SRM vehicle, 2026-10-29.
            raw_self_filter_boxes=config.get('raw_self_filter_boxes',[[[.14,-.36,.20],[.19,-.24,.28]]]),
            continuous_segments=True,
            pole_clearance_from_map=True,
            bounds=[metadata['origin'][0],metadata['origin'][0]+grid.shape[1]*metadata['resolution'],
                    metadata['origin'][1],metadata['origin'][1]+grid.shape[0]*metadata['resolution']],
            assumptions='Observed map and geometric pole detection; identity vehicle scale is an operator-approved initial estimate')
        attempts=[]
        config.pop('optimized_route',None)
        if config['loop_radius'] <= config['robot_radius']+config['pole_radius']+.05:
            report['error']='Pole spacing cannot accommodate the complete vehicle envelope'
        else:
            for angle in ((0.,) if profile.get('optimize_route') else (0.,math.pi)):
                config.update(profile)
                config['navigation_profile']=args.profile
                config['entry_angle']=angle
                check=check_route(config,grid,metadata)
                attempts.append(dict(entry_angle=angle,**check))
            choices=[v for v in attempts if v['passed']]
            if not choices:
                report['error']='Neither complete figure-eight route is clear in observed space'
            else:
                chosen=(min(choices,key=lambda v:v['compiler'].get('execution_prediction',{}).get('duration',v['compiler']['expected_duration']))
                        if profile.get('navigation_engine')=='trajectory' else max(choices,key=lambda v:v['minimum_obstacle_distance_m']))
                config['entry_angle']=chosen['entry_angle']
                if 'optimized_route' in chosen.get('compiler',{}):
                    config['optimized_route']=chosen['compiler']['optimized_route']
                    config['entry_angle']=config['optimized_route']['entry_angle']
                final = check_route(config,grid,metadata,out/'route-plan.json')
                if not final['passed']:
                    raise ValueError('Selected route could not be reproduced from its frozen parameters')
                (out/'route-analysis.json').write_text(json.dumps(dict(search=chosen,compiled=final,parking_source=parking_source),indent=2)+'\n')
                for name in ('prior.pcd','map.pgm','map.yaml','occupancy.json'):
                    shutil.copy2(source/name,out/name)
                (out/'arena.json').write_text(json.dumps(config,indent=2)+'\n')
                params=yaml.safe_load((source/'nav.yaml').read_text())
                params['small_gicp_relocalization']['ros__parameters']['prior_pcd_file']=str(out/'prior.pcd')
                params['map_server']['ros__parameters']['yaml_filename']=str(out/'map.yaml')
                speed=config['max_speed']
                params['controller_server']['ros__parameters']['FollowPath'].update(v_linear_min=-speed,v_linear_max=speed,
                    lookahead_dist=profile.get('lookahead_dist',.4),use_velocity_scaled_lookahead_dist=False,min_lookahead_dist=profile.get('lookahead_dist',.4),max_lookahead_dist=profile.get('lookahead_dist',.4),
                    use_interpolation=True,translation_kp=profile.get('translation_kp',3.4),min_approach_linear_velocity=.05,approach_velocity_scaling_dist=.2,reduction_ratio_at_high_curvature=.9)
                params['controller_server']['ros__parameters']['general_goal_checker']['xy_goal_tolerance']=min(config['checkpoint_tolerance'],config['parking_tolerance'])
                controller=params['controller_server']['ros__parameters']
                checkers=controller.setdefault('goal_checker_plugins',['general_goal_checker'])
                if 'continuous_goal_checker' not in checkers:
                    checkers.append('continuous_goal_checker')
                controller['continuous_goal_checker']=dict(plugin='nav2_controller::SimpleGoalChecker',stateful=False,
                    xy_goal_tolerance=.001,yaw_goal_tolerance=6.28)
                smooth=params['velocity_smoother']['ros__parameters']
                smooth.update(max_velocity=[speed,speed,0.],min_velocity=[-speed,-speed,0.],
                    max_accel=[config['max_acceleration']]*2+[0.],max_decel=[-config['max_acceleration']]*2+[0.])
                (out/'nav.yaml').write_text(yaml.safe_dump(params,sort_keys=False))
                manifest=json.loads((source/'manifest.json').read_text())
                manifest['navigation_profile']=args.profile
                manifest['parking_source']=parking_source
                manifest['route_plan_id']=final.get('route_id')
                mission_source=root/'nav_ws/src/intramural_mission/intramural_mission'
                manifest['route_sources_sha256']={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in mission_source.glob('*.py')}
                if args.response_model:
                    manifest['response_model_sha256']=hashlib.sha256(args.response_model.read_bytes()).hexdigest()
                manifest['profile_sha256']=hashlib.sha256(json.dumps(profile,sort_keys=True).encode()).hexdigest()
                manifest['physical_validation']='pending' if args.profile!='gold-reference' else 'reference_configuration_only'
                manifest.update(field_geometry_source='automatic geometric pole extraction from source point cloud',
                    field_geometry_verified=True, motion_permitted=False,start_permitted=False,
                    obstacle_policy='predictive_detour' if config['allow_bypass'] else 'stop_and_wait',
                    source_map=str(source),source_pcd_sha256=report['source_pcd_sha256'],
                    field_geometry_method='two_isolated_vertical_objects_plus_full_route_grid_check',
                    purpose='Field figure-eight candidate; explicit drive and mission start still required')
                manifest['files_sha256']={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in out.iterdir() if p.is_file()}
                (out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
                report.update(passed=True,lower_pole=lower,upper_pole=upper,route_check=final,route_search=chosen,navigation_config=str(out))
        report['route_attempts']=attempts
    (out/'field-detection.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2),flush=True)
    raise SystemExit(0 if report['passed'] else 1)


if __name__=='__main__':
    main()
