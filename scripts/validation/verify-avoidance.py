#!/usr/bin/env python3
"""Production avoidance, real-recording shadow replay and force-driven Gazebo acceptance."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import runpy
import subprocess
import sys
import time
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'scripts/internal'))
from intramural_mission.avoidance import DetourPlanner, collision
from intramural_mission.trajectory import RoutePlan, OmniTracker
from intramural_mission.response import Plant
from intramural_base.transport import CommandPolicy
from intramural_mission.proximity import lidar_to_body


def save(path, data):
    path.write_text(json.dumps(data, indent=2, allow_nan=False)+'\n')


def load_map(folder):
    return runpy.run_path(str(ROOT/'scripts/validation/verify-navigation-replay.py'))['load_grid'](folder)


def models(out, count):
    from nav_msgs.msg import OccupancyGrid
    config = json.loads((ROOT/'config/arena.json').read_text())
    config.update(json.loads((ROOT/'config/navigation-profiles.json').read_text())['candidate'])
    config.update(mission_variant='return', bounds=[-3.,7.,-4.,4.], lower_pole=[-10.,-10.],
                  upper_pole=[10.,10.], pole_clearance_from_map=True, parking=[4.,0.])
    rows = []
    for seed in range(count):
        rng = np.random.default_rng(seed)
        c = dict(config, max_speed=config['max_speed']*(.5,.75,1.)[seed%3])
        c['drive_acceleration_limit']=c['max_acceleration']
        grid = OccupancyGrid();grid.header.frame_id='map';grid.info.resolution=.05
        grid.info.width=200;grid.info.height=160;grid.info.origin.position.x=-3.;grid.info.origin.position.y=-4.
        grid.info.origin.orientation.w=1.;grid.data=[0]*32000
        plan = RoutePlan.build([('RETURN', [[x,0.] for x in np.linspace(0.,4.,81)])], c)
        center = np.array([rng.uniform(1.2,1.6), rng.uniform(-.02,.02)])
        radius = rng.uniform(.08,.15)
        channel=('both','left_only','right_only')[seed//3%3]
        wall=None
        if channel!='both':
            wall=np.array([[center[0]-.8,.35],[center[0]+.8,4.]])
            if channel=='left_only':wall[:,1]=[-4.,-.35]
            yy,xx=np.mgrid[:grid.info.height,:grid.info.width]
            cells=np.stack((-3.+(xx+.5)*.05,-4.+(yy+.5)*.05),axis=-1)
            occupied=np.all(cells>=wall[0],axis=-1)&np.all(cells<=wall[1],axis=-1)
            data=np.zeros(occupied.shape,dtype=np.int8);data[occupied]=100;grid.data=data.ravel().tolist()
        planner = DetourPlanner(c,grid)
        points = center + np.array([[x,y] for x in (-radius,0.,radius) for y in (-radius,0.,radius)])
        for i in range(6):planner.observations(points, i*.1)
        began=time.monotonic()
        result=planner.solve(dict(points=points, stamp=.6,plan=plan,progress=0.,pose=[0.,0.],
            velocity=[0.,0.],command=[0.,0.],acceleration=[0.,0.]))
        item=dict(seed=seed, passed=False, reason=result['reason'], planning_ms=result['seconds']*1000,
                  requested_speed=c['max_speed'],channel=channel,rejections=result['rejected'])
        selected=result['plan']
        if selected is not None:
            tracker=OmniTracker(selected,c); tracker.progress.s=result['progress']
            delay,tau=c['response_models'][seed%len(c['response_models'])]
            plant=Plant([0.,0.],delay,tau,acceleration_limit=c['drive_acceleration_limit'])
            policy=CommandPolicy([[1,0],[0,1]],c['max_speed'],c['max_acceleration'],c['max_speed'])
            policy.enabled=True;policy.last_tick=-.025
            minimum=math.inf;max_error=0.;trace=[]
            for tick in range(2000):
                t=tick*.025
                command,detail=tracker.step(plant.pose,0.,.025,stamp=t)
                policy.receive(*command,0.,now=t);sent,_=policy.tick(t)
                before=plant.pose.copy();plant.step(sent[:2])
                # Independent geometric oracle: the true square, not the planner's cluster circle.
                for f in (0.,.5,1.):
                    p=before+f*(plant.pose-before)
                    delta=np.maximum(np.abs(p-center)-radius,0.)
                    gap=float(np.linalg.norm(delta)-c['robot_radius'])
                    if wall is not None:
                        gap=min(gap,float(np.linalg.norm(np.maximum(np.maximum(wall[0]-p,p-wall[1]),0.))-c['robot_radius']))
                    minimum=min(minimum,gap)
                max_error=max(max_error,detail['error'])
                if tick%4==0:trace.append(dict(t=t,position=plant.pose.tolist(),command=list(sent[:2])))
                if minimum<=0 or max_error>c['max_tracking_error'] or detail['done']:break
            item.update(passed=bool(detail['done'] and minimum>0 and max_error<c['max_tracking_error']),
                seconds=t,minimum_clearance=minimum,max_tracking_error=max_error)
            if seed<3 or not item['passed']:
                (out/f'case-{seed:03}.jsonl').write_text(''.join(json.dumps(x)+'\n' for x in trace))
        item['wall_seconds']=time.monotonic()-began
        rows.append(item)
        save(out/'cases.json',rows)
        if seed%10==0 or not item['passed']:print(json.dumps(item),flush=True)
    report=dict(scope='Production planner/tracker/transport, synthetic obstacle truth and measured response models; not vehicle acceptance',
        count=count,passed=all(x['passed'] for x in rows),
        planning_p95_ms=float(np.percentile([r['planning_ms'] for r in rows],95)),cases=rows)
    report['functional_passed']=report['passed']
    report['performance_passed']=report['planning_p95_ms']<80
    report['passed']=report['functional_passed'] and report['performance_passed']
    save(out/'results.json',report)
    return report


def replay(out, archive):
    from avoidance_shadow import recorded_body_pose
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from intramural_mission.livox_adapter import convert
    from intramural_mission.safety import compensate_motion
    rows=[]
    for index in range(1,5):
        session=archive/'artifacts/mapping'/f'full-flow-20261020-{index:02}'
        folder=out/session.name;folder.mkdir()
        c=json.loads((session/'map/arena.json').read_text())
        grid=load_map(session/'map');planner=DetourPlanner(c,grid)
        reader=rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=str(session/'bag'),storage_id='sqlite3'),rosbag2_py.ConverterOptions('',''))
        types={t.name:get_message(t.type) for t in reader.get_all_topics_and_types()}
        scans=[];poses=[];imu=0
        while reader.has_next():
            name,data,stamp=reader.read_next()
            if name not in ('/livox/lidar','/livox/imu','/intramural_robot/aft_mapped_to_init'):continue
            message=deserialize_message(data,types[name])
            source=message.header.stamp.sec+message.header.stamp.nanosec/1e9
            if name=='/livox/lidar':scans.append((source,message))
            elif name=='/livox/imu':imu+=1
            else:
                poses.append([source,*recorded_body_pose(message,c['lidar_pose'])])
        history=np.asarray(poses)
        # The recording supplies a reference only; planner output never changes later observations.
        reference=[history[0,1:3].tolist()];reference_stamps=[history[0,0]]
        for sample in history:
            xy=sample[1:3]
            if np.linalg.norm(xy-reference[-1])>.04:
                reference.append(xy.tolist());reference_stamps.append(sample[0])
        shadow_config=dict(c,mission_variant='return',**json.loads((ROOT/'config/navigation-profiles.json').read_text())['candidate'])
        shadow_plan=RoutePlan.build([('RETURN',reference)],shadow_config) if len(reference)>2 else None
        shadow=DetourPlanner(shadow_config,grid)
        velocities=np.gradient(history[:,1:3],history[:,0],axis=0)
        decisions=[]
        observations=[];processed=0;missing=0;max_tracks=0;point_count=0
        for stamp,raw in scans:
            cloud=convert(raw)
            packed=np.ndarray((cloud.width,),dtype=np.dtype(dict(names=['x','y','z','offset'],formats=['<f4']*3+['<u4'],offsets=[0,4,8,20],itemsize=24)),buffer=cloud.data)
            xyz=np.column_stack([packed[k] for k in ('x','y','z')])
            body=lidar_to_body(xyz,c['lidar_pose'])
            valid=(body[:,2]>.15)&(body[:,2]<1.4)&(np.linalg.norm(body[:,:2],axis=1)>c['robot_radius'])
            times=stamp+packed['offset']*1e-9
            reference=float(max(stamp,times.max(initial=stamp)))
            # Completed bags can supply both bracketing odometry samples without extrapolation.
            if reference>history[-1,0] or min(stamp,reference)<history[0,0]:
                missing+=1;continue
            body=compensate_motion(body[valid,:2],times[valid],history,reference)
            position=np.array([np.interp(reference,history[:,0],history[:,k]) for k in (1,2)])
            yaw=np.interp(reference,history[:,0],np.unwrap(history[:,3]));co,si=math.cos(yaw),math.sin(yaw)
            world=body@np.array([[co,si],[-si,co]])+position
            within=np.linalg.norm(world-position,axis=1)<4.
            world=world[within]
            tracks=planner.observations(world,reference,sensor_position=position,point_times=times[valid][within])
            if shadow_plan is not None:
                shadow.observations(world,reference,sensor_position=position,point_times=times[valid][within])
                if processed%5==0:
                    progress=float(np.interp(reference,reference_stamps,shadow_plan.s))
                    # Distance is used only on this recorded reference, with its single RETURN gate.
                    progress=min(shadow_plan.s[-1],progress)
                    velocity=[float(np.interp(reference,history[:,0],velocities[:,k])) for k in (0,1)]
                    result=shadow.solve(dict(plan=shadow_plan,progress=progress,pose=position,velocity=velocity,
                        command=velocity,acceleration=[0.,0.],points=world,stamp=reference,
                        point_offsets=(times[valid][within]-stamp)*1e9,source_stamp=stamp,sensor_position=position))
                    proposal=result.pop('plan');result['has_proposal']=proposal is not None
                    decisions.append(result)
            max_tracks=max(max_tracks,len(tracks));processed+=1;point_count+=len(world)
            observations.append(dict(stamp=reference,pose=position.tolist(),tracks=tracks))
        (folder/'observations.jsonl').write_text(''.join(json.dumps(x)+'\n' for x in observations))
        (folder/'shadow-decisions.jsonl').write_text(''.join(json.dumps(x)+'\n' for x in decisions))
        row=dict(session=session.name,split='development' if index<3 else 'held_out',
            raw_scans=len(scans),processed_scans=processed,unbracketed_scans=missing,imu_messages=imu,
            obstacle_points=point_count,max_tracks=max_tracks,passed=processed>=.9*len(scans),
            scope='Original CustomMsg conversion, source-time motion compensation, recorded odometry and production obstacle tracker; shadow observations only')
        row['shadow_planning']=dict(decisions=len(decisions),proposals=sum(x['has_proposal'] for x in decisions),
            predicted_conflicts=sum(x['blocked'] for x in decisions),
            scope='Current candidate planner on a single-gate recorded-motion reference; not historical ordered mission progress or closed-loop execution')
        row['input_sha256']={f.name:hashlib.sha256(f.read_bytes()).hexdigest() for f in (session/'bag').glob('*.db3')}
        from avoidance_replay import raw_chain
        vehicle=archive/'cache/vehicle-20261015'
        if not (vehicle/'point-lio.yaml').exists():vehicle=ROOT/'cache/replay-vehicle'
        row['raw_chain']=raw_chain(ROOT,session,folder,vehicle,shadow=dict(config=shadow_config,
            grid=grid,plan=shadow_plan,stamps=reference_stamps))
        row['passed']=row['passed'] and row['raw_chain']['passed']
        rows.append(row);save(out/'results.json',dict(passed=all(x['passed'] for x in rows),sessions=rows))
        print(json.dumps(row),flush=True)
    return dict(passed=all(x['passed'] for x in rows),sessions=rows)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--suite',choices=['replay','model','gazebo','all'],default='all')
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--archive',type=Path,default=ROOT/'artifacts/field/2026-10-20/nuc-closeout/nuc')
    parser.add_argument('--cases',type=int,help='Explicit development subset; omitted uses 300 model / 60 Gazebo runs')
    parser.add_argument('--case-offset',type=int,default=0,help='Start at a named Gazebo matrix index for development')
    parser.add_argument('--case-indices',type=int,nargs='+',help='Explicit noncontiguous Gazebo subset; cannot combine with --cases or --case-offset')
    parser.add_argument('--baseline',action='store_true',help='Gazebo nominal runs with avoidance disabled, otherwise identical configuration')
    parser.add_argument('--gui',action='store_true',help='Show the physical Gazebo acceptance scene')
    parser.add_argument('--speed-scale',type=float,choices=(.5,.75,1.),default=1.,help='Gazebo speed fraction of unchanged candidate limit')
    args=parser.parse_args();out=args.output if args.output.is_absolute() else ROOT/args.output
    out=out.resolve()
    if not out.is_relative_to(ROOT/'artifacts') or out==ROOT/'artifacts' or out.exists():parser.error('Choose a new directory inside project artifacts/')
    if args.cases is not None and args.cases<1:parser.error('--cases must be positive')
    if args.case_offset<0:parser.error('--case-offset must be nonnegative')
    if args.case_indices is not None and (args.suite!='gazebo' or args.cases is not None or args.case_offset):
        parser.error('--case-indices requires Gazebo without --cases or --case-offset')
    if args.baseline and args.suite!='gazebo':parser.error('--baseline applies only to Gazebo')
    if args.suite in ('gazebo','all'):
        from avoidance_gazebo import matrix_indices
        try:matrix_indices(args.cases or 60,args.case_offset,args.baseline,args.case_indices)
        except ValueError as error:parser.error(str(error))
    if args.speed_scale!=1. and args.suite!='gazebo':parser.error('--speed-scale applies only to Gazebo')
    out.mkdir(parents=True)
    sources=list((ROOT/'nav_ws/src/intramural_mission/intramural_mission').glob('*.py'))
    sources+=list((ROOT/'nav_ws/src/intramural_base/intramural_base').glob('*.py'))
    sources+=list((ROOT/'nav_ws/src/intramural_sim').glob('*'))
    sources+=list((ROOT/'config').glob('*.json'))+list((ROOT/'launch').glob('arena*.py'))
    sources += [p for p in (ROOT/'scripts').rglob('*') if p.is_file() and p.suffix in ('.py','.sh','.cpp')]
    sources += [ROOT/'scripts/srm-nav',ROOT/'config/fastdds-services.xml',
        ROOT/'simulation/models/sentry_robot_cylinder.xacro',ROOT/'docker/sources.lock.json']
    manifest=dict(git=subprocess.check_output(['git','-C',str(ROOT),'rev-parse','HEAD'],text=True).strip(),
        runtime_image=os.environ.get('INTRAMURAL_IMAGE'),runtime_image_id=os.environ.get('INTRAMURAL_IMAGE_ID'),
        simulation_binary_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest()
            for p in (ROOT/'nav_ws/install/intramural_sim/lib/libintramural_drive.so',
                      ROOT/'nav_ws/install/intramural_sim/lib/intramural_sim/sensor_bridge') if p.is_file()},
        dirty=subprocess.check_output(['git','-C',str(ROOT),'status','--short'],text=True),
        source_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in (ROOT/'nav_ws/src/intramural_mission/intramural_mission').glob('*.py')},
        implementation_sha256={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources if p.is_file()},
        command=sys.argv,ros_domain=os.environ.get('ROS_DOMAIN_ID'),ign_partition=os.environ.get('IGN_PARTITION'),
        physical_devices_opened=False,suite=args.suite,development_subset=args.cases is not None or args.case_indices is not None)
    save(out/'manifest.json',manifest)
    reports={}
    for suite in (['replay','model','gazebo'] if args.suite=='all' else [args.suite]):
        folder=out/suite;folder.mkdir()
        if suite=='replay':report=replay(folder,args.archive)
        elif suite=='model':report=models(folder,args.cases or 300)
        else:
            from avoidance_gazebo import verify,verify_faults
            report=verify(ROOT,folder,args.cases or 60,args.case_offset,args.baseline,args.gui,args.speed_scale,args.case_indices)
            if args.cases is None and args.case_indices is None:
                report['faults']=verify_faults(ROOT,folder/'faults',folder/'arena.json')
                report['passed'] &= report['faults']['passed']
        reports[suite]=report
        changed=[name for name,digest in manifest['implementation_sha256'].items()
            if hashlib.sha256((ROOT/name).read_bytes()).hexdigest()!=digest]
        save(out/'results.json',dict(passed=not changed and all(x['passed'] for x in reports.values()),
            source_changed=changed,suites=reports))
    return 0 if not changed and all(x['passed'] for x in reports.values()) else 1


if __name__=='__main__':raise SystemExit(main())
