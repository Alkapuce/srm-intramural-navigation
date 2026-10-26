"""Independent physical oracle for the real arena launch and serial-simulation chain."""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import threading
import numpy as np


def aligned_clearance(measurements, actors, config):
    """Independent millisecond oracle; interpolate source-stamped truth, excluding teleport sweeps."""
    robot,actor=np.asarray(measurements),np.asarray(actors)
    if min(len(robot),len(actor))<2:return math.inf
    _,indices=np.unique(robot[:,0],return_index=True);robot=robot[np.sort(indices)]
    _,indices=np.unique(actor[:,0],return_index=True);actor=actor[np.sort(indices)]
    dt=.001;t=np.arange(max(robot[0,0],actor[0,0]),min(robot[-1,0],actor[-1,0]),dt)
    if not len(t):return math.inf
    a=np.column_stack([np.interp(t,robot[:,0],robot[:,k]) for k in (1,2)])
    b=np.column_stack([np.interp(t,actor[:,0],actor[:,k]) for k in (1,2)])
    index=np.clip(np.searchsorted(actor[:,0],t)-1,0,len(actor)-2)
    jumps=np.linalg.norm(np.diff(actor[:,1:],axis=0),axis=1)>np.maximum(.3,4*np.diff(actor[:,0]))
    gaps=np.linalg.norm(np.maximum(np.abs(a-b)-np.asarray(config['obstacle_size'][:2])/2,0.),axis=1)-config['robot_radius']
    valid=~jumps[index]
    speed=np.linalg.norm(np.diff(a-b,axis=0),axis=1)/dt
    speed[~(valid[:-1]&valid[1:])]=0.
    # The Lipschitz bound covers a closest approach between adjacent grid samples.
    bound=np.r_[speed,speed[-1] if len(speed) else 0.]*dt/2+.0003
    return float(min((gaps-bound)[valid])) if np.any(valid) else math.inf


def motion_timing(measurements):
    trace=np.asarray(measurements);dt=np.diff(trace[:,0])
    speed=np.linalg.norm(np.diff(trace[:,1:],axis=0),axis=1)/dt
    moving=np.flatnonzero(speed>=.05);active=np.zeros(len(dt),dtype=bool)
    if len(moving):active[moving[0]:moving[-1]+1]=True
    return dict(en_route_stopped_seconds=float(np.sum(dt[active&(speed<.05)])),
        en_route_below_point_two_mps_seconds=float(np.sum(dt[active&(speed<.2)])),
        stopped_seconds=float(np.sum(dt[speed<.05])),
        below_point_two_mps_seconds=float(np.sum(dt[speed<.2])),
        peak_measured_speed_mps=float(np.max(speed)),
        scope='Source-stamped physical position differences; whole run includes starting and final stop confirmation')


def response_agreement(measurements, commands, config):
    """Replay the observed serial command sequence in an independent planar integrator."""
    actual,commands=np.asarray(measurements),np.asarray(commands)
    if len(actual)<10 or len(commands)<10:
        return dict(passed=False,reason='Insufficient physical response samples')
    _,indices=np.unique(actual[:,0],return_index=True);actual=actual[np.sort(indices)]
    _,indices=np.unique(commands[:,0],return_index=True);commands=commands[np.sort(indices)]
    delay,tau=np.median(config['response_models'],axis=0)
    times=np.arange(actual[0,0],actual[-1,0]+.001,.005)
    pose,velocity=actual[0,1:].copy(),np.zeros(2);predicted=[]
    for stamp in times:
        i=np.searchsorted(commands[:,0],stamp-delay,side='right')-1
        u=commands[i,1:] if i>=0 else np.zeros(2)
        acceleration=(u-velocity)/tau
        acceleration*=min(1.,config['max_acceleration']/max(1e-9,np.linalg.norm(acceleration)))
        velocity+=acceleration*.005;pose+=velocity*.005;predicted.append(pose.copy())
    predicted=np.asarray(predicted)
    aligned=np.column_stack([np.interp(actual[:,0],times,predicted[:,i]) for i in (0,1)])
    errors=np.linalg.norm(aligned-actual[:,1:],axis=1)
    global_rmse=float(np.sqrt(np.mean(errors**2)))
    source=np.column_stack([np.interp(times,actual[:,0],actual[:,i+1]) for i in (0,1)])
    observed_velocity=np.gradient(source,.005,axis=0)
    local=[]
    for i,stamp in enumerate(times):
        # The planner predicts two seconds from an observed state, not an entire lap open loop.
        if i%400==0:pose,velocity=source[i].copy(),observed_velocity[i].copy()
        index=np.searchsorted(commands[:,0],stamp-delay,side='right')-1
        u=commands[index,1:] if index>=0 else np.zeros(2)
        acceleration=(u-velocity)/tau
        acceleration*=min(1.,config['max_acceleration']/max(1e-9,np.linalg.norm(acceleration)))
        velocity+=acceleration*.005;pose+=velocity*.005;local.append(pose.copy())
    errors=np.linalg.norm(np.asarray(local)-source,axis=1)
    rmse=float(np.sqrt(np.mean(errors**2)))
    return dict(passed=rmse<.08,position_rmse_m=rmse,position_max_error_m=float(max(errors)),
        samples=len(actual),commands=len(commands),delay=float(delay),tau=float(tau),
        window_seconds=2.,global_position_rmse_m=global_rmse,
        scope='Simulated force response to the same received serial sequence; command receipt timing, not real actuator calibration')


def verify_faults(root,out,config):
    out.mkdir()
    rows=[]
    for kind in ('pointcloud','localization','tf','control','planner','transport_link'):
        folder=out/kind
        with (out/(kind+'.log')).open('w') as log:
            process=subprocess.run([sys.executable,str(root/'scripts/validation/verify-stage6-fault.py'),
                '--output',str(folder),'--fault',kind],env=dict(os.environ,INTRAMURAL_ARENA_CONFIG=str(config)),
                stdout=log,stderr=subprocess.STDOUT)
        row=dict(fault=kind,passed=process.returncode==0 and (folder/'results.json').exists(),exit_code=process.returncode)
        rows.append(row);print(json.dumps(row),flush=True)
        (out/'results.json').write_text(json.dumps(dict(passed=all(r['passed'] for r in rows),cases=rows),indent=2)+'\n')
    return dict(passed=all(r['passed'] for r in rows),cases=rows,
        wire_break_scope='Unprotected simulated MCU must retain motion until communication is restored; this is a boundary reproduction, not physical disconnect safety')


def verify(root, out, count, offset=0, baseline=False, gui=False):
    import rclpy
    from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import Twist
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from rclpy.parameter import Parameter
    from rclpy.executors import SingleThreadedExecutor
    from intramural_mission.bypass import winding
    c=json.loads((root/'config/arena.json').read_text())
    c.update(json.loads((root/'config/navigation-profiles.json').read_text())['candidate'])
    c.update(allow_bypass=not baseline,
             simulation_drive='response',navigation_profile='candidate')
    c['drive_acceleration_limit']=c['max_acceleration']
    c['avoidance_actor']=True
    c.update(actor_acceleration=2.,actor_tau=.08)
    c['obstacle_initial']=[c['bounds'][1]+5.,c['bounds'][3]+5.]
    config=out/'arena.json';config.write_text(json.dumps(c,indent=2)+'\n')
    env=dict(os.environ,INTRAMURAL_ARENA_CONFIG=str(config),INTRAMURAL_TRANSPORT='serial-sim')
    subprocess.run([sys.executable,str(root/'scripts/internal/generate-arena.py')],env=env,check=True)
    log=(out/'system.log').open('w')
    process=subprocess.Popen([sys.executable,str(root/'scripts/internal/arena-process.py'),
        '--localization','sensors','--transport','serial-sim']+
        ([] if gui else ['--no-gui']),
        env=env,stdout=log,stderr=subprocess.STDOUT)
    rclpy.init();node=rclpy.create_node('avoidance_physical_acceptance',parameter_overrides=[Parameter('use_sim_time',value=True)])
    status={};truth={};commands=[];path=[];guard_ms=[];controller_ms=[];planner_ms=[];planner_seen=set()
    current=[];actuation=[];measurements=[];actor_trace=[];actor_state={};progress_clock=[];switches=[];bypass_seen={}
    obstacle=None;last_obstacle=None;minimum=math.inf;static_minimum=math.inf;last_pose=None;sim_stamps=[]
    def state(msg):
        nonlocal status
        status=json.loads(msg.data)
        controller_ms.append(status.get('control_tick_ms',0.))
        run=status.get('run_id');count=status.get('bypass_count',0)
        if count>bypass_seen.get(run,0):
            bypass_seen[run]=count
            switches.append(dict(run_id=run,route_id=status.get('route_id'),bypass_count=count,
                speed_mps=truth.get('speed'),source_age_seconds=node.get_clock().now().nanoseconds/1e9-truth.get('stamp',0.)))
        report=status.get('avoidance',{})
        key=status.get('run_id'),report.get('route_id'),report.get('stamp')
        if report.get('seconds') is not None and key not in planner_seen:
            planner_seen.add(key);planner_ms.append(report['seconds']*1000)
        if status.get('state')=='RUNNING':
            progress_clock.append([status.get('original_progress',0.),node.get_clock().now().nanoseconds/1e9])
    def measured(msg):
        nonlocal minimum,static_minimum,last_pose,last_obstacle
        p=msg.pose.pose.position;xy=np.array([p.x,p.y])
        truth.update(position=xy,speed=math.hypot(msg.twist.twist.linear.x,msg.twist.twist.linear.y),
            stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9)
        q=msg.pose.pose.orientation
        truth['yaw']=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        measurements.append([msg.header.stamp.sec+msg.header.stamp.nanosec/1e9,p.x,p.y])
        current.append(xy.tolist());sim_stamps.append(msg.header.stamp.sec+msg.header.stamp.nanosec/1e9)
        previous=xy if last_pose is None else last_pose
        for f in (0.,.25,.5,.75,1.):
            swept=previous+f*(xy-previous)
            static_minimum=min(static_minimum,*(float(np.linalg.norm(swept-c[key])-c['pole_radius']-c['robot_radius'])
                for key in ('lower_pole','upper_pole')))
            xmin,xmax,ymin,ymax=c['bounds']
            static_minimum=min(static_minimum,float(min(swept[0]-xmin,xmax-swept[0],swept[1]-ymin,ymax-swept[1])-c['robot_radius']))
        actual_obstacle=actor_state.get('position') if obstacle is not None else None
        if actual_obstacle is not None:
            previous=xy if last_pose is None else last_pose
            old=actual_obstacle if last_obstacle is None or np.linalg.norm(actual_obstacle-last_obstacle)>.3 else last_obstacle
            for f in (0.,.25,.5,.75,1.):
                relative=previous+f*(xy-previous)-(old+f*(actual_obstacle-old))
                gap=np.linalg.norm(np.maximum(np.abs(relative)-np.array(c['obstacle_size'][:2])/2,0.))-c['robot_radius']
                minimum=min(minimum,float(gap))
        last_pose=xy.copy();last_obstacle=None if actual_obstacle is None else actual_obstacle.copy()
    ns='/'+c['namespace']+'/'
    node.create_subscription(String,ns+'mission/status',state,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(Odometry,ns+'chassis_odometry_gt',measured,qos_profile_sensor_data)
    node.create_subscription(Twist,ns+'cmd_vel',lambda m:commands.append([m.linear.x,m.linear.y]),10)
    def transport(m):
        yaw=truth.get('yaw',0.);co,si=math.cos(yaw),math.sin(yaw)
        actuation.append([node.get_clock().now().nanoseconds/1e9,
            co*m.linear.x-si*m.linear.y,si*m.linear.x+co*m.linear.y])
    node.create_subscription(Twist,ns+'cmd_vel_transport',transport,10)
    def actor_pose(message):
        p=message.pose.pose.position
        stamp=message.header.stamp.sec+message.header.stamp.nanosec/1e9
        actor_state.update(position=np.array([p.x,p.y]),stamp=stamp)
        actor_trace.append([stamp,p.x,p.y])
    node.create_subscription(Odometry,ns+'acceptance/obstacle_odometry',actor_pose,qos_profile_sensor_data)
    actor_command=node.create_publisher(Twist,ns+'acceptance/obstacle_velocity',10)
    def actor_velocity(velocity):
        msg=Twist();msg.linear.x,msg.linear.y=map(float,velocity);actor_command.publish(msg)
    node.create_subscription(String,ns+'motion_guard/status',lambda m:guard_ms.append(json.loads(m.data).get('guard_tick_ms',0.)),10)
    clients={name:node.create_client(Trigger,ns+'mission/'+name) for name in ('start','start_here','prepare','cancel','reset')}
    executor=SingleThreadedExecutor();executor.add_node(node)
    reader=threading.Thread(target=executor.spin,daemon=True);reader.start()
    def until(check,seconds,label):
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:
            if process.poll() is not None:raise RuntimeError(f'Arena exited during {label}: {process.returncode}')
            time.sleep(.01)
            if status.get('state')=='FAULT':raise RuntimeError(f'Mission fault during {label}: {status.get("reason")}')
            if check():return
        raise RuntimeError(f'Timeout {label}: {status.get("state")} {status.get("reason")} {status.get("avoidance")}')
    def call(name):
        if not clients[name].wait_for_service(timeout_sec=5):raise RuntimeError('Missing service '+name)
        future=clients[name].call_async(Trigger.Request());until(future.done,180 if name in ('start','start_here','prepare') else 10,name)
        if not future.result().success:raise RuntimeError(f'{name}: {future.result().message}')
    def move(point):
        nonlocal obstacle
        req=f'name: "movable_obstacle", position: {{x: {point[0]}, y: {point[1]}, z: {c["obstacle_size"][2]/2}}}, orientation: {{w: 1}}'
        result=subprocess.run(['ign','service','-s','/world/default/set_pose','--reqtype','ignition.msgs.Pose',
            '--reptype','ignition.msgs.Boolean','--timeout','3000','--req',req],text=True,capture_output=True,timeout=5)
        if result.returncode or 'true' not in result.stdout:
            raise RuntimeError(f'Obstacle move not acknowledged: rc={result.returncode}, stdout={result.stdout!r}, stderr={result.stderr!r}')
        obstacle=np.asarray(point)
    results=[];failure=None
    try:
        until(lambda:status.get('state')=='IDLE' and status.get('ready') and truth and actor_state,120,'sensor readiness')
        reference_clock=None;reference_motion=None
        indices=list(range(offset,offset+count))
        if offset==0 and count==60 and not baseline:
            indices.remove(47);indices.insert(0,47)
        for index in [-1,*indices]:
            if status.get('state')=='DONE':call('reset')
            actor_velocity([0.,0.]);move(c['obstacle_initial']);obstacle=None
            until(lambda:status.get('state')=='IDLE' and status.get('ready'),10,'next run')
            entry='start_here' if index==60 else 'prepare' if index==61 else 'start'
            preparation=None
            if entry!='start':
                call('start')
                until(lambda:truth.get('position') is not None and
                    np.linalg.norm(truth['position']-c['parking'])>.6,30,'entry departure')
                call('cancel')
                until(lambda:status.get('state')=='CANCELLED' and truth.get('speed',1.)<.03,10,'entry cancellation')
                call('reset')
                until(lambda:status.get('state')=='IDLE' and status.get('ready'),10,'entry reset')
                if entry=='prepare':
                    call('prepare')
                    until(lambda:status.get('state')=='PREPARED',10,'prepared route')
                    time.sleep(.5)
                    preparation=dict(state=status.get('state'),speed=truth.get('speed'),command=commands[-1])
                    if preparation['state']!='PREPARED' or preparation['speed']>.03 or any(abs(v)>1e-6 for v in preparation['command']):
                        raise RuntimeError(f'Preparation moved the vehicle: {preparation}')
            current.clear();commands.clear();actuation.clear();measurements.clear();actor_trace.clear();progress_clock.clear()
            minimum=math.inf;static_minimum=math.inf;last_pose=None;last_obstacle=None
            begin_wall=time.monotonic();begin_sim=sim_stamps[-1]
            call('start_here' if entry=='start_here' else 'start')
            until(lambda:status.get('run_id') and status.get('state')=='RUNNING',20,'start')
            run_id=status['run_id']
            motion_start=node.get_clock().now().nanoseconds/1e9
            route=json.loads((root/'artifacts/navigation-runs'/run_id/'route.json').read_text())
            points=np.asarray(route['points']);gates=route['gates']
            kind='nominal' if index<10 or index>=60 else ('static' if index<40 else 'dynamic')
            target_phase=(0,2,4,3,7,9)[max(0,index-10)%6]
            a=0 if target_phase==0 else gates[target_phase-1][1]
            b=gates[target_phase][1]
            at=b if target_phase in (3,7) else (a+b)//2
            target=points[at];tangent=np.asarray(route['tangent'][at]);normal=np.array([-tangent[1],tangent[0]])
            active=False;crossing=None;finished_actor=False;speed=(.5,1.,1.5)[index%3]
            maneuver=('cross_left','cross_right','head_on','diagonal','slow_lead','stop_go','cross_again')[(index-40)%7]
            direction={'cross_left':normal,'cross_right':-normal,'head_on':-tangent,
                'diagonal':(normal-tangent)/math.sqrt(2.),'slow_lead':tangent,
                'stop_go':normal,'cross_again':-normal}[maneuver]
            def space(sign):
                safe=0.
                half=np.asarray(c['obstacle_size'][:2])/2
                for distance in np.arange(.05,.91,.05):
                    point=target+sign*direction*distance
                    xmin,xmax,ymin,ymax=c['bounds']
                    if min(point[0]-xmin-half[0],xmax-point[0]-half[0],point[1]-ymin-half[1],ymax-point[1]-half[1])<.05:break
                    if any(np.linalg.norm(np.maximum(np.abs(point-c[key])-half,0.))<c['pole_radius']+.05 for key in ('lower_pole','upper_pole')):break
                    safe=float(distance)
                return safe
            entrance,exit_distance=space(-1),space(1)
            if kind=='dynamic' and min(entrance,exit_distance)<.65:
                target_phase=3 if index%2 else 7
                at=gates[target_phase][1];target=points[at];tangent=np.asarray(route['tangent'][at])
                normal=np.array([-tangent[1],tangent[0]])
                direction={'cross_left':normal,'cross_right':-normal,'head_on':-tangent,
                    'diagonal':(normal-tangent)/math.sqrt(2.),'slow_lead':tangent,
                    'stop_go':normal,'cross_again':-normal}[maneuver]
                entrance,exit_distance=space(-1),space(1)
            reversal=min(.3,.5*min(entrance,exit_distance)/speed)
            travel_time=c['actor_tau']+(entrance/speed+speed/(2*c['actor_acceleration'])
                if entrance>=speed**2/(2*c['actor_acceleration']) else math.sqrt(2*entrance/c['actor_acceleration']))
            s=np.asarray(route['s']);v=np.asarray(route['speed'])
            # A separate unobstructed run fixes actor timing before any avoidance response.
            nominal_arrival=(float(np.interp(s[at],reference_clock[:,0],reference_clock[:,1]))
                if reference_clock is not None else 0.)
            actor_start=motion_start+max(0.,nominal_arrival-travel_time)
            if kind=='dynamic':
                if min(entrance,exit_distance)<.1:raise RuntimeError('Actor has insufficient physical crossing space')
                move(target-direction*entrance)
                obstacle=None
            if kind=='static':move(target);active=True
            deadline=time.monotonic()+100
            while status.get('state')!='DONE' and time.monotonic()<deadline:
                until(lambda:truth,1,'pose')
                now=time.monotonic()
                if kind=='dynamic':
                    if crossing is None and not finished_actor and node.get_clock().now().nanoseconds/1e9>=actor_start:
                        obstacle=actor_state['position'].copy();active=True
                        crossing=node.get_clock().now().nanoseconds/1e9+travel_time
                    if active:
                        t=node.get_clock().now().nanoseconds/1e9-crossing
                        velocity=direction*speed
                        if maneuver=='stop_go' and 0.<t<1.:velocity=direction*0.
                        if maneuver=='cross_again' and reversal<t<3*reversal:velocity=-velocity
                        actor_velocity(velocity)
                        exit_time=exit_distance/speed+(1. if maneuver=='stop_go' else 4*reversal if maneuver=='cross_again' else 0.)
                        if t>exit_time and np.linalg.norm(actor_state['position']-target)>exit_distance:
                            actor_velocity([0.,0.]);move(c['obstacle_initial']);obstacle=None;active=False;finished_actor=True
                if minimum<-.05 or static_minimum<=0:
                    raise RuntimeError(f'Physical contact in run {index}: obstacle={minimum}, poles={static_minimum}')
                if status.get('state')=='FAULT':raise RuntimeError(f'Run {index}: {status.get("reason")}')
                time.sleep(.015)
            if status.get('state')!='DONE':raise RuntimeError(f'Run {index} did not finish: {status.get("avoidance")}')
            elapsed=time.monotonic()-begin_wall;sim_elapsed=sim_stamps[-1]-begin_sim
            if kind!='nominal':minimum=aligned_clearance(measurements,actor_trace,c)
            turns=[winding(current,c[key]) for key in ('lower_pole','upper_pole')]
            parked=float(np.linalg.norm(truth['position']-c['parking']))
            nominal_gap=None
            if kind=='dynamic' and reference_motion is not None and actor_trace:
                actors=np.asarray(actor_trace);times=actors[:,0]-motion_start
                nominal=np.column_stack([np.interp(times,reference_motion[:,0],reference_motion[:,k]) for k in (1,2)])
                gaps=np.linalg.norm(np.maximum(np.abs(nominal-actors[:,1:])-np.asarray(c['obstacle_size'][:2])/2,0.),axis=1)-c['robot_radius']
                nominal_gap=float(np.min(gaps))
            passed=(turns==[1,-1] and len(status['history'])==10 and parked<c['parking_tolerance']
                    and (kind!='static' or status['bypass_count']>0) and min(minimum,static_minimum)>0
                    and (kind!='dynamic' or (crossing is not None and math.isfinite(minimum) and nominal_gap is not None and nominal_gap<.15))
                    and sim_elapsed/elapsed>=.9)
            row=dict(index=index,kind=kind,entry=entry,preparation=preparation,run_id=run_id,passed=bool(passed),wall_seconds=elapsed,
                sim_seconds=sim_elapsed,real_time_factor=sim_elapsed/elapsed,parking_error=parked,winding=turns,
                bypass_count=status['bypass_count'],blocked_count=status['blocked_count'],
                visibility='present_before_motion' if kind=='dynamic' else None,
                target_phase=target_phase,obstacle_speed_command=speed if kind=='dynamic' else 0.,
                actor_start_sim=actor_start if kind=='dynamic' else None,
                nominal_counterfactual_clearance=nominal_gap,
                minimum_pole_clearance=static_minimum,
                minimum_dynamic_clearance=None if not math.isfinite(minimum) else minimum)
            row['motion_timing']=motion_timing(measurements)
            row['route_switches']=[s for s in switches if s['run_id']==run_id]
            row['response']=response_agreement(measurements,actuation,c)
            row['truth_sampling']={name:dict(samples=len(values),max_gap_seconds=float(np.max(np.diff(np.asarray(values)[:,0]))))
                for name,values in (('robot',measurements),('obstacle',actor_trace)) if len(values)>1}
            row['passed'] &= (row['response']['passed'] and len(row['truth_sampling'])==2
                and all(v['max_gap_seconds']<=.1 for v in row['truth_sampling'].values()))
            row['maneuver']=maneuver if kind=='dynamic' else None
            if index<0:
                if not row['passed']:raise RuntimeError(f'Unobstructed timing reference failed: {row}')
                reference_clock=np.asarray(progress_clock);reference_clock[:,1]-=motion_start
                reference_motion=np.asarray(measurements);reference_motion[:,0]-=motion_start
                (out/'reference-clock.json').write_text(json.dumps(dict(result=row,progress_clock=reference_clock.tolist(),
                    measured_motion=reference_motion.tolist()))+'\n')
                continue
            results.append(row);(out/f'run-{index:02}.json').write_text(json.dumps(dict(result=row,truth=current),indent=2)+'\n')
            (out/f'inputs-{index:02}.json').write_text(json.dumps(dict(transport_world=actuation,
                measured_positions=measurements,obstacle_poses=actor_trace))+'\n')
            (out/'cases.json').write_text(json.dumps(results,indent=2)+'\n');print(json.dumps(row),flush=True)
            if not row['passed']:raise RuntimeError(f'Run failed independent acceptance: {row}')
            actor_velocity([0.,0.]);move(c['obstacle_initial']);obstacle=None
    except Exception as error:
        failure=str(error)
        (out/'failure.json').write_text(json.dumps(dict(error=failure,status=status),indent=2)+'\n')
        (out/'failed-truth.json').write_text(json.dumps(current)+'\n')
        (out/'failed-inputs.json').write_text(json.dumps(dict(transport_world=actuation,
            measured_positions=measurements,obstacle_poses=actor_trace))+'\n')
    finally:
        executor.shutdown();reader.join(timeout=5.);node.destroy_node();rclpy.shutdown()
        process.send_signal(signal.SIGINT)
        try:process.wait(timeout=100)
        except subprocess.TimeoutExpired:
            process.kill();process.wait();failure=failure or 'Arena forced exit'
        log.close()
    report=dict(passed=failure is None and process.returncode==0 and len(results)==count,
        requested_runs=count,completed_runs=len(results),failure=failure,exit_code=process.returncode,
        control_p99_ms=float(np.percentile(controller_ms,99)) if controller_ms else None,
        guard_p99_ms=float(np.percentile(guard_ms,99)) if guard_ms else None,cases=results,
        planner_p95_ms=float(np.percentile(planner_ms,95)) if planner_ms else None,
        planning_reports=len(planner_ms),planning_timeouts=sum(t>80 for t in planner_ms),
        scope='Force-driven Gazebo, raw simulated sensors, Point-LIO + small_gicp and PTY serial; not real-vehicle acceptance')
    report['functional_passed']=report['passed']
    report['performance_passed']=bool(controller_ms and guard_ms and report['control_p99_ms']<10 and report['guard_p99_ms']<5
        and (baseline or planner_ms and report['planner_p95_ms']<80))
    report['passed']=report['functional_passed'] and report['performance_passed']
    (out/'results.json').write_text(json.dumps(report,indent=2)+'\n')
    return report
