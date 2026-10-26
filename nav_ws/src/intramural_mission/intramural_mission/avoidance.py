"""Bounded, ordered detours; the raw-cloud guard never depends on this planner."""
from dataclasses import replace
from dataclasses import asdict
import json
import hashlib
import math
import multiprocessing as mp
import queue
import time
import numpy as np
from scipy.ndimage import label,binary_dilation
from scipy.optimize import linear_sum_assignment
from scipy.spatial import cKDTree
from intramural_mission.bypass import entry_path, winding
from intramural_mission.trajectory import RoutePlan
from intramural_mission.route import quintic_curve
from intramural_mission.safety import response_horizon,stopping_horizon


def acceleration_motion(dt):
    tau=.4
    velocity=-tau*np.expm1(-np.asarray(dt)/tau)
    return tau*(dt-velocity),velocity,np.exp(-np.asarray(dt)/tau)


def obstacle_transition(dt):
    position,velocity,decay=acceleration_motion(dt)
    f=np.eye(6);f[:2,2:4]=np.eye(2)*dt;f[:2,4:]=np.eye(2)*position
    f[2:4,4:]=np.eye(2)*velocity;f[4:,4:]=np.eye(2)*decay
    return f


class ObstacleTracker:
    def __init__(self):
        self.tracks, self.stamp, self.serial = [], -math.inf, 0

    def update(self, points, stamp, weights=None,sensor_position=None,point_times=None):
        points = np.asarray(points, dtype=float).reshape(-1, 2)
        weights = np.ones(len(points)) if weights is None else np.asarray(weights)
        point_times=np.full(len(points),stamp) if point_times is None else np.asarray(point_times,dtype=float)
        if not math.isfinite(stamp) or not np.isfinite(points).all():
            raise ValueError('Finite timestamped obstacle points required')
        if weights.shape != (len(points),) or not np.isfinite(weights).all() or np.any(weights <= 0):
            raise ValueError('Positive observation support counts required')
        if point_times.shape!=(len(points),) or not np.isfinite(point_times).all() or np.any(point_times>stamp+.01):
            raise ValueError('Observation source times must precede the prediction reference')
        if stamp <= self.stamp:
            return self.snapshot(self.stamp)
        self.stamp = stamp
        clusters = []
        if len(points):
            cells = np.floor(points/.10+1e-7).astype(int)
            origin = cells.min(axis=0)
            cells -= origin
            shape = cells.max(axis=0)+1
            if max(shape) > 512:
                raise ValueError('Obstacle observations exceed local planning window')
            occupied = np.zeros(shape, bool)
            occupied[tuple(cells.T)] = True
            # Join scan gaps narrower than the vehicle, including partially visible box faces.
            groups, count = label(binary_dilation(occupied), np.ones((3, 3)))
            ids = groups[tuple(cells.T)]
            for i in range(1, count+1):
                cluster = points[ids == i]
                if sum(weights[ids == i]) >= 3:
                    center = (cluster.min(axis=0)+cluster.max(axis=0))/2
                    clusters.append((center, float(max(np.linalg.norm(cluster-center, axis=1)))+.025,
                                     (cluster.max(axis=0)-cluster.min(axis=0))/2+.025,
                                     float(np.average(point_times[ids==i],weights=weights[ids==i]))))
        self.tracks = [t for t in self.tracks if stamp-t['seen'] <= .6]
        for t in self.tracks:
            dt = stamp-t['stamp']
            f=obstacle_transition(dt)
            g=np.vstack((np.eye(2)*dt**3/6,np.eye(2)*dt*dt/2,np.eye(2)*dt))
            t['x'] = f@t['x']
            t['p'] = f@t['p']@f.T + 15.**2*g@g.T
            t['stamp'] = stamp
            t['dt']=dt
        used = set()
        if self.tracks and clusters:
            costs = np.array([[np.linalg.norm(obstacle_transition(c[3]-stamp)[:2]@t['x']-c[0]) for c in clusters] for t in self.tracks])
            rows, cols = linear_sum_assignment(costs)
            for i, j in zip(rows, cols):
                t = self.tracks[i]
                if costs[i, j] > .45 + 2*math.sqrt(max(np.diag(t['p'])[:2])):
                    continue
                extent=np.maximum(t['extent'],clusters[j][2])
                lag=stamp-clusters[j][3]
                measurement=obstacle_transition(-lag)[:2]
                expected=measurement@t['x']
                center=np.clip(expected,clusters[j][0]+clusters[j][2]-extent,
                    clusters[j][0]-clusters[j][2]+extent)
                if sensor_position is not None:
                    partial=clusters[j][2]<.5*extent
                    face_center=clusters[j][0]+np.sign(clusters[j][0]-sensor_position)*(extent-clusters[j][2])
                    center=np.where(partial,face_center,center)
                # Only newly revealed dimensions recenter the shape; observable axes still estimate motion.
                revealed=clusters[j][2]>t['extent']+.04
                shape_shift=np.where(revealed,center-expected,0.)
                t['x'][:2]+=shape_shift
                t['measurement_center']+=shape_shift
                innovation = measurement@t['p']@measurement.T+np.eye(2)*.02**2
                residual=center-measurement@t['x']
                if float(residual@np.linalg.solve(innovation,residual))>9.:
                    # A motion start is a velocity jump, not measurement noise to average away.
                    dt=max(.01,clusters[j][3]-t['seen'])
                    velocity=(center-t['measurement_center'])/dt
                    t['x']=obstacle_transition(lag)@np.r_[center,velocity,t['x'][4:]]
                    t['p']=np.diag([.1**2]*2+[2*.02**2/dt**2]*2+[2.**2]*2)
                    innovation=measurement@t['p']@measurement.T+np.eye(2)*.02**2
                k = np.linalg.solve(innovation, (t['p']@measurement.T).T).T
                t['x'] += k@(center-measurement@t['x'])
                h = np.eye(6)-k@measurement
                t['p'] = h@t['p']@h.T + k@(np.eye(2)*.02**2)@k.T
                t.update(radius=float(np.linalg.norm(extent)), extent=extent, seen=clusters[j][3],measurement_center=center,
                    observations=t['observations']+1)
                # A small derivative inside its own uncertainty is scan jitter, not motion evidence.
                sigma=np.sqrt(np.maximum(0.,np.diag(t['p'])))
                if (np.linalg.norm(t['x'][2:4])>max(.08,max(sigma[2:4])) or
                        np.linalg.norm(t['x'][4:])>max(.4,max(sigma[4:]))):t['moving_until']=stamp+.3
                used.add(j)
        for j, (center, radius, extent, source_time) in enumerate(clusters):
            if j not in used:
                self.serial += 1
                initial=np.diag([.02**2]*2+[.5**2]*2+[2.**2]*2)
                advance=obstacle_transition(stamp-source_time)
                self.tracks.append(dict(id=self.serial, x=np.r_[center, 0., 0.,0.,0.],
                    p=advance@initial@advance.T,
                    radius=radius,extent=extent,stamp=stamp,seen=source_time,measurement_center=center,observations=1,moving_until=stamp))
        return self.snapshot(stamp)

    def snapshot(self, stamp):
        return [dict(id=t['id'], center=t['x'][:2].tolist(), velocity=t['x'][2:4].tolist(),acceleration=t['x'][4:].tolist(),
                     covariance=t['p'].tolist(), radius=t['radius'], extent=t['extent'].tolist(), age=max(0., stamp-t['seen']),
                     velocity_observed=t['observations']>=2,
                     moving=bool(t['observations']<3 or stamp<t['moving_until']))
                for t in self.tracks]


def obstacle_gap(points, track):
    delta=np.asarray(points)-track['center']
    if 'extent' in track:
        return np.linalg.norm(np.maximum(np.abs(delta)-track['extent'],0.),axis=-1)
    return np.linalg.norm(delta,axis=-1)-track['radius']


def segment_box_distance(a, b, extent):
    """Exact distance from each swept center segment to an axis aligned contour."""
    extent=np.asarray(extent);d=b-a;length2=np.maximum(1e-12,np.sum(d*d,axis=1))
    distance=np.minimum(np.linalg.norm(np.maximum(np.abs(a)-extent,0.),axis=1),
                        np.linalg.norm(np.maximum(np.abs(b)-extent,0.),axis=1))
    for sign in ((-1,-1),(-1,1),(1,-1),(1,1)):
        corner=extent*sign
        fraction=np.clip(np.sum((corner-a)*d,axis=1)/length2,0.,1.)
        distance=np.minimum(distance,np.linalg.norm(a+fraction[:,None]*d-corner,axis=1))
    parallel=np.abs(d)<1e-12
    safe=np.where(parallel,1.,d)
    t0,t1=(-extent-a)/safe,(extent-a)/safe
    lo=np.where(parallel,-np.inf,np.minimum(t0,t1)).max(axis=1)
    hi=np.where(parallel,np.inf,np.maximum(t0,t1)).min(axis=1)
    intersects=(np.maximum(0.,lo)<=np.minimum(1.,hi))&~np.any(parallel&(np.abs(a)>extent),axis=1)
    return np.where(intersects,0.,distance)


def collision(points, times, tracks, radius):
    """Sweep each interval against relative obstacle motion, including uncertain position."""
    p, times = np.asarray(points), np.asarray(times)
    return collision_segments(p[:-1],p[1:],times[:-1],times[1:],tracks,radius)


def prediction_uncertainty(track,horizon):
    covariance=np.asarray(track['covariance'])
    horizon=np.asarray(horizon)+track.get('prediction_age',0.)
    if not track.get('moving',True):
        return np.maximum(0.,2*math.sqrt(max(np.diag(covariance)[:2]))+.02*horizon-track.get('shared_margin',0.))
    h=horizon if track.get('velocity_observed',True) else np.minimum(horizon,.15)
    variance=np.maximum(covariance[0,0]+2*h*covariance[0,2]+h*h*covariance[2,2],
                        covariance[1,1]+2*h*covariance[1,3]+h*h*covariance[3,3])
    if len(covariance)==6:
        a=acceleration_motion(h)[0]
        variance=np.maximum(covariance[0,0]+2*h*covariance[0,2]+h*h*covariance[2,2]
            +a*a*covariance[4,4]+2*a*covariance[0,4]+2*h*a*covariance[2,4],
            covariance[1,1]+2*h*covariance[1,3]+h*h*covariance[3,3]
            +a*a*covariance[5,5]+2*a*covariance[1,5]+2*h*a*covariance[3,5])
    # The collision radius already contains the shared static error budget.
    return np.maximum(0.,2*np.sqrt(np.maximum(0.,variance))+.025*horizon*horizon-track.get('shared_margin',0.))


def obstacle_positions(track,times):
    t=np.asarray(times)+track.get('prediction_age',0.)
    positions=np.asarray(track['center'])+t[...,None]*np.asarray(track['velocity'])
    acceleration=np.asarray(track.get('acceleration',[0.,0.]))
    if track.get('moving',True) and np.any(acceleration):
        positions+=acceleration_motion(t)[0][...,None]*acceleration
    return positions


def collision_segments(start,end,begin,finish,tracks,radius):
    for track in tracks:
        a=np.asarray(start)-obstacle_positions(track,begin)
        b=np.asarray(end)-obstacle_positions(track,finish)
        acceleration=np.asarray(track.get('acceleration',[0.,0.]))
        d=b-a
        f = np.clip(-np.sum(a*d, axis=1)/np.maximum(1e-12, np.sum(d*d, axis=1)), 0., 1.)
        uncertainty=prediction_uncertainty(track,finish)
        if track.get('moving',True):
            uncertainty+=np.linalg.norm(acceleration)*(np.asarray(finish)-np.asarray(begin))**2/8
        broad=np.linalg.norm(a+f[:,None]*d,axis=1)
        bound=np.linalg.norm(track['extent']) if 'extent' in track else track['radius']
        if not np.any(broad<=radius+bound+uncertainty):
            continue
        gap=(segment_box_distance(a,b,track['extent']) if 'extent' in track
             else np.linalg.norm(a+f[:, None]*d,axis=1)-track['radius'])
        if np.any(gap <= radius+uncertainty):
            return True
    return False


def stopped_collision(brakes,brake_times,tracks,radius):
    # Rest is still occupied space: a moving obstacle may arrive after braking ends.
    horizon=np.maximum(2.,brake_times[:,-1,None])
    begin=np.minimum(brake_times[:,-1,None]+np.arange(0.,2.05,.05),horizon)
    finish=np.minimum(begin+.05,horizon)
    points=np.broadcast_to(brakes[:,-1,None,:],begin.shape+(2,)).reshape(-1,2)
    return collision_segments(points,points,begin.ravel(),finish.ravel(),tracks,radius)


def preview(plan, progress, speed, horizon=2.):
    times = np.arange(0., horizon+.025, .05)
    distance = float(progress)
    points = []
    for _ in times:
        point, _, reference = plan.sample(distance)
        points.append(point)
        distance += max(.05, reference, speed if len(points) == 1 else 0.)*.05
    return np.asarray(points), times


def splice(plan, progress, bridge, join_s, limits, clearance, initial_speed):
    i = max(0, int(np.searchsorted(plan.s, progress, side='right'))-1)
    j = min(len(plan.points)-1, int(np.searchsorted(plan.s, join_s)))
    points = list(plan.points[:i])+[tuple(p) for p in bridge]+list(plan.points[j+1:])
    shift = len(bridge)-(j-i+1)
    gates = []
    for phase, index in plan.gates:
        if index < i:
            mapped = index
        elif index > j:
            mapped = index+shift
        else:
            fraction = (plan.s[index]-plan.s[i])/max(1e-9, plan.s[j]-plan.s[i])
            mapped = i+round(fraction*(len(bridge)-1))
        gates.append((phase, mapped))
    route, begin = [], 0
    for phase, index in gates:
        route.append((phase, points[begin:index+1]))
        begin = index
    if len(route) == 10 and i < plan.gates[0][1]:
        route[-1] = ('RETURN', list(reversed(route[0][1])))
        from intramural_mission.route import line
        # The measured departure may differ from home inside parking tolerance; do not reverse that tiny correction.
        returning=route[-1][1]
        near=next((n for n,p in enumerate(returning)
            if math.dist(p,plan.points[-1])<limits.get('parking_tolerance',.08)),len(returning)-1)
        route[-1]=('RETURN',returning[:near+1])
        route[-1][1].extend(line(route[-1][1][-1],plan.points[-1],limits['path_spacing'])[1:])
        left,right=route[-2][1],route[-1][1]
        # Smoothed entry/exit gates differ; reversing entry also needs a smooth lower-exit joint.
        li,rj=len(left)-1,0
        distance=0.
        while li>2 and distance<.6:
            distance+=math.dist(left[li],left[li-1]);li-=1
        distance=0.
        while rj<len(right)-3 and distance<.6:
            distance+=math.dist(right[rj],right[rj+1]);rj+=1
        def direction_curvature(a,b,d):
            u,v=np.asarray(b)-a,np.asarray(d)-b
            curvature=2*(u[0]*v[1]-u[1]*v[0])/max(1e-9,np.linalg.norm(u)*np.linalg.norm(v)*np.linalg.norm(u+v))
            tangent=(u+v)/max(1e-9,np.linalg.norm(u+v))
            return tangent,float(curvature)
        u,ka=direction_curvature(left[li-1],left[li],left[li+1])
        v,kb=direction_curvature(right[rj-1],right[rj],right[rj+1])
        joint=quintic_curve(left[li],right[rj],u,v,ka,kb,limits['path_spacing'])
        if min(clearance(joint))<=limits.get('execution_clearance',0.):
            raise ValueError('Return joint has insufficient static clearance')
        middle=len(joint)//2
        route[-2]=(route[-2][0],left[:li]+joint[:middle+1])
        route[-1]=('RETURN',joint[middle:]+right[rj+1:])
    updated = RoutePlan.build(route, limits, clearance)
    speeds = np.asarray(updated.speed).copy()
    start_s = updated.s[i]
    # Preserve the moving boundary; forward propagation replaces the implicit zero start.
    speeds[i] = min(limits['max_speed'], initial_speed)
    for k in range(i+1, len(speeds)):
        speeds[k] = min(speeds[k], math.sqrt(speeds[k-1]**2+2*limits['max_acceleration']*(updated.s[k]-updated.s[k-1])))
    return replace(updated, speed=tuple(speeds)), start_s


class DetourPlanner:
    def __init__(self, config, grid):
        from intramural_mission.planning import GridMap
        self.c, self.grid = config, grid
        self.map = GridMap(grid, config)
        self.tracker = ObstacleTracker()
        self.prediction_seconds = .025
        self.attempt = 0
        self.restore_confirm=None
        row,col=np.mgrid[:grid.info.height,:grid.info.width]
        xy=np.column_stack(((col.ravel()+.5)*self.map.res,(row.ravel()+.5)*self.map.res))
        self.grid_points=xy@np.array([[self.map.co,self.map.si],[-self.map.si,self.map.co]])+self.map.origin

    def observations(self, points, stamp, weights=None,sensor_position=None,point_times=None):
        points = np.asarray(points, dtype=float).reshape(-1, 2)
        weights = np.ones(len(points)) if weights is None else np.asarray(weights)
        if len(points):
            delta = points-self.map.origin
            col = np.floor((self.map.co*delta[:, 0]+self.map.si*delta[:, 1])/self.map.res).astype(int)
            row = np.floor((-self.map.si*delta[:, 0]+self.map.co*delta[:, 1])/self.map.res).astype(int)
            valid = (row >= 0)&(col >= 0)&(row < self.map.free.shape[0])&(col < self.map.free.shape[1])
            selected = np.zeros(len(points), bool)
            selected[valid] = self.map.distance[row[valid], col[valid]] > .10
            points = points[selected]
            weights = weights[selected]
            if point_times is not None:point_times=np.asarray(point_times)[selected]
        return [dict(track,shared_margin=self.c.get('minimum_clearance',.02))
                for track in self.tracker.update(points,stamp,weights,sensor_position,point_times)]

    def solve(self, request):
        began = time.monotonic()
        offsets=request.get('point_offsets')
        source=request.get('source_stamp',request['stamp'])
        point_times=(np.full(len(request['points']),source) if offsets is None else source+np.asarray(offsets)*1e-9)
        tracks = self.observations(request['points'], request['stamp'], request.get('weights'),request.get('sensor_position'),point_times)
        plan, progress = request['plan'], request['progress']
        pose, velocity = np.asarray(request['pose']), np.asarray(request['velocity'])
        switch_delay=max(0.,request.get('pose_age',0.))+.08
        pose=pose+velocity*switch_delay+.5*np.asarray(request.get('acceleration',[0.,0.]))*.08**2
        speed = float(np.linalg.norm(velocity))
        radius = self.c['robot_radius']+self.c.get('minimum_clearance', .02)
        samples, times = preview(plan, progress, speed)
        age = max(0., request.get('observation_age', 0.))
        for t in tracks:
            t['age'] += age
            t['prediction_age'] = age
        report = dict(stamp=request['stamp'], route_id=plan.route_id, run_id=request.get('run_id'),tracks=tracks,
                      plan=None, blocked=False, imminent=False,reason='', side=0, rejected={})
        models=self.c.get('response_models',[(self.c.get('response_delay',.2),self.c.get('response_tau',.15))])
        endpoints=[max(models,key=sum)]
        if any(t['moving'] for t in tracks):
            fastest=min(models,key=sum)
            if fastest!=endpoints[0]:endpoints.append(fastest)
        if tracks:
            from intramural_mission.response import predict_motion
            # Every candidate shares this initial braking branch; a conflict makes further search futile.
            for _,_,brakes,brake_times,_ in predict_motion(plan,self.c,pose,velocity,
                    request.get('command',velocity),request.get('acceleration',[0.,0.]),progress,horizon=0.,models=endpoints):
                if (collision(brakes[0],brake_times[0],tracks,radius)
                        or stopped_collision(brakes,brake_times,tracks,radius)):
                    return dict(report,blocked=True,imminent=True,execution_failure=True,
                        reason='BRAKING_ENVELOPE_CONFLICT',seconds=time.monotonic()-began)
        def reject(reason):
            report['rejected'][reason] = report['rejected'].get(reason, 0)+1
        def executable(proposal, start_s, horizon=2.):
            from intramural_mission.response import predict_motion
            predict_started=time.monotonic()
            predictions=predict_motion(proposal,self.c,pose,velocity,request.get('command',velocity),
                request.get('acceleration',[0.,0.]),start_s,horizon=horizon,models=endpoints)
            if horizon>=2.:self.prediction_seconds=.5*self.prediction_seconds+.5*(time.monotonic()-predict_started)
            for actual,timing,brakes,brake_times,error in predictions:
                if (error>self.c['max_tracking_error']
                        or min(self.map.clearance(brakes.reshape(-1,2),include_poles=not self.c.get('pole_clearance_from_map',False)))<=self.c.get('execution_clearance',0.)
                        or collision(actual,timing,tracks,radius)
                        or collision_segments(brakes[:,:-1].reshape(-1,2),brakes[:,1:].reshape(-1,2),
                            brake_times[:,:-1].ravel(),brake_times[:,1:].ravel(),tracks,radius)
                        or stopped_collision(brakes,brake_times,tracks,radius)):
                    return False
            return True

        def remaining_seconds(proposal, first, end=None):
            return float(np.sum((np.diff(proposal.s)/np.maximum(.05,
                (np.asarray(proposal.speed)[:-1]+np.asarray(proposal.speed)[1:])/2))[first:end]))

        restore=request.get('restore_plan')
        nominal=restore if restore is not None and restore.points==plan.points else plan
        first=int(np.searchsorted(plan.s,progress))
        last=min(len(plan.s)-1,int(np.searchsorted(plan.s,progress+max(1.,speed*3.))))
        def retimed(scale, end=last):
            speeds=np.asarray(plan.speed).copy()
            # Always scale the unmodified profile; repeated observations must not ratchet it down.
            speeds[first:end+1]=np.asarray(nominal.speed)[first:end+1]*scale
            speeds[first]=speed
            return replace(plan,speed=tuple(speeds),route_id=hashlib.sha256(
                plan.route_id.encode()+speeds.tobytes()).hexdigest()[:20])

        if not tracks or (not request.get('path_blocked') and not collision(samples, times, tracks, radius)):
            if nominal is not plan:
                previous=self.restore_confirm
                self.restore_confirm=None
                for scale in (1.,.8,.6):
                    proposal=retimed(scale)
                    if remaining_seconds(proposal,first,last)>=.9*remaining_seconds(plan,first,last):continue
                    fast,arrival=preview(proposal,progress,speed)
                    if collision(fast,arrival,tracks,radius) or not executable(proposal,progress):
                        if time.monotonic()-began>.06:break
                        continue
                    key=request.get('run_id'),plan.route_id,nominal.route_id,scale
                    count=(previous[1]+int(request['stamp']>previous[2]) if previous and previous[0]==key else 1)
                    self.restore_confirm=key,count,request['stamp']
                    if count>=2:
                        complete=np.allclose(proposal.speed[first+1:],nominal.speed[first+1:])
                        return dict(report,plan=proposal,progress=progress,side=0,rejoin_progress=progress,join_s=progress,
                            scale=scale,expected_seconds=remaining_seconds(proposal,first),
                            reason='RESTORE_READY' if complete else 'RETIME_READY',seconds=time.monotonic()-began)
                    break
                else:self.restore_confirm=None
            else:self.restore_confirm=None
            if (tracks or request.get('alternative_route',request.get('bypassing'))) and not executable(plan,progress):
                report.update(blocked=True,execution_failure=True,reason='EXECUTION_CONFLICT')
                for scale in (.8,.6,.4,.2,.1):
                    proposal=retimed(scale)
                    if executable(proposal,progress):
                        report.update(plan=proposal,progress=progress,side=0,join_s=plan.s[last],
                            rejoin_progress=plan.s[last],scale=scale,expected_seconds=remaining_seconds(proposal,first),
                            reason='RETIME_READY')
                        break
                    if time.monotonic()-began>.06:break
            if report['plan'] is not None or not report['blocked'] or not tracks:
                return dict(report, seconds=time.monotonic()-began)
        report.update(blocked=True, reason='PREDICTED_CONFLICT')
        self.restore_confirm=None
        # The braking prefix remains physical even if the desired path turns away.
        stop_horizon=stopping_horizon(self.c,speed)
        coast = np.stack((pose, pose+velocity*stop_horizon))
        if collision(coast, [0., stop_horizon], tracks, radius):
            report.update(imminent=True,reason='STOPPING_PREFIX_CONFLICT')
        if not report['imminent'] and executable(plan,progress,horizon=.3):
            # Reserve a checked continuation before search can consume the solve budget.
            report['continuation_until']=request['stamp']+age+.3
            report['execution_failure']=False
        threat=min(tracks,key=lambda t:float(np.min(obstacle_gap(
            samples-obstacle_positions(t,times)+t['center'],t)-prediction_uncertainty(t,times))))
        reference = np.asarray(plan.points)
        lengths = np.asarray(plan.s)
        phase=request.get('phase',0)
        join_phase=min(phase+1,len(plan.gates)-1)
        if len(plan.gates)==10 and join_phase in (3,7):
            cross=reference[plan.gates[join_phase][1]]
            if collision([cross,cross],[0.,0.],tracks,radius):
                # Move the blocked shared joint and reconnect inside its successor, before reaching it.
                join_phase+=1
        join_limit=plan.gates[join_phase][1]
        original=request.get('original_plan') or plan
        original_start=0 if phase==0 else original.gates[phase-1][1]
        original_end=original.gates[min(join_phase,len(original.gates)-1)][1]
        original_corridor=cKDTree(np.asarray(original.points)[original_start:original_end+1])
        start = min(len(lengths)-2, int(np.searchsorted(lengths, progress)))
        end = min(join_limit+1,len(lengths), int(np.searchsorted(lengths, progress+self.c.get('bypass_max_deviation', 1.1)*4)))
        blocked_index = start+int(np.argmin(np.linalg.norm(reference[start:end]-threat['center'], axis=1)))
        tangent = np.asarray(plan.tangent[blocked_index]); normal = np.array([-tangent[1], tangent[0]])
        distance = radius+threat['radius']+max(.12,float(prediction_uncertainty(threat,0.)))+.10
        winners = []
        if any(t['velocity_observed'] and t['moving'] for t in tracks):
            # Compare slowing on the current line with detours using the same finish-time score.
            join=min(len(lengths)-1,int(np.searchsorted(lengths,progress+max(1.,speed*2.))))
            for scale in (.8,.6,.4,.2,.1):
                proposal=retimed(scale,join)
                points,arrival=preview(proposal,progress,speed)
                if not collision(points,arrival,tracks,radius) and executable(proposal,progress):
                    winners.append((remaining_seconds(proposal,start),0.,proposal,progress,0,lengths[join],scale,lengths[join]))
                    break
                if time.monotonic()-began>.05:break
        attempt=self.attempt
        self.attempt+=1
        offsets=(.6,1.2,2.)
        offsets=offsets[attempt//2%3:]+offsets[:attempt//2%3]
        for offset in offsets:
            if time.monotonic()-began > .07:
                break
            join = min(join_limit,len(lengths)-2, int(np.searchsorted(lengths, lengths[blocked_index]+offset)))
            if join <= start+2:
                continue
            corridor_tree = cKDTree(reference[start:join+1])
            def clearance(p):
                p = np.asarray(p)
                gap = self.map.clearance(p, include_poles=not self.c.get('pole_clearance_from_map', False))
                for track in tracks:
                    # Search the observed contour; arrival-time uncertainty is checked on timed candidates.
                    uncertainty=prediction_uncertainty(track,0.)
                    gap = np.minimum(gap, obstacle_gap(p,track)-radius-uncertainty)
                corridor = np.maximum(corridor_tree.query(p)[0],original_corridor.query(p)[0])
                return np.minimum(gap, self.c.get('bypass_max_deviation', 1.1)-corridor)
            margin=self.c.get('bypass_max_deviation',1.1)+self.map.res
            region=reference[start:join+1]
            local=np.all((self.grid_points>=region.min(axis=0)-margin)&
                         (self.grid_points<=region.max(axis=0)+margin),axis=1)
            free_mask=np.zeros(len(self.grid_points),bool)
            free_mask[local]=clearance(self.grid_points[local])>self.c['path_spacing']/2
            free_mask=free_mask.reshape(self.map.free.shape)
            delta=pose-self.map.origin
            col=math.floor((self.map.co*delta[0]+self.map.si*delta[1])/self.map.res)
            row=math.floor((-self.map.si*delta[0]+self.map.co*delta[1])/self.map.res)
            # The exact continuous start can be clear while its cell center is too close.
            if 0<=row<free_mask.shape[0] and 0<=col<free_mask.shape[1] and clearance([pose])[0]>.0125:
                free_mask[row,col]=True
            for side in ((-1,1) if attempt%2==0 else (1,-1)):
                if winners and time.monotonic()-began+self.prediction_seconds+.012 > .070:
                    break
                waypoint = np.asarray(threat['center'])+normal*side*distance
                legs = []
                waypoints = [pose, waypoint-tangent*.35, waypoint+tangent*.35, reference[join]]
                for a, b in zip(waypoints, waypoints[1:]):
                    leg = entry_path(self.c, a, b, self.grid, free_mask=free_mask)
                    if leg is None:
                        reject('search')
                        break
                    legs.extend(leg if not legs else leg[1:])
                else:
                    vertices = [legs[0]]
                    for a, b, d in zip(legs, legs[1:], legs[2:]):
                        u, v = np.asarray(b)-a, np.asarray(d)-b
                        if np.dot(u, v) < .999*np.linalg.norm(u)*np.linalg.norm(v):
                            vertices.append(b)
                    vertices.append(legs[-1])
                    vertices = np.asarray(vertices)
                    ss = np.r_[0., np.cumsum(np.linalg.norm(np.diff(vertices, axis=0), axis=1))]
                    if min(np.diff(ss)) < 1e-7:
                        reject('duplicate')
                        continue
                    direction = velocity/max(speed, 1e-9) if speed > .08 else (vertices[1]-vertices[0])/np.linalg.norm(vertices[1]-vertices[0])
                    terminal = np.asarray(plan.tangent[join])
                    k = plan.curvature[join]
                    vectors=np.diff(vertices,axis=0)
                    vectors/=np.linalg.norm(vectors,axis=1)[:,None]
                    middle=vectors[:-1]+vectors[1:]
                    middle/=np.maximum(1e-9,np.linalg.norm(middle,axis=1))[:,None]
                    directions=np.vstack((direction,middle,terminal))
                    incoming=np.asarray(request.get('acceleration',[0.,0.]))
                    ka=float(np.dot(incoming,[-direction[1],direction[0]]))/max(.1,speed**2) if speed>.08 else 0.
                    for tension in (1.,.5,.25):
                        curves=[quintic_curve(a,b,directions[n]*tension,directions[n+1]*tension,
                            ka*tension if n==0 else 0.,k*tension if n==len(vertices)-2 else 0.,.025)
                            for n,(a,b) in enumerate(zip(vertices,vertices[1:]))]
                        bridge=np.asarray([p for n,curve in enumerate(curves) for p in curve[1 if n else 0:]])
                        if (min(clearance(bridge))>.0125 and
                                min(self.map.clearance(bridge,include_poles=not self.c.get('pole_clearance_from_map',False)))>=self.c.get('execution_clearance',0.)):break
                    else:
                        reject('smooth_clearance')
                        continue
                    for scale in (1., .8, .6):
                        if time.monotonic()-began>.065:
                            break
                        limits = dict(self.c)
                        try:
                            proposal, start_s = splice(plan, progress, bridge, lengths[join], limits,
                                lambda p: self.map.clearance(p, include_poles=not self.c.get('pole_clearance_from_map', False)), speed)
                        except ValueError:
                            reject('return_joint')
                            continue
                        speeds = np.asarray(proposal.speed).copy()
                        first = np.searchsorted(proposal.s, start_s)
                        last = first+len(bridge)
                        speeds[first:last] *= scale
                        speeds[first] = speed
                        proposal = replace(proposal, speed=tuple(speeds), route_id=hashlib.sha256(
                            proposal.route_id.encode()+speeds.tobytes()).hexdigest()[:20])
                        # Full-route topology is checked in addition to local gate ownership.
                        if any(winding(proposal.points, self.c[key]) != winding(plan.points, self.c[key])
                               for key in ('lower_pole', 'upper_pole')):
                            reject('topology')
                            continue
                        points, arrival = preview(proposal, start_s, speed)
                        if collision(points, arrival, tracks, radius):
                            reject('timed_collision')
                            continue
                        if not executable(proposal,start_s):
                            reject('response')
                            continue
                        duration = remaining_seconds(proposal,first)
                        rejoin = proposal.s[min(len(proposal.s)-1, first+len(bridge)-1)]
                        winners.append((duration, -float(min(clearance(bridge))), proposal, start_s, side, lengths[join], scale, rejoin))
                        break
        if winners:
            best = min(winners, key=lambda x: x[:2])
            duration, gap, proposal, start_s, side, join_s, scale, rejoin = best
            report.update(plan=proposal, progress=start_s, side=side, join_s=join_s,
                          rejoin_progress=rejoin,
                          rejoin_phase=int(np.searchsorted([plan.s[i] for _,i in plan.gates],join_s)),
                          scale=scale, expected_seconds=float(duration), reason='DETOUR_READY' if side else 'YIELD_READY',
                          start_pose=pose.tolist(), source_progress=progress)
        report['seconds'] = time.monotonic()-began
        return report


def _work(config, grid, incoming, outgoing, ready):
    planner = DetourPlanner(config, grid)
    ready.set()
    while True:
        request = incoming.get()
        if request is None:
            return
        started=time.monotonic()
        try:
            result = planner.solve(request)
            if result['plan'] is not None:result['plan_json']=json.dumps(asdict(result['plan']))
        except (ValueError, IndexError, FloatingPointError) as error:
            result = dict(stamp=request['stamp'], route_id=request['plan'].route_id,
                          run_id=request.get('run_id'),
                          plan=None, blocked=True, reason=str(error), tracks=[], seconds=0.)
        result['seconds']=time.monotonic()-started
        outgoing.put(result)


class PlanningWorker:
    def __init__(self, config, grid):
        context = mp.get_context('spawn')
        self.incoming, self.outgoing = context.Queue(1), context.Queue(1)
        self.ready=context.Event()
        self.process = context.Process(target=_work, args=(config, grid, self.incoming, self.outgoing,self.ready), daemon=True)
        self.process.start()
        self.busy = False
        self.pending = None

    def submit(self, request):
        self.pending = request
        self.dispatch()

    def dispatch(self):
        if not self.busy and self.pending is not None:
            self.incoming.put_nowait(self.pending)
            self.pending = None
            self.busy = True

    def poll(self):
        if not self.process.is_alive():
            raise RuntimeError('Avoidance worker exited')
        try:
            result = self.outgoing.get_nowait()
        except queue.Empty:
            return None
        self.busy = False
        self.dispatch()
        return result

    def close(self):
        if self.process.is_alive():
            # The worker owns no device; terminating it must never delay a final zero.
            self.process.terminate()
            self.process.join(timeout=1.)
            if self.process.is_alive():
                self.process.kill()
                self.process.join(timeout=1.)
        self.incoming.cancel_join_thread()
        self.outgoing.cancel_join_thread()
        self.incoming.close()
        self.outgoing.close()
