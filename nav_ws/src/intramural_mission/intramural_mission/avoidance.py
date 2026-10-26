"""Bounded, ordered detours; the raw-cloud guard never depends on this planner."""
from dataclasses import replace
import math
import multiprocessing as mp
import queue
import time
import numpy as np
from scipy.ndimage import label
from scipy.optimize import linear_sum_assignment
from scipy.interpolate import make_interp_spline
from scipy.spatial import cKDTree
from intramural_mission.bypass import entry_path, winding
from intramural_mission.route import line
from intramural_mission.trajectory import RoutePlan
from intramural_mission.safety import response_horizon


class ObstacleTracker:
    def __init__(self):
        self.tracks, self.stamp, self.serial = [], -math.inf, 0

    def update(self, points, stamp, weights=None):
        points = np.asarray(points, dtype=float).reshape(-1, 2)
        weights = np.ones(len(points)) if weights is None else np.asarray(weights)
        if not math.isfinite(stamp) or not np.isfinite(points).all():
            raise ValueError('Finite timestamped obstacle points required')
        if weights.shape != (len(points),) or not np.isfinite(weights).all() or np.any(weights <= 0):
            raise ValueError('Positive observation support counts required')
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
            groups, count = label(occupied, np.ones((3, 3)))
            ids = groups[tuple(cells.T)]
            for i in range(1, count+1):
                cluster = points[ids == i]
                if sum(weights[ids == i]) >= 3:
                    center = (cluster.min(axis=0)+cluster.max(axis=0))/2
                    clusters.append((center, float(max(np.linalg.norm(cluster-center, axis=1)))+.025))
        self.tracks = [t for t in self.tracks if stamp-t['seen'] <= .6]
        for t in self.tracks:
            dt = stamp-t['stamp']
            f = np.eye(4); f[:2, 2:] = np.eye(2)*dt
            g = np.vstack((np.eye(2)*dt*dt/2, np.eye(2)*dt))
            t['x'] = f@t['x']
            t['p'] = f@t['p']@f.T + .3**2*g@g.T
            t['stamp'] = stamp
        used = set()
        if self.tracks and clusters:
            costs = np.array([[np.linalg.norm(t['x'][:2]-c[0]) for c in clusters] for t in self.tracks])
            rows, cols = linear_sum_assignment(costs)
            for i, j in zip(rows, cols):
                t = self.tracks[i]
                if costs[i, j] > .45 + 2*math.sqrt(max(np.diag(t['p'])[:2])):
                    continue
                innovation = t['p'][:2, :2]+np.eye(2)*.02**2
                k = np.linalg.solve(innovation, t['p'][:, :2].T).T
                t['x'] += k@(clusters[j][0]-t['x'][:2])
                h = np.eye(4); h[:, :2] -= k
                t['p'] = h@t['p']@h.T + k@(np.eye(2)*.02**2)@k.T
                t.update(radius=clusters[j][1], seen=stamp, observations=t['observations']+1)
                used.add(j)
        for j, (center, radius) in enumerate(clusters):
            if j not in used:
                self.serial += 1
                self.tracks.append(dict(id=self.serial, x=np.r_[center, 0., 0.],
                    p=np.diag([.02**2]*2+[.5**2]*2), radius=radius, stamp=stamp, seen=stamp, observations=1))
        return self.snapshot(stamp)

    def snapshot(self, stamp):
        return [dict(id=t['id'], center=t['x'][:2].tolist(), velocity=t['x'][2:].tolist(),
                     covariance=t['p'].tolist(), radius=t['radius'], age=max(0., stamp-t['seen']),
                     moving=bool(t['observations']<3 or np.linalg.norm(t['x'][2:])>.08))
                for t in self.tracks]


def collision(points, times, tracks, radius):
    """Sweep each interval against relative obstacle motion, including uncertain position."""
    p, times = np.asarray(points), np.asarray(times)
    for track in tracks:
        velocity = np.asarray(track['velocity'])
        age = track.get('age', 0.)
        relative = p-np.asarray(track['center'])-(times+age)[:, None]*velocity
        a, d = relative[:-1], np.diff(relative, axis=0)
        f = np.clip(-np.sum(a*d, axis=1)/np.maximum(1e-12, np.sum(d*d, axis=1)), 0., 1.)
        covariance = np.asarray(track['covariance'])
        horizon = times[1:]+age
        variance = max(np.diag(covariance)[:2]) + horizon**2*max(np.diag(covariance)[2:])
        uncertainty = 2*np.sqrt(np.maximum(0., variance)) + .025*horizon**2
        if not track.get('moving', True):
            uncertainty = 2*math.sqrt(max(np.diag(covariance)[:2]))+.02*horizon
        if np.any(np.linalg.norm(a+f[:, None]*d, axis=1) <= radius+track['radius']+uncertainty):
            return True
    return False


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
    end = i+len(bridge)-1
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

    def observations(self, points, stamp, weights=None):
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
        return self.tracker.update(points, stamp, weights)

    def solve(self, request):
        began = time.monotonic()
        tracks = self.observations(request['points'], request['stamp'], request.get('weights'))
        plan, progress = request['plan'], request['progress']
        pose, velocity = np.asarray(request['pose']), np.asarray(request['velocity'])
        speed = float(np.linalg.norm(velocity))
        radius = self.c['robot_radius']+self.c.get('minimum_clearance', .02)
        samples, times = preview(plan, progress, speed)
        age = max(0., request.get('observation_age', 0.))
        for t in tracks:
            t['age'] += age
        report = dict(stamp=request['stamp'], route_id=plan.route_id, tracks=tracks,
                      plan=None, blocked=False, reason='', side=0, rejected={})
        def reject(reason):
            report['rejected'][reason] = report['rejected'].get(reason, 0)+1
        if not collision(samples, times, tracks, radius):
            return dict(report, seconds=time.monotonic()-began)
        report.update(blocked=True, reason='PREDICTED_CONFLICT')
        # The braking prefix remains physical even if the desired path turns away.
        coast = np.stack((pose, pose+velocity*response_horizon(self.c)))
        if collision(coast, [0., response_horizon(self.c)], tracks, radius):
            report['reason'] = 'STOPPING_PREFIX_CONFLICT'
        threat = min(tracks, key=lambda t: min(np.linalg.norm(samples-np.asarray(t['center']), axis=1)))
        reference = np.asarray(plan.points)
        lengths = np.asarray(plan.s)
        start = min(len(lengths)-2, int(np.searchsorted(lengths, progress)))
        end = min(len(lengths), int(np.searchsorted(lengths, progress+self.c.get('bypass_max_deviation', 1.1)*4)))
        blocked_index = start+int(np.argmin(np.linalg.norm(reference[start:end]-threat['center'], axis=1)))
        tangent = np.asarray(plan.tangent[blocked_index]); normal = np.array([-tangent[1], tangent[0]])
        covariance = np.asarray(threat['covariance'])
        distance = radius+threat['radius']+max(.12,
            2*math.sqrt(max(np.diag(covariance)[:2])+4*max(np.diag(covariance)[2:]))+.10)
        winners = []
        for offset in (.6, 1.2, 2.):
            join = min(len(lengths)-2, int(np.searchsorted(lengths, lengths[blocked_index]+offset)))
            if join <= start+2:
                continue
            for side in (-1, 1):
                if time.monotonic()-began > .080:
                    break
                waypoint = np.asarray(threat['center'])+normal*side*distance
                corridor_tree = cKDTree(reference[start:join+1])
                # Future moving occupants are checked in time; current geometry seeds the search.
                def clearance(p):
                    p = np.asarray(p)
                    gap = self.map.clearance(p, include_poles=not self.c.get('pole_clearance_from_map', False))
                    for track in tracks:
                        gap = np.minimum(gap, np.linalg.norm(p-track['center'], axis=1)-radius-track['radius'])
                    # Restrict search to this occurrence of the ordered corridor.
                    corridor = corridor_tree.query(p)[0]
                    return np.minimum(gap, self.c.get('bypass_max_deviation', 1.1)-corridor)
                legs = []
                waypoints = [pose, waypoint-tangent*.35, waypoint+tangent*.35, reference[join]]
                for a, b in zip(waypoints, waypoints[1:]):
                    leg = entry_path(self.c, a, b, self.grid, clearance)
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
                    spline = make_interp_spline(ss, vertices, k=5, bc_type=(
                        [(1, direction), (2, np.asarray(request.get('acceleration', [0., 0.]))/max(.1, speed**2))],
                        [(1, terminal), (2, k*np.array([-terminal[1], terminal[0]]))]))
                    bridge = spline(np.linspace(0., ss[-1], max(8, math.ceil(ss[-1]/.025))))
                    if min(clearance(bridge)) <= .0125:
                        reject('smooth_clearance')
                        continue
                    for scale in (1., .8, .6):
                        limits = dict(self.c)
                        proposal, start_s = splice(plan, progress, bridge, lengths[join], limits,
                            lambda p: self.map.clearance(p, include_poles=not self.c.get('pole_clearance_from_map', False)), speed)
                        speeds = np.asarray(proposal.speed).copy()
                        first = np.searchsorted(proposal.s, start_s)
                        last = first+len(bridge)
                        speeds[first:last] *= scale
                        proposal = replace(proposal, speed=tuple(speeds))
                        # Full-route topology is checked in addition to local gate ownership.
                        if any(winding(proposal.points, self.c[key]) != winding(plan.points, self.c[key])
                               for key in ('lower_pole', 'upper_pole')):
                            reject('topology')
                            continue
                        points, arrival = preview(proposal, start_s, speed, 3.)
                        if collision(points, arrival, tracks, radius):
                            reject('timed_collision')
                            continue
                        from intramural_mission.response import predict_motion
                        predictions = predict_motion(proposal, self.c, pose, velocity,
                            request.get('command', velocity), request.get('acceleration', [0., 0.]), start_s)
                        valid = True
                        for actual, timing, tails, error in predictions:
                            # A stop tail is swept as well as the normally executed motion.
                            for f in (0., .5, 1.):
                                swept = actual+f*(tails-actual)
                                if (error > self.c['max_tracking_error']
                                    or min(self.map.clearance(swept, include_poles=not self.c.get('pole_clearance_from_map', False))) <= 0
                                    or collision(swept, timing+f*response_horizon(self.c), tracks, radius)):
                                    valid = False
                                    break
                            if not valid:
                                break
                        if not valid:
                            reject('response')
                            continue
                        duration = np.sum(np.linalg.norm(np.diff(bridge, axis=0), axis=1)/np.maximum(.05, np.asarray(proposal.speed)[first+1:last]))
                        rejoin = proposal.s[min(len(proposal.s)-1, first+len(bridge)-1)]
                        winners.append((duration, -float(min(clearance(bridge))), proposal, start_s, side, lengths[join], scale, rejoin))
                        break
        if winners:
            best = min(winners, key=lambda x: x[:2])
            duration, gap, proposal, start_s, side, join_s, scale, rejoin = best
            report.update(plan=proposal, progress=start_s, side=side, join_s=join_s,
                          rejoin_progress=rejoin,
                          scale=scale, expected_seconds=float(duration), reason='DETOUR_READY',
                          start_pose=pose.tolist(), source_progress=progress)
        report['seconds'] = time.monotonic()-began
        return report


def _work(config, grid, incoming, outgoing):
    planner = DetourPlanner(config, grid)
    while True:
        request = incoming.get()
        if request is None:
            return
        try:
            result = planner.solve(request)
        except (ValueError, IndexError, FloatingPointError) as error:
            result = dict(stamp=request['stamp'], route_id=request['plan'].route_id,
                          plan=None, blocked=True, reason=str(error), tracks=[], seconds=0.)
        outgoing.put(result)


class PlanningWorker:
    def __init__(self, config, grid):
        context = mp.get_context('spawn')
        self.incoming, self.outgoing = context.Queue(1), context.Queue(1)
        self.process = context.Process(target=_work, args=(config, grid, self.incoming, self.outgoing), daemon=True)
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
        self.incoming.close()
        self.outgoing.close()
