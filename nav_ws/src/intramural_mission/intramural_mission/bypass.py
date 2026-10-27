"""Bounded costmap detours retaining ordered segment gates and pole winding."""

import heapq
import math
import time
import numpy as np
from scipy.ndimage import distance_transform_edt
from intramural_mission.route import line, clearance, segments, primitives


def winding(points, pole):
    angles = [math.atan2(y-pole[1], x-pole[0]) for x, y in points]
    return round(sum(math.remainder(b-a, 2*math.pi) for a, b in zip(angles, angles[1:])) / (2*math.pi))


def entry_path(c, start, goal, grid, clearance=None, free_mask=None, deadline=None, continuous_start=False):
    """Connect a localized start to the first gate on the known static map."""
    if deadline is not None and time.monotonic()>=deadline:
        return None
    if grid is None or grid.header.frame_id != "map":
        return None
    res, width, height = grid.info.resolution, grid.info.width, grid.info.height
    if res <= 0 or width * height != len(grid.data):
        raise ValueError("Invalid occupancy grid")
    free = np.asarray(grid.data).reshape(height, width) == 0
    if free_mask is None:
        distance = distance_transform_edt(np.pad(free, 1))[1:-1, 1:-1] * res
        # Same footprint and cell/sampling uncertainty as field route verification.
        free &= distance > c['robot_radius'] + res / math.sqrt(2) + c['path_spacing'] / 2
    else:
        if free_mask.shape != free.shape:
            raise ValueError('Prepared traversability must match the map')
        free &= free_mask
    origin = grid.info.origin
    q = origin.orientation
    yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
    co, so = math.cos(yaw), math.sin(yaw)

    def cell(p):
        x, y = p[0]-origin.position.x, p[1]-origin.position.y
        return math.floor((-so*x+co*y)/res), math.floor((co*x+so*y)/res)

    def point(v):
        x, y = (v[1]+.5)*res, (v[0]+.5)*res
        return [origin.position.x+co*x-so*y, origin.position.y+so*x+co*y]

    def valid(v):
        return 0 <= v[0] < height and 0 <= v[1] < width and free[v]

    # Keep the connector consistent with the mission's analytic clearance check.
    yy, xx = np.mgrid[:height, :width]
    x, y = (xx+.5)*res, (yy+.5)*res
    wx, wy = origin.position.x+co*x-so*y, origin.position.y+so*x+co*y
    if clearance is not None:
        free &= clearance(np.column_stack((wx.ravel(), wy.ravel()))).reshape(height, width) > c['path_spacing']/2
    xmin, xmax, ymin, ymax = c['bounds']
    margin = c['robot_radius'] + c.get('minimum_clearance', .05)
    free &= np.minimum.reduce((wx-xmin, xmax-wx, wy-ymin, ymax-wy)) >= margin
    if not c.get('pole_clearance_from_map', False):
        for pole in ('lower_pole', 'upper_pole'):
            free &= np.hypot(wx-c[pole][0], wy-c[pole][1]) >= c['pole_radius']+margin
    travel_cost=(1+.10/np.maximum(.05,distance_transform_edt(free)*res)
                 if free_mask is not None else np.ones_like(free,dtype=float))
    source, target = cell(start), cell(goal)
    if not valid(source) or not valid(target):
        return None
    direct=line(start,goal,min(res/2,c['path_spacing']))
    if free_mask is None and all(valid(cell(p)) for p in direct):
        return line(start,goal,c['path_spacing'])
    queue, costs, previous = [(math.dist(source, target), 0., source)], {source: 0.}, {}
    while queue:
        if deadline is not None and time.monotonic()>=deadline:
            return None
        _, cost, v = heapq.heappop(queue)
        if cost != costs[v]:
            continue
        if v == target:
            cells = [v]
            while cells[-1] != source:
                cells.append(previous[cells[-1]])
            path = [list(start)] + [point(p) for p in reversed(cells) if not continuous_start or p != source] + [list(goal)]
            if free_mask is not None:
                def sampled_cost(points):
                    p=np.asarray(points);delta=p-[origin.position.x,origin.position.y]
                    row=np.floor((-so*delta[:,0]+co*delta[:,1])/res).astype(int)
                    col=np.floor((co*delta[:,0]+so*delta[:,1])/res).astype(int)
                    weights=travel_cost[row,col]
                    return np.r_[0.,np.cumsum(np.linalg.norm(np.diff(p,axis=0),axis=1)*(weights[:-1]+weights[1:])/2)]
                cumulative=sampled_cost(path)
                indices=[0]
                for n,(a,b,d) in enumerate(zip(path,path[1:],path[2:]),1):
                    u,v=np.asarray(b)-a,np.asarray(d)-b
                    if np.dot(u,v)<.999*np.linalg.norm(u)*np.linalg.norm(v):indices.append(n)
                indices.append(len(path)-1)
                corners=[path[n] for n in indices];path=[corners[0]];i=0
                while i<len(corners)-1:
                    j=len(corners)-1
                    while j>i+1:
                        if deadline is not None and time.monotonic()>=deadline:
                            return None
                        shortcut=line(corners[i],corners[j],res/2)
                        if (all(valid(cell(p)) for p in shortcut) and
                                sampled_cost(shortcut)[-1]<=1.01*(cumulative[indices[j]]-cumulative[indices[i]])):break
                        j-=1
                    path.append(corners[j]);i=j
            return [p for a, b in zip(path, path[1:]) for p in line(a, b, c['path_spacing'])[:-1]] + [list(goal)]
        for dy, dx in ((-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)):
            nxt = v[0]+dy, v[1]+dx
            if not valid(nxt) or (dx and dy and (not valid((v[0]+dy,v[1])) or not valid((v[0],v[1]+dx)))):
                continue
            new = cost + math.hypot(dx, dy)*travel_cost[nxt]
            if new < costs.get(nxt, math.inf):
                costs[nxt], previous[nxt] = new, v
                heapq.heappush(queue, (new+math.dist(nxt,target), new, nxt))
    return None


def plan(c, route, index, progress, pose, obstacles):
    """Return replacement current/next paths, or None when no bounded route exists.

    Only measured lethal cells are obstacles; analytic walls/poles come from the
    same prior map as navigation. Live scan and costmap guards remain authoritative.
    """
    res = c['map_resolution']
    xmin, xmax, ymin, ymax = c['bounds']
    width, height = math.ceil((xmax-xmin)/res), math.ceil((ymax-ymin)/res)
    yy, xx = np.mgrid[:height, :width]
    xy = np.stack((xmin+(xx+.5)*res, ymin+(yy+.5)*res), axis=-1)
    occupied = np.zeros((height, width), dtype=bool)
    reference_route = segments(c) if len(route) == 10 else primitives(c)
    if len(route) == 10:
        reference_route[-1] = ('RETURN', list(reversed(route[0][1])))
    for x, y in obstacles:
        col, row = math.floor((x-xmin)/res), math.floor((y-ymin)/res)
        if 0 <= row < height and 0 <= col < width:
            occupied[row, col] = True
    margin = c['robot_radius'] + .20
    free = (distance_transform_edt(~occupied)*res if occupied.any() else np.full((height,width),np.inf)) > margin + res*math.sqrt(2)/2
    free &= np.minimum.reduce((xy[:,:,0]-xmin, xmax-xy[:,:,0], xy[:,:,1]-ymin, ymax-xy[:,:,1])) > margin
    for key in ('lower_pole', 'upper_pole'):
        free &= np.linalg.norm(xy-np.array(c[key]), axis=2) > c['pole_radius']+margin

    def cell(p):
        return (math.floor((p[1]-ymin)/res), math.floor((p[0]-xmin)/res))

    def point(v):
        return xy[v].tolist()

    def valid(v, mask):
        return 0 <= v[0] < height and 0 <= v[1] < width and mask[v]

    def search(start, goal, phase, reference):
        # A narrow corridor plus the phase's angular sector prevents shortcuts.
        distance = np.full((height, width), np.inf)
        for p in reference:
            distance = np.minimum(distance, np.linalg.norm(xy-np.array(p), axis=2))
        mask = free & (distance <= c['bypass_max_deviation'])
        if phase in ('LOWER', 'LOWER_EXIT', 'UPPER'):
            center = c['upper_pole' if phase == 'UPPER' else 'lower_pole']
            a = math.atan2(start[1]-center[1], start[0]-center[0])
            b = a + math.remainder(math.atan2(goal[1]-center[1], goal[0]-center[0])-a, 2*math.pi)
            angle = a + np.arctan2(np.sin(np.arctan2(xy[:,:,1]-center[1], xy[:,:,0]-center[0])-a), np.cos(np.arctan2(xy[:,:,1]-center[1], xy[:,:,0]-center[0])-a))
            mask &= (angle >= min(a,b)-.08) & (angle <= max(a,b)+.08)
        source, target = cell(start), cell(goal)
        # Newly visible cells can put a stopped robot just inside planning inflation.
        # Permit its current cell only when the physical footprint is still clear.
        if (0 <= source[0] < height and 0 <= source[1] < width
                and min(clearance(c,start)) > .12
                and all(math.dist(start,p) > c['robot_radius']+.12 for p in obstacles)):
            mask[source] = True
        if not valid(source, mask) or not valid(target, mask):
            return None
        queue = [(math.dist(source,target), 0., source)]
        costs, previous = {source: 0.}, {}
        while queue:
            _, cost, v = heapq.heappop(queue)
            if cost != costs[v]:
                continue
            if v == target:
                path = [v]
                while path[-1] != source:
                    path.append(previous[path[-1]])
                path = [start] + [point(p) for p in reversed(path)] + [goal]
                return [q for a,b in zip(path,path[1:]) for q in line(a,b,c['path_spacing'])[:-1]] + [goal]
            for dy, dx in ((-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)):
                nxt = v[0]+dy, v[1]+dx
                if not valid(nxt, mask) or (dx and dy and (not valid((v[0]+dy,v[1]),mask) or not valid((v[0],v[1]+dx),mask))):
                    continue
                # Prefer staying near the original route, with room around obstacles.
                step = math.hypot(dx,dy)*(1 + 2*distance[nxt])
                new = cost+step
                if new < costs.get(nxt, math.inf):
                    costs[nxt], previous[nxt] = new, v
                    heapq.heappush(queue,(new+math.dist(nxt,target),new,nxt))
        return None

    phase, original = route[index]
    endpoint = original[-1]
    candidates = [endpoint]
    if not valid(cell(endpoint), free):
        candidates = sorted((point((r,col)) for r,col in zip(*np.where(free))
                             if math.dist(point((r,col)),endpoint) <= c['bypass_max_deviation']),
                            key=lambda p: math.dist(p,endpoint))
        # Try a small spread of free gates rather than running A* for every cell.
        chosen = []
        for p in candidates:
            if all(math.dist(p,q) >= .2 for q in chosen):
                chosen.append(p)
            if len(chosen) == 16:
                break
        candidates = chosen
    baseline = [p for _, path in route for p in path]
    expected = [winding(baseline,c[k]) for k in ('lower_pole','upper_pole')]
    for goal in candidates:
        path = search(pose, goal, phase, reference_route[index][1])
        if path is None:
            continue
        replacement = [(phase, path)]
        if math.dist(goal,endpoint) > 1e-6:
            if index+1 == len(route):
                continue  # Parking is never relocated.
            name, nxt = route[index+1]
            following = search(goal,nxt[-1],name,reference_route[index+1][1])
            if following is None:
                continue
            replacement.append((name,following))
        proposal = route[:index]+[(phase,original[:progress]+path)]+replacement[1:]+route[index+len(replacement):]
        points = [p for _, path in proposal for p in path]
        if [winding(points,c[k]) for k in ('lower_pole','upper_pole')] == expected:
            return replacement
    return None
