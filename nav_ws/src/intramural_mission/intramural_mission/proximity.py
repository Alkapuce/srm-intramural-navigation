"""Short-range raw-LiDAR stop independent of global localization and costmap."""

import math
import numpy as np
from scipy.spatial import cKDTree


def outside_self(points, boxes):
    keep=np.ones(len(points),dtype=bool)
    for box in boxes:
        box=np.asarray(box,dtype=float)
        if box.shape!=(2,3) or not np.isfinite(box).all() or np.any(box[0]>box[1]):
            raise ValueError('Self-filter boxes require finite ordered XYZ bounds')
        keep &= ~np.all((points>=box[0])&(points<=box[1]),axis=1)
    return keep


def surface_support(points, min_points):
    """Reject isolated scan returns; keep compact surfaces without a temporal delay."""
    if min_points<=1:return np.ones(len(points),dtype=bool)
    if not len(points):return np.empty(0,dtype=bool)
    return cKDTree(points).query_ball_point(points,.08,return_length=True)>=min_points


def lidar_to_body(points, pose):
    """Apply the configured sensor-to-body rigid transform once."""
    px, py, pz, roll, pitch, yaw = pose
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch),
                             math.sin(pitch), math.cos(yaw), math.sin(yaw))
    rotation = np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                         [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                         [-sp, cp*sr, cp*cr]])
    return np.asarray(points) @ rotation.T + [px, py, pz]


def obstacle_near(points, direction, radial=.85, forward=1.15, lateral=.5, immediate=None, min_points=1):
    """Input XY is in the body frame; ground and sensor no-returns are removed."""
    if len(points) == 0:
        return False
    xy = np.asarray(points)[:, :2]
    # Gazebo encodes some no-return rays as finite (0,0,0), inside the sensor.
    xy = xy[np.sum(xy * xy, axis=1) > .04]
    if len(xy) == 0:
        return False
    immediate = radial if immediate is None else immediate
    hits = np.sum(xy * xy, axis=1) < immediate * immediate
    if np.count_nonzero(hits) >= min_points:
        return True
    dx, dy = direction
    norm = math.hypot(dx, dy)
    if norm < .05:
        return False
    dx, dy = dx / norm, dy / norm
    along = xy[:, 0] * dx + xy[:, 1] * dy
    across = np.abs(xy[:, 0] * dy - xy[:, 1] * dx)
    # Rounded swept footprint avoids stopping for points outside the braking path.
    nearest = np.clip(along, 0, max(0, forward-radial))
    hits |= ((along-nearest)**2+across**2 < radial**2) & (across < lateral) & (along >= 0)
    return bool(np.count_nonzero(hits) >= min_points)


def obstacle_on_path(points, path, reach, radius, min_points=1):
    """Sweep the physical footprint along the ordered future path in body XY."""
    xy, route = np.asarray(points), np.asarray(path)
    if not xy.size:
        return False
    xy = xy[:, :2]
    hits = np.linalg.norm(xy, axis=1) < radius
    if np.count_nonzero(hits) >= min_points:
        return True
    if len(route) < 2:
        return False
    # A point farther than the entire sweep cannot intersect any of its segments.
    xy = xy[np.sum(xy*xy, axis=1) < (max(0., reach)+radius)**2]
    if not len(xy):
        return False
    hits = np.sum(xy*xy, axis=1) < radius**2
    if np.linalg.norm(route[0]) < 1e-7:
        # An executable coast prefix is already ordered; projecting again can discard it.
        future = route
    else:
        delta = np.diff(route, axis=0)
        length2 = np.sum(delta*delta, axis=1)
        fraction = np.clip(-np.sum(route[:-1]*delta, axis=1)/np.maximum(length2, 1e-12), 0., 1.)
        projections = route[:-1] + fraction[:, None]*delta
        # A long braking horizon can revisit the crossing; never project onto that later visit.
        distance=np.linalg.norm(projections,axis=1)
        index=0
        while index+1<len(distance) and distance[index+1]<=distance[index]+1e-9:
            index+=1
        future = np.vstack(([0., 0.], projections[index], route[index+1:]))
    delta=np.diff(future,axis=0)
    lengths=np.linalg.norm(delta,axis=1)
    starts=np.r_[0.,np.cumsum(lengths)[:-1]]
    selected=(lengths>1e-9)&(starts<reach)
    a=future[:-1][selected]
    if not len(a):return False
    d=delta[selected]*np.minimum(1.,(reach-starts[selected])/lengths[selected])[:,None]
    low=np.min(np.minimum(a,a+d),axis=0)-radius
    high=np.max(np.maximum(a,a+d),axis=0)+radius
    xy=xy[np.all(xy>=low,axis=1)&np.all(xy<=high,axis=1)]
    if not len(xy):return False
    # Short braking sweeps fit one small array operation; avoid per-segment Python overhead.
    if len(xy)*len(a)<=65536:
        local=xy[:,None,:]-a
        fraction=np.clip(np.sum(local*d,axis=2)/np.sum(d*d,axis=1),0.,1.)
        residual=local-fraction[...,None]*d
        return bool(np.count_nonzero(np.any(np.sum(residual*residual,axis=2)<radius**2,axis=1))>=min_points)
    hits=np.sum(xy*xy,axis=1)<radius**2
    travelled = 0.
    tree=cKDTree(xy) if len(xy)>=256 else None
    for a, b in zip(future, future[1:]):
        length = np.linalg.norm(b-a)
        if length < 1e-9:
            continue
        if travelled >= reach:
            break
        b = a + (b-a)*min(1., (reach-travelled)/length)
        d = b-a
        if tree is not None:
            possible=np.asarray(tree.query_ball_point((a+b)/2,radius+np.linalg.norm(d)/2),dtype=int)
            possible=possible[~hits[possible]]
        else:
            possible = np.flatnonzero(~hits & np.all(xy >= np.minimum(a,b)-radius, axis=1)
                                      & np.all(xy <= np.maximum(a,b)+radius, axis=1))
        if not len(possible):
            travelled+=length
            continue
        local = xy[possible]-a
        projection = np.clip((local@d)/np.dot(d,d), 0., 1.)
        residual = local-projection[:, None]*d
        hits[possible] |= np.sum(residual*residual, axis=1) < radius**2
        if np.count_nonzero(hits) >= min_points:
            return True
        travelled += length
    return False
