"""Short-range raw-LiDAR stop independent of global localization and costmap."""

import math
import numpy as np


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
    delta = np.diff(route, axis=0)
    length2 = np.sum(delta*delta, axis=1)
    fraction = np.clip(-np.sum(route[:-1]*delta, axis=1)/np.maximum(length2, 1e-12), 0., 1.)
    projections = route[:-1] + fraction[:, None]*delta
    index = int(np.argmin(np.linalg.norm(projections, axis=1)))
    future = np.vstack(([0., 0.], projections[index], route[index+1:]))
    travelled = 0.
    for a, b in zip(future, future[1:]):
        length = np.linalg.norm(b-a)
        if length < 1e-9:
            continue
        if travelled >= reach:
            break
        b = a + (b-a)*min(1., (reach-travelled)/length)
        d = b-a
        projection = np.clip(((xy-a)@d)/np.dot(d,d), 0., 1.)
        hits |= np.linalg.norm(xy-a-projection[:, None]*d, axis=1) < radius
        if np.count_nonzero(hits) >= min_points:
            return True
        travelled += length
    return False
