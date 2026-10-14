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


def obstacle_near(points, direction, radial=.85, forward=1.15, lateral=.5):
    """Input XY is in the body frame; ground and sensor no-returns are removed."""
    if len(points) == 0:
        return False
    xy = np.asarray(points)[:, :2]
    # Gazebo encodes some no-return rays as finite (0,0,0), inside the sensor.
    xy = xy[np.sum(xy * xy, axis=1) > .04]
    if len(xy) == 0:
        return False
    if np.any(np.sum(xy * xy, axis=1) < radial * radial):
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
    return bool(np.any(((along-nearest)**2+across**2 < radial**2) & (across < lateral)))
