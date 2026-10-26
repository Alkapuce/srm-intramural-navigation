"""Executable latency prefix for raw-cloud sweeps, independent of ROS transports."""

import math
import numpy as np


def response_horizon(limits):
    """Bound coasting displacement with delay plus the first-order stopping tail."""
    recorded = list(limits.get('response_models', []))
    stress = list(limits.get('response_stress_models', []))
    if any(len(m) != 2 or not all(math.isfinite(v) and v >= 0 for v in m) for m in recorded+stress):
        raise ValueError('Finite nonnegative delay/tau pairs required')
    tau = limits.get('response_tau', float(np.median([m[1] for m in recorded])) if recorded else .15)
    models = [(limits.get('response_delay', .2), tau)] + recorded
    if limits.get('require_stress_models', True):models += stress
    if any(not all(math.isfinite(v) and v >= 0 for v in m) for m in models):
        raise ValueError('Finite nonnegative delay/tau pairs required')
    return max(delay + tau for delay, tau in models)


def collision_reach(limits, speed, sensor_age=0.):
    """Cover stopping plus the observed sensor pipeline and the 40/100 Hz command ticks."""
    horizon = response_horizon(limits) + max(limits.get('pose_update_period', .1), sensor_age) + .035
    return max(.35, speed * horizon) + limits.get('minimum_clearance', .02)


def compensate_motion(points, stamps, history, reference_time):
    """Put sampled body XY into one current body frame using source-time odometry."""
    points = np.asarray(points, dtype=float)
    history = np.asarray(history, dtype=float)
    stamps = np.broadcast_to(np.asarray(stamps, dtype=float), (len(points),))
    if history.ndim != 2 or history.shape[1] != 4 or len(history) < 2:
        raise ValueError('ODOMETRY_HISTORY_WARMUP')
    times = history[:, 0]
    if (np.any(np.diff(times) <= 0) or reference_time-times[-1] > .25
            or (len(stamps) and (stamps.min() < times[0] or stamps.max() > reference_time+.01))):
        raise ValueError('POINT_TIME_OUTSIDE_ODOMETRY')
    poses = history[:, 1:].copy()
    poses[:, 2] = np.unwrap(poses[:, 2])
    def sample(t):
        p = np.stack([np.interp(t, times, poses[:, k]) for k in range(3)], axis=-1)
        # Only extrapolate the short unpublished tail of the latest 10 Hz odometry.
        return p + np.maximum(0., np.asarray(t)-times[-1])[..., None] * ((poses[-1]-poses[-2])/(times[-1]-times[-2]))
    source, target = sample(stamps), sample(reference_time)
    co, si = np.cos(source[:, 2]), np.sin(source[:, 2])
    world = np.column_stack((co*points[:, 0]-si*points[:, 1], si*points[:, 0]+co*points[:, 1])) + source[:, :2]
    delta = world-target[:2]
    co, si = math.cos(target[2]), math.sin(target[2])
    return np.column_stack((co*delta[:, 0]+si*delta[:, 1], -si*delta[:, 0]+co*delta[:, 1]))


def map_motion_pose(pose, source_stamp, history, reference_time):
    """Advance the map pose with the same odometry used to compensate the scan."""
    history = np.asarray(history)
    yaw = np.unwrap(history[:, 3])
    dt = history[-1, 0]-history[-2, 0]
    source_yaw = float(np.interp(source_stamp, history[:, 0], yaw))
    current_yaw = yaw[-1]+max(0., reference_time-history[-1, 0])*(yaw[-1]-yaw[-2])/dt
    angle = pose[2]+current_yaw-source_yaw
    origin = compensate_motion([[0., 0.]], [source_stamp], history, reference_time)[0]
    co, si = math.cos(angle), math.sin(angle)
    return np.asarray(pose[:2])-np.array([co*origin[0]-si*origin[1], si*origin[0]+co*origin[1]]), angle


def executable_path(path, velocity, delay):
    """Keep current momentum before joining the ordered reference; do not teleport onto it."""
    points = np.asarray(path, dtype=float)
    velocity = np.asarray(velocity, dtype=float)
    if points.ndim != 2 or points.shape[1] != 2 or not np.isfinite(points).all():
        raise ValueError("Invalid safety path")
    if (
        velocity.shape != (2,)
        or not np.isfinite(velocity).all()
        or not math.isfinite(delay)
        or delay < 0
    ):
        raise ValueError("Invalid executable motion")
    end = velocity * delay
    nearest = 0
    while (
        nearest + 1 < len(points)
        and np.linalg.norm(points[nearest + 1] - end)
        <= np.linalg.norm(points[nearest] - end) + 1e-9
    ):
        nearest += 1
    result = [[0.0, 0.0], end.tolist()] + points[nearest:].tolist()
    return [
        p for i, p in enumerate(result) if i == 0 or math.dist(p, result[i - 1]) > 1e-7
    ]
