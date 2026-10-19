"""Executable latency prefix for raw-cloud sweeps, independent of ROS transports."""

import math
import numpy as np


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
