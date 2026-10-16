"""Observed-space occupancy export using a fitted floor and OctoMap ray updates."""
import json
from pathlib import Path
import struct
import subprocess

import numpy as np


def fit_floor(points, sensor_origins):
    ceiling = float(np.quantile(sensor_origins[:, 2], .1) - .04)
    low = points[points[:, 2] < ceiling]
    if len(low) < 100:
        raise ValueError("Too few floor candidates below the sensor; scan more visible floor")
    rng = np.random.default_rng(0)
    sample = low[rng.choice(len(low), min(5000, len(low)), replace=False)]
    design = np.c_[sample[:, :2], np.ones(len(sample))]
    best, count = None, 0
    for _ in range(160):
        indices = rng.choice(len(sample), 3, replace=False)
        if abs(np.linalg.det(design[indices])) < .01:
            continue
        plane = np.linalg.solve(design[indices], sample[indices, 2])
        if np.linalg.norm(plane[:2]) > .1:
            continue
        support = np.count_nonzero(np.abs(design @ plane - sample[:, 2]) < .025)
        if support > count:
            best, count = plane, support
    if best is None or count < max(50, len(sample) * .15):
        raise ValueError("No supported near-horizontal floor plane; inspect height data")
    design = np.c_[low[:, :2], np.ones(len(low))]
    mask = np.abs(design @ best - low[:, 2]) < .025
    plane = np.linalg.lstsq(design[mask], low[mask, 2], rcond=None)[0]
    span = np.ptp(low[mask, :2], axis=0)
    if np.linalg.norm(plane[:2]) > .1 or min(span) < .5:
        raise ValueError("Floor fit has insufficient extent or excessive slope")
    height = sensor_origins[:, 2] - np.c_[sensor_origins[:, :2], np.ones(len(sensor_origins))] @ plane
    if np.min(height) < .04:
        raise ValueError("Estimated floor reaches the sensor; check the pose/coordinate frames")
    return plane, {"z_equals_ax_by_c": plane.tolist(), "inliers": int(mask.sum()),
                   "rms_m": float(np.sqrt(np.mean((design[mask] @ plane-low[mask, 2])**2))),
                   "xy_span_m": span.tolist(), "sensor_height_range_m": [float(height.min()), float(height.max())]}


def raycast_binary(root):
    source = root / "scripts/internal/occupancy-raycast.cpp"
    binary = root / "cache/mapping/occupancy-raycast"
    binary.parent.mkdir(parents=True, exist_ok=True)
    if not binary.exists() or binary.stat().st_mtime < source.stat().st_mtime:
        temporary = binary.with_suffix(".tmp")
        subprocess.run(["c++", "-O2", "-std=c++17", str(source), "-loctomap", "-loctomath", "-o", str(temporary)], check=True)
        temporary.replace(binary)
    return binary


def raycast_grid(scans, sensor_origins, plane, origin, size, resolution, root):
    """Scans and ray origins share the exported map frame; unknown cells stay unknown."""
    if len(scans) != len(sensor_origins):
        raise ValueError("Every scan requires its own timestamp-matched ray origin")
    binary = raycast_binary(root)
    process = subprocess.Popen([str(binary)], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    used = 0
    try:
        process.stdin.write(b"SRMGRID1" + struct.pack("<d2d2I3d", resolution, *origin, *size, *plane))
        for points, sensor in zip(scans, sensor_origins):
            height = points[:, 2] - (points[:, :2] @ plane[:2] + plane[2])
            ground = np.abs(height) <= .03
            obstacle = (height >= .05) & (height <= 1.4)
            selected = ground | obstacle
            data = np.c_[points[selected], obstacle[selected]].astype("<f4")
            process.stdin.write(struct.pack("<I3f", len(data), *sensor))
            process.stdin.write(data.tobytes())
            used += len(data)
        process.stdin.close()
        process.stdin = None
        output, error = process.communicate(timeout=180)
        if process.returncode:
            raise RuntimeError("OctoMap export failed: " + error.decode(errors="replace"))
        if len(output) != int(size[0] * size[1]):
            raise RuntimeError("OctoMap returned an incomplete grid")
        return np.frombuffer(output, dtype=np.uint8).reshape(size[1], size[0]), {
            "algorithm": "octomap_3d_rays_floor_relative_projection",
            "ground_tolerance_m": .03, "obstacle_height_range_m": [.05, 1.4],
            "projection_uses_voxel_bounds": True,
            "inserted_points": used, "scans": len(scans),
            "octomap": json.loads(error.decode().strip()),
            "limits": "Near-horizontal floor assumed; unknown space is not filled and vehicle radius is unchanged",
        }
    finally:
        if process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
