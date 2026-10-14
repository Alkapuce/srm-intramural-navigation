#!/usr/bin/env python3
"""Move the simulated interference obstacle; does not change robot commands."""

import argparse, json, math, subprocess
from pathlib import Path

p = argparse.ArgumentParser(description=__doc__)
p.add_argument("operation", choices=["place", "park"])
p.add_argument("--x", type=float)
p.add_argument("--y", type=float)
a = p.parse_args()
c = json.loads(Path("/work/config/arena.json").read_text())
x, y = c["obstacle_initial"] if a.operation == "park" else (a.x, a.y)
if x is None or y is None or not all(math.isfinite(v) for v in (x, y)):
    raise ValueError("Finite x,y required")
xmin, xmax, ymin, ymax = c["bounds"]
sx, sy, sz = c["obstacle_size"]
if not (xmin + sx / 2 < x < xmax - sx / 2 and ymin + sy / 2 < y < ymax - sy / 2):
    raise ValueError("Obstacle outside arena")
req = f'name: "movable_obstacle", position: {{x: {x}, y: {y}, z: {sz / 2}}}, orientation: {{w: 1}}'
r = subprocess.run(
    [
        "ign",
        "service",
        "-s",
        "/world/default/set_pose",
        "--reqtype",
        "ignition.msgs.Pose",
        "--reptype",
        "ignition.msgs.Boolean",
        "--timeout",
        "3000",
        "--req",
        req,
    ],
    capture_output=True,
    text=True,
    check=True,
)
if "true" not in r.stdout:
    raise RuntimeError("Obstacle pose not acknowledged")
print(f"Obstacle {a.operation} at ({x},{y})")
