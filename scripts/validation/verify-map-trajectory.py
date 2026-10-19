#!/usr/bin/env python3
"""Compile a measured map and optional localized starts, without opening any device."""

import argparse
from dataclasses import asdict
import json
import math
from pathlib import Path
import numpy as np
from PIL import Image
import yaml
from nav_msgs.msg import OccupancyGrid
from intramural_mission.planning import compile_route

parser = argparse.ArgumentParser()
parser.add_argument("--map-dir", required=True)
parser.add_argument("--output", required=True)
parser.add_argument("--profile", choices=["safe", "candidate"])
parser.add_argument("--start", type=float, nargs=2, action="append", default=[])
args = parser.parse_args()
root = Path(__file__).resolve().parents[2]
folder = Path(args.map_dir)
c = json.loads((folder / "arena.json").read_text())
profile = args.profile or c.get("navigation_profile", "safe")
if args.profile or c.get("navigation_engine") != "trajectory":
    if args.profile and args.profile != c.get("navigation_profile"):
        c.pop("optimized_route",None)
    c.update(json.loads((root / "config/navigation-profiles.json").read_text())[profile])
metadata = yaml.safe_load((folder / "map.yaml").read_text())
pixels = np.asarray(Image.open(folder / metadata["image"]))
g = OccupancyGrid()
g.header.frame_id = "map"
g.info.width = pixels.shape[1]
g.info.height = pixels.shape[0]
g.info.resolution = metadata["resolution"]
x, y, yaw = metadata["origin"]
g.info.origin.position.x = x
g.info.origin.position.y = y
g.info.origin.orientation.w = math.cos(yaw / 2)
g.info.origin.orientation.z = math.sin(yaw / 2)
g.data = (
    np.where(pixels[::-1] == 254, 0, np.where(pixels[::-1] == 205, -1, 100))
    .astype(np.int8)
    .ravel()
    .tolist()
)
out = Path(args.output)
out.mkdir(parents=True, exist_ok=True)
results = []
for i, start in enumerate([None] + args.start):
    try:
        plan, route, report = compile_route(c, g, start)
        (out / f"route-{i}.json").write_text(json.dumps(asdict(plan), indent=2) + "\n")
        results.append(
            dict(
                start=start,
                passed=True,
                length=plan.s[-1],
                gates=len(plan.gates),
                route_id=plan.route_id,
                **report,
            )
        )
    except ValueError as exc:
        results.append(dict(start=start, passed=False, reason=str(exc)))
report = dict(
    profile=profile, results=results, passed=all(r["passed"] for r in results)
)
(out / "results.json").write_text(json.dumps(report, indent=2) + "\n")
print(json.dumps(report))
raise SystemExit(0 if report["passed"] else 1)
