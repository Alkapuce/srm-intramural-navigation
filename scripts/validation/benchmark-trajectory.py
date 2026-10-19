#!/usr/bin/env python3
"""Deterministic delayed-motion tests of the production tracker; never sends ROS commands."""

import argparse
import hashlib
import json
from pathlib import Path
import sys
import math
import numpy as np

root = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(root / "tests"))
from test_trajectory import rollout

p = argparse.ArgumentParser()
p.add_argument("--output", required=True)
p.add_argument("--profile", default="safe", choices=["safe", "candidate"])
p.add_argument("--cases", type=int, default=24)
args = p.parse_args()
out = Path(args.output)
out.mkdir(parents=True, exist_ok=True)
base = json.loads((root / "config/arena.json").read_text())
profile = json.loads((root / "config/navigation-profiles.json").read_text())[
    args.profile
]
results = []
for i in range(args.cases):
    rng = np.random.default_rng(i)
    radius = float(rng.uniform(0.6, 1.5))
    angle = float(rng.uniform(-math.pi, math.pi))
    co, si = math.cos(angle), math.sin(angle)

    def transform(p):
        return [co * p[0] - si * p[1], si * p[0] + co * p[1]]

    parking = [0, 0] if i % 2 else [4 * radius, 0]
    c = dict(base, **profile)
    c.update(
        parking=transform(parking),
        lower_pole=transform([2 * radius, radius]),
        upper_pole=transform([2 * radius, 3 * radius]),
        loop_radius=radius,
        entry_angle=math.pi if i % 2 else 0.0,
    )
    delay = float(rng.uniform(0.1, 0.4))
    tau = float(rng.uniform(0.05, 0.3))
    gain = float(rng.uniform(0.7, 1.3))
    result, trace = rollout(c, delay, tau, gain, 0.002, i)
    result.update(seed=i, delay=delay, tau=tau, gain=gain, radius=radius)
    result["passed"] = bool(
        result["done"]
        and result["winding"] == [1, -1]
        and result["parking_error"] < 0.12
        and result["max_tracking_error"] < c["max_tracking_error"]
        and result["minimum_pole_clearance"] > 0.02
    )
    (out / f"case-{i:02d}.jsonl").write_text(
        "".join(json.dumps(x) + "\n" for x in trace)
    )
    results.append(result)
    print(json.dumps(result), flush=True)
report = dict(
    model="delayed first order holonomic plant; not vehicle validation",
    profile=args.profile,
    cases=results,
    passed=all(v["passed"] for v in results),
    source_sha256={
        str(f.relative_to(root)): hashlib.sha256(f.read_bytes()).hexdigest()
        for f in (root / "nav_ws/src/intramural_mission/intramural_mission").glob(
            "*.py"
        )
    },
)
(out / "results.json").write_text(json.dumps(report, indent=2) + "\n")
raise SystemExit(0 if report["passed"] else 1)
