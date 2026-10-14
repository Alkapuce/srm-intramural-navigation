#!/usr/bin/env python3
"""Independent geometric audit of four interference runs from Gazebo truth logs."""

import argparse
import json
import math
from pathlib import Path

import numpy as np
from intramural_mission.route import segments, clearance

p = argparse.ArgumentParser()
p.add_argument("--input", required=True)
a = p.parse_args()
folder = Path(a.input).resolve()
if not folder.is_relative_to(Path("/work/artifacts")):
    raise ValueError("Evidence must be under artifacts")
results = json.loads((folder / "results.json").read_text())
records = [json.loads(line) for line in (folder / "trajectory.jsonl").read_text().splitlines()]
c = json.loads((folder / "arena.json").read_text())
reference = [point for _, path in segments(c) for point in path]
expected = [name for name, _ in segments(c)]
audit = []
for run, result in enumerate(results, 1):
    sample = [r for r in records if r["run"] == run]
    if len(sample) < 500 or result["history"] != expected:
        raise RuntimeError(f"Missing actual trajectory or ordered phases in run {run}")
    positions = np.asarray([(r["x"], r["y"]) for r in sample])
    winding = {}
    for key in ("lower_pole", "upper_pole"):
        pole = np.asarray(c[key])
        theta = np.unwrap(np.arctan2(positions[:, 1] - pole[1], positions[:, 0] - pole[0]))
        winding[key] = float(theta[-1] - theta[0])
    if not 6.0 < winding["lower_pole"] < 6.6 or not -6.6 < winding["upper_pole"] < -6.0:
        raise RuntimeError(f"Wrong true winding in run {run}: {winding}")
    deviation = max(min(math.dist((r["x"], r["y"]), point) for point in reference)
                    for r in sample[::5])
    allowed = c['bypass_max_deviation']+.1 if result.get('persistent') else c['max_tracking_error']
    if deviation > allowed or result["min_static_clearance_m"] <= 0:
        raise RuntimeError(f"Unsafe physical trajectory in run {run}: {deviation}")
    entered = np.asarray([(r['x'],r['y']) for r in sample if r['phase'] == 'ENTER'])[::5]
    returned = np.asarray([(r['x'],r['y']) for r in sample if r['phase'] == 'RETURN'])[::5]
    return_deviation = float(np.linalg.norm(returned[:,None]-entered[None,:],axis=2).min(axis=1).max())
    if return_deviation > c['max_tracking_error']:
        raise RuntimeError(f'Return did not retrace entry in run {run}: {return_deviation}')
    blocked = [r for r in sample if r["state"] == "BLOCKED"]
    if len(blocked) < 20:
        raise RuntimeError(f"No sustained obstacle stop in run {run}")
    target = result["target"]
    minimum_dynamic_distance = min(math.dist((r["x"], r["y"]), target) for r in blocked)
    if minimum_dynamic_distance < c.get('robot_radius',.3905)+.25+.08:
        raise RuntimeError(f"Box collision risk in run {run}: {minimum_dynamic_distance}")
    dynamic_clearance = None
    if result.get('persistent'):
        dynamic_clearance = min(math.hypot(max(abs(r['x']-target[0])-.25,0),
            max(abs(r['y']-target[1])-.25,0))-c['robot_radius'] for r in sample
            if r['wall'] >= blocked[0]['wall'])
        if dynamic_clearance < .08 or result.get('bypass_count',0) == 0:
            raise RuntimeError(f'Persistent obstacle was not safely bypassed: {dynamic_clearance}')
    item = {"scenario": result["scenario"], "run": run,
            "true_winding_rad": winding, "max_reference_deviation_m": deviation,
            "minimum_box_center_distance_while_blocked_m": minimum_dynamic_distance,
            "minimum_dynamic_clearance_m": dynamic_clearance,
            "max_return_to_entry_deviation_m": return_deviation,
            "blocked_samples": len(blocked)}
    audit.append(item)
(folder / "geometry-audit.json").write_text(json.dumps(audit, indent=2) + "\n")
print("PASS physical winding, ordered phases, static and dynamic clearance in", len(audit), "runs")
