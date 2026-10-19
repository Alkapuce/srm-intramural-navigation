#!/usr/bin/env python3
"""Read-only projection replay of recorded false handoff and true tracking faults."""

import argparse
import json
from pathlib import Path
from intramural_mission.route import segments
from intramural_mission.trajectory import Progress, RoutePlan

p = argparse.ArgumentParser()
p.add_argument("--record-root", required=True)
p.add_argument("--output", required=True)
args = p.parse_args()
root = Path(args.record_root)
c = json.loads((root / "smooth150-215756/candidate-config/arena.json").read_text())
# These failures occurred on the unchanged circle arcs, not the later entry/exit smoothing.
route = segments(c, smooth=False)
plan = RoutePlan.build(route, c)
results = {}
for name, file, begin in [
    ("false_handoff", "handoff-fault-trace.jsonl", 2),
    ("true_deviation", "upper-fault-trace.jsonl", 4),
]:
    progress = Progress(plan)
    progress.s = plan.s[plan.gates[begin - 1][1]]
    progress.phase = begin
    previous = None
    errors = []
    for line in (root / "cruise120-220751" / file).open():
        d = json.loads(line)
        m = d.get("mission", {})
        if m.get("segment", -1) < begin:
            continue
        dt = 0.1 if previous is None else max(0.0, d["time"] - previous)
        previous = d["time"]
        error = progress.project(m["pose"], max(0.15, 1.5 * (dt + 0.1)))
        errors.append(error)
        if m["state"] == "FAULT":
            break
    results[name] = dict(
        samples=len(errors),
        max_projection_error=max(errors),
        phase=progress.phase,
        fault=max(errors) > c["max_tracking_error"],
    )
assert (
    not results["false_handoff"]["fault"] and results["false_handoff"]["phase"] > 2
), results
assert results["true_deviation"]["fault"], results
out = Path(args.output)
out.parent.mkdir(parents=True, exist_ok=True)
out.write_text(
    json.dumps(
        dict(
            scope="recorded pose projection, not a new physical control result",
            passed=True,
            results=results,
        ),
        indent=2,
    )
    + "\n"
)
print(json.dumps(results))
