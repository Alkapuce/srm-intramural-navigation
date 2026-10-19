#!/usr/bin/env python3
"""Plot independent physical trajectories, poles, footprint clearance and speed."""

import argparse, json, math
from pathlib import Path
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle
from intramural_mission.route import segments, primitives

p = argparse.ArgumentParser()
p.add_argument("--input", required=True)
a = p.parse_args()
folder = Path(a.input)
c = json.loads((folder / "arena.json").read_text())
r = json.loads((folder / "results.json").read_text())
data = [json.loads(s) for s in (folder / "trajectory.jsonl").read_text().splitlines()]
fig, (ax, sv) = plt.subplots(1, 2, figsize=(13, 6))
xmin, xmax, ymin, ymax = c["bounds"]
ax.add_patch(
    Rectangle((xmin, ymin), xmax - xmin, ymax - ymin, fill=False, edgecolor="black")
)
for k in ["lower_pole", "upper_pole"]:
    ax.add_patch(Circle(c[k], c["pole_radius"], color="orange"))
    ax.add_patch(
        Circle(
            c[k],
            c["pole_radius"] + c.get("robot_radius", math.hypot(c["robot_length"], c["robot_width"]) / 2),
            fill=False,
            linestyle=":",
            edgecolor="gray",
        )
    )
reference=folder/'reference-path.json'
route = ([('FULL',json.loads(reference.read_text())['points'])] if reference.is_file() else
         primitives(c) if r["variant"] == "primitives" else segments(c))
for _, points in route:
    ax.plot(
        [v[0] for v in points], [v[1] for v in points], "--", color="gray", linewidth=1
    )
for run in sorted(set(v["run"] for v in data if v["run"])):
    d = [v for v in data if v["run"] == run]
    ax.plot([v["x"] for v in d], [v["y"] for v in d], label=f"Run {run}", alpha=0.7)
    sv.plot(
        [v["sim"] - d[0]["sim"] for v in d],
        [v["speed"] for v in d],
        label=f"Run {run}",
        alpha=0.7,
    )
ax.scatter(*c["parking"], color="green", label="Parking")
ax.set_aspect("equal")
ax.set(
    xlabel="Gazebo world x (m)",
    ylabel="Gazebo world y (m)",
    title="Independent physical trajectory",
)
ax.legend(loc="upper left")
ax.grid(alpha=0.2)
sv.set(
    xlabel="Elapsed simulation time (s)",
    ylabel="Actual planar speed (m/s)",
    title="Physical speed and final stop",
)
sv.grid(alpha=0.2)
fig.suptitle(
    f"{r['mode']} localization | {r['variant']} | {len(r['rounds'])} completed runs (placeholder robot)"
)
fig.tight_layout()
fig.savefig(folder / "trajectory.png", dpi=150)
fig.savefig(folder / "trajectory.svg")
print(folder / "trajectory.png")
