#!/usr/bin/env python3
"""Read-only package, source and live ROS graph diagnostics."""

import argparse
import json
import os
import platform
import subprocess
from pathlib import Path

import rclpy
from source_control import validate

p = argparse.ArgumentParser()
p.add_argument("--output", required=True)
a = p.parse_args()
root = Path(__file__).resolve().parents[2]
out = Path(a.output).resolve()
if not out.is_relative_to(root / "artifacts"):
    raise ValueError("Output must be under artifacts")
out.mkdir(parents=True, exist_ok=True)
lock = json.loads((root / "docker/sources.lock.json").read_text())
sources = []
for repo in lock["repositories"]:
    sources.append(validate(root, repo))
packages = {}
for name in ("intramural_mission", "intramural_base", "point_lio",
             "small_gicp_relocalization", "nav2_controller"):
    r = subprocess.run(["ros2", "pkg", "prefix", name], capture_output=True, text=True)
    packages[name] = r.stdout.strip() if r.returncode == 0 else None
rclpy.init()
node = rclpy.create_node("candidate_diagnostic")
try:
    nodes = sorted([f"{namespace.rstrip('/')}/{name}" for name, namespace
                    in node.get_node_names_and_namespaces()])
    topics = sorted([name for name, _ in node.get_topic_names_and_types()])
finally:
    node.destroy_node()
    rclpy.shutdown()
report = {"platform": platform.platform(), "python": platform.python_version(),
          "ros_domain_id": os.environ.get("ROS_DOMAIN_ID"),
          "ign_partition": os.environ.get("IGN_PARTITION"),
          "transport": os.environ.get("INTRAMURAL_TRANSPORT"),
          "sources": sources, "packages": packages, "nodes": nodes, "topics": topics,
          "vehicle_calibration_verified": False,
          "hardware_connection_checked": False}
(out / "diagnostics.json").write_text(json.dumps(report, indent=2) + "\n")
if not all(item["matches"] for item in sources) or not all(packages.values()):
    raise RuntimeError("Pinned source or required package unavailable")
print("PASS candidate read-only diagnostics: " + str(out / "diagnostics.json"))
