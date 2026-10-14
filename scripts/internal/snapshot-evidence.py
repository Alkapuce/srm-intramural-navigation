#!/usr/bin/env python3
"""Capture generated arena and checked-in implementation hashes for a test run."""

import argparse
import hashlib
import json
import shutil
from pathlib import Path

p = argparse.ArgumentParser()
p.add_argument("--output", required=True)
a = p.parse_args()
root = Path(__file__).resolve().parents[2]
out = Path(a.output).resolve()
if not out.is_relative_to(root / "artifacts"):
    raise ValueError("Evidence must be under artifacts")
out.mkdir(parents=True, exist_ok=True)
for file in ("config/arena.json", "config/base-sim.json", "config/fastdds-services.xml",
             "docker/sources.lock.json", "simulation/generated/manifest.json"):
    source = root / file
    shutil.copyfile(source, out / source.name)
shutil.copytree(root / "simulation/generated", out / "arena-snapshot", dirs_exist_ok=True)
files = [file for file in (root / "scripts").rglob("*")
         if file.is_file() and file.suffix in (".py", ".sh", ".cpp")]
files += list((root / "launch").glob("arena*.py"))
files += list((root / "nav_ws/src/intramural_mission").rglob("*.py"))
files += list((root / "nav_ws/src/intramural_base").rglob("*.py"))
files += list((root / "config").glob("*.json"))
files += [root / "config/fastdds-services.xml"]
files += [root / 'config/arena-gui.config', root / '算法组校内赛规则/sentry_robot_cylinder.xacro',
          root / 'nav_ws/src/intramural_mission/package.xml']
(out / "implementation-sha256.json").write_text(json.dumps({
    str(file.relative_to(root)): hashlib.sha256(file.read_bytes()).hexdigest()
    for file in sorted(set(files))}, indent=2) + "\n")
print(f"Captured {len(files)} source/config hashes in {out}")
