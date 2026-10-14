#!/usr/bin/env python3
"""Write a local, hash-addressed pre-vehicle candidate inventory."""

import argparse
import hashlib
import json
import subprocess
from datetime import datetime, timezone
from pathlib import Path
from internal.source_control import git, validate

p = argparse.ArgumentParser()
p.add_argument("--output", required=True)
p.add_argument("--evidence", action="append", default=[])
a = p.parse_args()
root = Path("/work")
out = Path(a.output).resolve()
if not out.is_relative_to(root / "artifacts"):
    raise ValueError("Candidate inventory must be under artifacts")
out.mkdir(parents=True, exist_ok=True)
if (out / "candidate-manifest.json").exists():
    raise ValueError("Candidate manifest already exists; choose a new output directory")
tracked = subprocess.check_output(["git", "-C", str(root), "ls-files", "-z"])
files = [root / name.decode() for name in tracked.split(b"\0") if name]
if not files or any(not file.is_file() for file in files):
    raise ValueError("Freeze requires a complete root Git checkout")
file_hashes = {str(f.relative_to(root)): hashlib.sha256(f.read_bytes()).hexdigest()
               for f in sorted(set(files))}
evidence = {}
for path in a.evidence:
    file = (root / path).resolve()
    if not file.is_relative_to(root / "artifacts") or not file.exists():
        raise ValueError("Evidence path missing or outside artifacts: " + path)
    evidence[str(file.relative_to(root))] = hashlib.sha256(file.read_bytes()).hexdigest()
manifest = {"name": "SRM intramural pre-vehicle candidate",
    "generated_utc": datetime.now(timezone.utc).isoformat(),
    "vehicle_validated": False,
    "firmware_watchdog_confirmed": False,
    "real_rosbag_available": False,
    "public_sources": json.loads((root / "docker/sources.lock.json").read_text())["repositories"],
    "files_sha256": file_hashes, "evidence_sha256": evidence}
manifest["public_checkout_states"] = [validate(root, repo) for repo in manifest["public_sources"]]
manifest["project_git"] = {"head": git(root, "rev-parse", "HEAD"),
                           "branch": git(root, "branch", "--show-current"),
                           "worktree_dirty": bool(git(root, "status", "--porcelain"))}
(out / "candidate-manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
print(f"Frozen {len(file_hashes)} project files and {len(evidence)} evidence files at {out}")
