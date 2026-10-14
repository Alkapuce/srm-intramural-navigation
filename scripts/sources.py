#!/usr/bin/env python3
"""Inspect locked sources or export committed local repairs as portable patches."""

import argparse
import hashlib
import json
from pathlib import Path

from internal.source_control import check_base, diff_from_base, git, repo_path, validate

ROOT = Path(__file__).resolve().parent.parent


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    commands.add_parser("status")
    export = commands.add_parser("export")
    export.add_argument("repository", help="Repository basename or locked path")
    args = parser.parse_args()
    lock_path = ROOT / "docker/sources.lock.json"
    lock = json.loads(lock_path.read_text())
    if args.command == "status":
        for repo in lock["repositories"]:
            print(json.dumps(validate(ROOT, repo), ensure_ascii=False))
        return
    matches = [repo for repo in lock["repositories"] if args.repository in
               (repo["path"], Path(repo["path"]).name)]
    if len(matches) != 1:
        parser.error("Select exactly one locked repository")
    repo = matches[0]
    path = repo_path(ROOT, repo)
    check_base(path, repo)
    if not git(path, "branch", "--show-current").startswith("intramural/"):
        parser.error("Export requires a named intramural/ local development branch")
    if git(path, "status", "--porcelain"):
        parser.error("Commit reviewed changes first; export requires a clean source branch")
    content = diff_from_base(path, repo)
    if not content:
        parser.error("No changes relative to pinned upstream base")
    patch_name = repo.get("patch", {}).get("path", f"patches/{path.name}-local.patch")
    patch = (ROOT / patch_name).resolve()
    if not patch.is_relative_to(ROOT / "patches"):
        parser.error("Patch output must be under patches/")
    if patch.exists() and "patch" not in repo:
        parser.error("Unregistered patch already exists; preserve and inspect it")
    patch.write_bytes(content)
    repo["patch"] = {"path": patch_name, "sha256": hashlib.sha256(content).hexdigest()}
    lock_path.write_text(json.dumps(lock, indent=2) + "\n")
    validate(ROOT, repo)
    print(f"Exported {repo['path']} to {patch_name}; review and commit patch + lock in root")


if __name__ == "__main__":
    main()
