"""Validate upstream bases and exact portable patches, regardless of local HEAD."""

import hashlib
import os
import subprocess
from pathlib import Path


def git(path, *args):
    return subprocess.check_output(["git", "-C", str(path), *args], text=True).strip()


def repo_path(root, repo):
    path = (root / repo["path"]).resolve()
    if not path.is_relative_to(root.resolve()):
        raise RuntimeError("Source path escapes project")
    if Path(git(path, "rev-parse", "--show-toplevel")).resolve() != path:
        raise RuntimeError(f"Expected an independent source repository: {path}")
    return path


def check_base(path, repo):
    if subprocess.run(["git", "-C", str(path), "merge-base", "--is-ancestor",
                       repo["commit"], "HEAD"], capture_output=True).returncode:
        raise RuntimeError(f"HEAD does not descend from pinned upstream base: {path}")
    if repo["recursive"]:
        status = git(path, "submodule", "status", "--recursive")
        if any(line.startswith(("+", "-", "U")) for line in status.splitlines()):
            raise RuntimeError(f"Submodule mismatch: {path}")
    if git(path, "ls-files", "--others", "--exclude-standard"):
        raise RuntimeError(f"Untracked source files; inspect before exporting: {path}")


def diff_from_base(path, repo):
    return subprocess.check_output(["git", "-C", str(path), "diff", "--no-ext-diff",
                                    "--no-textconv", repo["commit"], "--binary"])


def validate(root, repo, apply_missing=False):
    path = repo_path(root, repo)
    check_base(path, repo)
    head = git(path, "rev-parse", "HEAD")
    status = git(path, "status", "--porcelain")
    patch_info = repo.get("patch")
    if patch_info:
        patch = (root / patch_info["path"]).resolve()
        if not patch.is_relative_to(root.resolve()):
            raise RuntimeError("Patch path escapes project")
        content = patch.read_bytes()
        if hashlib.sha256(content).hexdigest() != patch_info["sha256"]:
            raise RuntimeError(f"Patch checksum mismatch: {patch}")
        if apply_missing and head == repo["commit"] and not status:
            git(path, "apply", "--check", str(patch))
            git(path, "apply", str(patch))
            # A patch can add files that are still untracked after applying;
            # mark only those paths intent-to-add for normal diff validation.
            added = subprocess.check_output(["git", "-C", str(path), "ls-files",
                                              "--others", "--exclude-standard", "-z"])
            names = [os.fsdecode(name) for name in added.split(b"\0") if name]
            if names:
                git(path, "add", "--intent-to-add", "--", *names)
        if diff_from_base(path, repo) != content:
            raise RuntimeError(f"Changes differ from locked patch; inspect/export explicitly: {path}")
        if git(path, "diff", "--cached", "--name-only"):
            raise RuntimeError(f"Staged changes; preserve and inspect: {path}")
        if head != repo["commit"] and git(path, "status", "--porcelain"):
            raise RuntimeError(f"Local source branch has uncommitted changes: {path}")
        git(path, "apply", "--reverse", "--check", str(patch))
    elif head != repo["commit"] or status:
        raise RuntimeError(f"Unpatched checkout differs from pinned base: {path}")
    return {"path": repo["path"], "expected_base": repo["commit"], "actual_head": head,
            "branch": git(path, "branch", "--show-current") or "(detached)",
            "state": "committed-patch" if head != repo["commit"] else
                     "applied-patch" if patch_info else "upstream", "matches": True}
