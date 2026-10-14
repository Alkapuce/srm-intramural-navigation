#!/usr/bin/env python3
"""Restore pinned public sources; preserve and verify existing local patch branches."""

import json
from pathlib import Path

from internal.source_control import git, validate

ROOT = Path(__file__).resolve().parent.parent


def main():
    for repo in json.loads((ROOT / 'docker/sources.lock.json').read_text())['repositories']:
        path = (ROOT / repo['path']).resolve()
        if not path.is_relative_to(ROOT):
            raise RuntimeError('Source path escapes intramural')
        if not path.exists() or not any(path.iterdir()):
            path.mkdir(parents=True, exist_ok=True)
            git(path, 'init', '-q')
            git(path, 'remote', 'add', 'origin', repo['url'])
            git(path, 'fetch', '--depth=1', 'origin', repo['commit'])
            git(path, 'checkout', '--detach', 'FETCH_HEAD')
            if repo['recursive']:
                git(path, 'submodule', 'update', '--init', '--recursive', '--depth=1')
        state = validate(ROOT, repo, apply_missing=True)
        print(f"OK {repo['path']} {repo['commit']} {state['state']}", flush=True)


if __name__ == '__main__':
    main()
