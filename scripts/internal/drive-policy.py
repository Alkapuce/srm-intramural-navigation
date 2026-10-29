#!/usr/bin/env python3
"""Create a separate navigation configuration with the selected obstacle policy."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import tempfile


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('navigation', type=Path)
    policy = parser.add_mutually_exclusive_group(required=True)
    policy.add_argument('--allow-bypass', dest='bypass', action='store_true')
    policy.add_argument('--no-bypass', dest='bypass', action='store_false')
    args = parser.parse_args()
    source = args.navigation.resolve(strict=True)
    manifest = json.loads((source/'manifest.json').read_text())
    config = json.loads((source/'arena.json').read_text())
    if (not manifest.get('field_geometry_verified') or not manifest.get('route_sources_sha256')
            or config.get('navigation_engine') != 'trajectory'):
        parser.error('需使用已通过场地检查的轨迹导航配置')
    for name in ('arena.json', 'route-plan.json', 'route-analysis.json'):
        if hashlib.sha256((source/name).read_bytes()).hexdigest() != manifest.get('files_sha256', {}).get(name):
            parser.error(f'{name} 已变化；请重新生成场地配置')
    config['allow_bypass'] = args.bypass
    manifest['obstacle_policy'] = 'predictive_detour' if args.bypass else 'stop_and_wait'
    cache = Path(__file__).resolve().parents[2]/'cache'
    cache.mkdir(exist_ok=True)
    out = Path(tempfile.mkdtemp(prefix='drive-policy-', dir=cache))
    for item in source.iterdir():
        if item.name not in ('arena.json', 'manifest.json', 'nav.yaml'):
            (out/item.name).symlink_to(os.path.relpath(item.resolve(), out))
    shutil.copy2(source/'nav.yaml', out/'nav.yaml')
    (out/'arena.json').write_text(json.dumps(config, indent=2)+'\n')
    manifest['files_sha256']['arena.json'] = hashlib.sha256((out/'arena.json').read_bytes()).hexdigest()
    (out/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    print(out)


if __name__ == '__main__':
    main()
