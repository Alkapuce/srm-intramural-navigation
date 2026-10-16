"""Find recent field outputs in their normal locations, excluding archive/test trees."""
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]


def latest(kind, root=ROOT):
    patterns = {
        'navigation': ('artifacts/mapping/*/navigation/manifest.json',
                       'artifacts/navigation/*/manifest.json', 'cache/navigation-*/manifest.json'),
        'map': ('artifacts/mapping/*/map/manifest.json', 'cache/vehicle-navigation-*/manifest.json'),
        'bag': ('artifacts/mapping/*/bag/metadata.yaml',),
    }
    candidates = []
    for pattern in patterns[kind]:
        for marker in root.glob(pattern):
            folder = marker.parent
            if kind != 'bag':
                try:
                    manifest = json.loads(marker.read_text())
                except (OSError, ValueError):
                    continue
                if not isinstance(manifest, dict):
                    continue
                if kind == 'navigation' and manifest.get('field_geometry_verified') is not True:
                    continue
                if not all((folder/name).is_file() for name in ('arena.json', 'nav.yaml', 'map.yaml', 'map.pgm', 'prior.pcd')):
                    continue
            candidates.append(marker)
    if not candidates:
        raise ValueError(f'没有找到可用的 {kind}；请先建图，或显式传入目录')
    return max(candidates, key=lambda p: (p.stat().st_mtime_ns, str(p))).parent.resolve()


if __name__ == '__main__':
    try:
        print(latest(sys.argv[1]))
    except ValueError as error:
        print(error, file=sys.stderr)
        raise SystemExit(2)
