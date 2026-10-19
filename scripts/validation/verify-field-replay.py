#!/usr/bin/env python3
"""Replay closed field bags through export, quality checks and ordered route compilation."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]


def parking(session):
    detection = session / 'navigation/field-detection.json'
    if detection.exists():
        return json.loads(detection.read_text())['parking']
    log = (session / 'navigation.log').read_text()
    recorded, _ = json.JSONDecoder().raw_decode(log[log.index('{'):])
    return recorded['body_pose_xyz_rpy'][:2]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--archive', type=Path, required=True, help='Archived NUC root containing artifacts/ and cache/')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--profile', choices=['safe', 'candidate'], default='candidate')
    parser.add_argument('--sessions', nargs='+', default=[f'full-flow-20261020-{i:02}' for i in range(1,5)])
    args = parser.parse_args()
    output = args.output.resolve()
    if output.exists() or not any(output.is_relative_to(ROOT/d) and output != ROOT/d for d in ('artifacts','cache')):
        parser.error('Choose a new project artifacts/ or cache/ directory')
    output.mkdir(parents=True)
    records = []
    for name in args.sessions:
        source = args.archive.resolve() / 'artifacts/mapping' / name
        target = output / name
        target.mkdir()
        result = dict(session=name, passed=False, physical_devices_opened=False)
        try:
            xy = parking(source)
            original = hashlib.sha256((source/'map/prior.pcd').read_bytes()).hexdigest()
            commands = [
                ('export', [ROOT/'scripts/prepare-vehicle-navigation.py', '--bag', source/'bag',
                            '--vehicle-config', args.archive.resolve()/'cache/vehicle-20261015', '--output', target/'map']),
                ('quality', [ROOT/'scripts/validation/verify-map-quality.py', '--map-config', target/'map',
                             '--output', target/'map-quality.json']),
                ('field', [ROOT/'scripts/internal/field_geometry.py', '--map-config', target/'map',
                           '--parking', *xy, '--profile', args.profile, '--output', target/'navigation']),
                ('reload', [ROOT/'scripts/validation/verify-map-trajectory.py', '--map-dir', target/'navigation',
                            '--output', target/'route-reload'])]
            for stage, command in commands:
                with (target/(stage+'.log')).open('w') as log:
                    subprocess.run([sys.executable, *map(str,command)], stdout=log, stderr=subprocess.STDOUT,
                                   check=True, timeout=300)
            if hashlib.sha256((target/'map/prior.pcd').read_bytes()).hexdigest() != original:
                raise RuntimeError('Localization prior changed during occupancy-only processing')
            result.update(passed=True, parking=xy, prior_unchanged=True,
                          detection=json.loads((target/'navigation/field-detection.json').read_text()))
        except (OSError, ValueError, KeyError, subprocess.SubprocessError) as error:
            result['error'] = str(error)
        records.append(result)
        print(json.dumps({k:v for k,v in result.items() if k!='detection'}), flush=True)
        (output/'results.json').write_text(json.dumps(records, indent=2)+'\n')
    return 0 if all(r['passed'] for r in records) else 1


if __name__ == '__main__':
    raise SystemExit(main())
