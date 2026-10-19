#!/usr/bin/env python3
"""Prepare a slow, explicit recovery using the most recent recorded actual trajectory."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import shutil
import yaml
from intramural_mission.route import clearance, line


def recorded_path(records, parking):
    samples=[d['mission'] for d in records if 'mission' in d and d['mission'].get('run',0)>0]
    if not samples:
        raise ValueError('No recorded run')
    run=max(m['run'] for m in samples)
    points=[]
    for m in samples:
        if m['run']!=run or m['state'] not in ('RUNNING','BLOCKED','FAULT','DONE','CANCELLED'):
            continue
        p=m.get('pose')
        if not p or len(p)!=2 or not all(math.isfinite(v) for v in p):
            raise ValueError('Invalid recorded pose')
        if points and math.dist(points[-1],p)>.3:
            raise ValueError('Recorded trajectory has a gap; cannot safely reverse it')
        if not points or math.dist(points[-1],p)>.02:
            points.append(p)
    if len(points)<2 or math.dist(points[0],parking)>.15:
        raise ValueError('Recorded run did not start near this map parking position')
    return list(reversed(points))+[list(parking)]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trace',required=True)
    parser.add_argument('--navigation-config',required=True)
    parser.add_argument('--output',required=True)
    args=parser.parse_args()
    source=Path(args.navigation_config).resolve();out=Path(args.output).resolve()
    c=json.loads((source/'arena.json').read_text())
    path=recorded_path([json.loads(l) for l in Path(args.trace).read_text().splitlines() if l.strip()],c['parking'])
    floor=min(c.get('minimum_clearance',.05),min(clearance(c,path[0]))-.01)
    for a,b in zip(path,path[1:]):
        for p in line(a,b,c['path_spacing']):
            value=min(clearance(c,p))
            if value < floor-.01:
                raise ValueError('Reversing this trace would approach a pole/boundary more closely')
            floor=min(c.get('minimum_clearance',.05),max(floor,value-.01))
    if out.exists():
        raise ValueError('Output must be a new directory')
    c.update(mission_variant='return',navigation_engine='legacy',return_path=path,max_speed=.1,allow_bypass=False,
             assumptions='Debug recovery along reversed recorded actual poses; not a competition lap')
    params=yaml.safe_load((source/'nav.yaml').read_text())
    params['controller_server']['ros__parameters']['FollowPath'].update(v_linear_min=-.1,v_linear_max=.1)
    params['velocity_smoother']['ros__parameters'].update(max_velocity=[.1,.1,0.],min_velocity=[-.1,-.1,0.])
    params['small_gicp_relocalization']['ros__parameters']['prior_pcd_file']=str(out/'prior.pcd')
    params['map_server']['ros__parameters']['yaml_filename']=str(out/'map.yaml')
    out.mkdir(parents=True)
    for name in ('prior.pcd','map.pgm','map.yaml','occupancy.json'):
        shutil.copy2(source/name,out/name)
    (out/'arena.json').write_text(json.dumps(c,indent=2)+'\n')
    (out/'nav.yaml').write_text(yaml.safe_dump(params,sort_keys=False))
    manifest=json.loads((source/'manifest.json').read_text())
    manifest.update(motion_permitted=False,start_permitted=False,recovery_trace=str(Path(args.trace).resolve()))
    manifest['files_sha256']={name:hashlib.sha256((out/name).read_bytes()).hexdigest() for name in manifest['files_sha256']}
    (out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(f'低速回位配置: {out}; 回程 {sum(math.dist(a,b) for a,b in zip(path,path[1:])):.2f} m；启动后仍需 srm-nav start')


if __name__=='__main__':
    main()
