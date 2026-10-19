#!/usr/bin/env python3
"""Compare archived routes with current control and executable-stop constraints, without devices."""
import argparse
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import sys
import numpy as np
import yaml
from PIL import Image
from nav_msgs.msg import OccupancyGrid
from intramural_mission.planning import GridMap, compile_route
from intramural_mission.trajectory import RoutePlan
from intramural_mission.response import predict_execution
from intramural_mission.safety import response_horizon

ROOT = Path(__file__).resolve().parents[2]

def load_grid(folder):
    metadata=yaml.safe_load((folder/'map.yaml').read_text())
    pixels=np.asarray(Image.open(folder/metadata['image']))
    grid=OccupancyGrid();grid.header.frame_id='map'
    grid.info.width=pixels.shape[1];grid.info.height=pixels.shape[0];grid.info.resolution=metadata['resolution']
    grid.info.origin.position.x,grid.info.origin.position.y=metadata['origin'][:2]
    yaw=metadata['origin'][2];grid.info.origin.orientation.w=math.cos(yaw/2);grid.info.origin.orientation.z=math.sin(yaw/2)
    grid.data=np.where(pixels[::-1]==254,0,np.where(pixels[::-1]==205,-1,100)).ravel().tolist()
    return grid


def coast_events(file, measured, limits):
    previous=None;first=None;minimum=math.inf
    for line in file.open():
        row=json.loads(line)
        if row['state'] in ('DONE','FAULT','CANCELLED'):break
        if not row.get('pose') or not row.get('history') or row.get('tf_age_sim_seconds') is None:continue
        stamp=row['history'][0]['sim_time']+row['elapsed_sim_seconds']-row['tf_age_sim_seconds']
        position=np.array(row['pose'])
        if previous is not None and stamp-previous[0] > .001:
            velocity=(position-previous[1])/(stamp-previous[0])
            if np.linalg.norm(velocity)<2*limits['max_speed']:
                coast=velocity*response_horizon(limits)
                points=position+np.linspace(0.,1.,max(2,math.ceil(np.linalg.norm(coast)/(measured.res/2))))[:,None]*coast
                gap=float(min(measured.clearance(points)));minimum=min(minimum,gap)
                if gap<=0 and first is None:
                    first=dict(elapsed=row['elapsed_sim_seconds'],pose=position.tolist(),velocity=velocity.tolist(),clearance=gap)
        if previous is None or stamp>previous[0]+.001:previous=(stamp,position)
    return dict(first_counterfactual_coast_risk=first,minimum_clearance=minimum,
                scope='Archived localization derivatives and static map, not a claim of physical stopping distance')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--archive',type=Path,required=True)
    p.add_argument('--map-dir',type=Path,required=True)
    p.add_argument('--runs',nargs='+',required=True)
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args();out=args.output.resolve()
    if any(not name or Path(name).name != name or name in ('.','..') for name in args.runs):
        p.error('Run IDs must be single directory names')
    if out.is_relative_to(args.archive.resolve()):p.error('Replay output must not modify the input archive')
    if out.exists() or not any(out.is_relative_to(ROOT/d) and out!=ROOT/d for d in ('artifacts','cache')):
        p.error('Choose a new project artifacts/ or cache/ directory')
    out.mkdir(parents=True);grid=load_grid(args.map_dir)
    profile=json.loads((ROOT/'config/navigation-profiles.json').read_text())['candidate'];results=[]
    for ident in args.runs:
        folder=args.archive/'artifacts/navigation-runs'/ident
        manifest=json.loads((folder/'manifest.json').read_text());c=dict(manifest['config'],**profile)
        measured=GridMap(grid,c)
        legacy = hashlib.sha256(measured.free.tobytes()+repr((float(np.float32(measured.res)),measured.origin.tolist(),measured.yaw)).encode()).hexdigest()
        if manifest['map_id'] not in (measured.map_id,legacy):raise ValueError('Replay map identity does not match the archived run')
        old=RoutePlan(**json.loads((folder/'route.json').read_text()))
        record=dict(run_id=ident,recorded=coast_events(folder/'status.jsonl',measured,c),passed=False)
        try:
            # Recheck the original geometry and its actual entry, rather than replacing a failed route with another one.
            plan,_,report=compile_route(c,grid,start=old.points[0])
            predictions=[predict_execution(plan,c,measured,plant_response=model)
                         for model in c['response_models']+c.get('response_stress_models',[])]
            record.update(passed=all(v['passed'] for v in predictions),optimized_route=report['optimized_route'],
                          predictions=predictions,full_history_gates=len(plan.gates),planning_seconds=report['planning_seconds'])
            (out/(ident+'-route.json')).write_text(json.dumps(asdict(plan),indent=2)+'\n')
        except ValueError as error:record['error']=str(error)
        results.append(record)
        print(json.dumps(record),flush=True)
        (out/'results.json').write_text(json.dumps(results,indent=2)+'\n')
    return 0 if all(r['passed'] for r in results) else 1


if __name__=='__main__':raise SystemExit(main())
