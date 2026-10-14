#!/usr/bin/env python3
"""Render recorded simulator truth; this is a data plot, not a GUI screenshot."""
import argparse
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--input',default='/work/artifacts/stage2/navigation-complete')
args=parser.parse_args()
root=Path(args.input).resolve()
if not root.is_relative_to(Path('/work/artifacts')):raise ValueError('Invalid input directory')
records=[json.loads(line) for line in (root/'trajectory.jsonl').read_text().splitlines()]
result=json.loads((root/'results.json').read_text())
x=[p['x'] for p in records];y=[p['y'] for p in records]
t=[p['sim_time']-records[0]['sim_time'] for p in records]
fig,axes=plt.subplots(1,2,figsize=(11,4.5),constrained_layout=True)
axes[0].plot(x,y,label='Simulator chassis ground truth')
axes[0].scatter([x[0],x[-1]],[y[0],y[-1]],c=['green','red'],zorder=3)
axes[0].set(xlabel='Gazebo world x (m)',ylabel='Gazebo world y (m)',title='Actual motion: arrival, then canceled return')
axes[0].axis('equal');axes[0].legend();axes[0].grid(alpha=.3)
axes[1].plot(t,[p['speed'] for p in records])
axes[1].set(xlabel='Elapsed simulation time (s)',ylabel='Ground-truth planar speed (m/s)',title='Physical stop after action cancellation')
axes[1].grid(alpha=.3)
fig.suptitle(f"Nav2 status: SUCCEEDED -> CANCELED | map goal error {result['arrival']['error_m']:.3f} m")
fig.savefig(root/'trajectory.png',dpi=160)
fig.savefig(root/'trajectory.svg')
print(root/'trajectory.png')
