#!/usr/bin/env python3
"""Own a fresh full-navigation run and validate goal, cancel, graph and shutdown."""
import argparse
import json
import signal
import subprocess
from pathlib import Path

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--output',default='/work/artifacts/stage2/end-to-end')
parser.add_argument('--no-rviz',action='store_true')
parser.add_argument('--idle',action='store_true',help='Validate active standby and shutdown without sending a goal')
args=parser.parse_args()
out=Path(args.output).resolve()
if not out.is_relative_to(Path('/work/artifacts')):raise ValueError('Invalid output')
out.mkdir(parents=True,exist_ok=True)
(out/'shutdown.json').unlink(missing_ok=True)
with (out/'system.log').open('w') as log:
    command=['python3','/work/scripts/internal/navigation-process.py']
    if args.no_rviz:command.append('--no-rviz')
    process=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    try:
        verifier='verify-idle-navigation.py' if args.idle else 'verify-navigation.py'
        for script,timeout in [(verifier,360),('inspect-navigation.py',120)]:
            with (out/(script.removesuffix('.py')+'.log')).open('w') as record:
                subprocess.run(['python3','/work/scripts/validation/'+script,'--output',str(out)],
                    stdout=record,stderr=subprocess.STDOUT,check=True,timeout=timeout)
        if not args.idle:
            subprocess.run(['python3','/work/scripts/validation/plot-navigation.py','--input',str(out)],
                           check=True,timeout=30)
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:process.wait(timeout=150)
            except subprocess.TimeoutExpired:
                process.kill();process.wait()
                raise RuntimeError('Full navigation shutdown needed forced termination')
    log.flush()
text=(out/'system.log').read_text()
if process.returncode!=0:raise RuntimeError(f'System supervisor failed: {process.returncode}')
markers=['process has died','Segmentation fault','Aborted','needed forced termination',
         'abnormal exit','escalating to','Traceback (most recent call last)', 'terminate called']
if any(marker in text for marker in markers):
    raise RuntimeError('A system child failed or shutdown escalated; inspect system.log')
for marker in ('Gazebo server: clean exit 0','Gazebo GUI: clean exit 0',
               'ROS simulation nodes: clean exit 0','Navigation: clean exit 0'):
    if marker not in text:raise RuntimeError(f'Missing shutdown evidence: {marker}')
if not args.no_rviz and 'Navigation RViz: clean exit 0' not in text:
    raise RuntimeError('Missing RViz shutdown evidence')
(out/'shutdown.json').write_text(json.dumps({'exit_code':process.returncode,
    'child_failures':False,'forced_termination':False,
    'rviz_map_shader_diagnostic':'active samplers with a different type' in text},indent=2)+'\n')
mode='active standby without a goal' if args.idle else 'goal and moving-task cancellation'
print(f'PASS: {mode}, sensor pipeline, TF authorities, clocks and clean shutdown; {out}')
