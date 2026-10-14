#!/usr/bin/env python3
"""Regression for cleanup with live sensors and no previously submitted goal."""
import argparse
import json
import math
import time
from pathlib import Path
import rclpy
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--output',required=True)
args=parser.parse_args()
out=Path(args.output).resolve()
if not out.is_relative_to(Path('/work/artifacts')):raise ValueError('Invalid output')
out.mkdir(parents=True,exist_ok=True)
rclpy.init();node=rclpy.create_node('stage2_idle_verifier')
count=0;maximum=0.0

def sample(msg):
    global count,maximum
    count+=1
    maximum=max(maximum,math.hypot(msg.twist.twist.linear.x,msg.twist.twist.linear.y))

node.create_subscription(Odometry,'/red_standard_robot1/chassis_odometry_gt',sample,qos_profile_sensor_data)
try:
    client=node.create_client(GetState,'/red_standard_robot1/bt_navigator/get_state')
    if not client.wait_for_service(timeout_sec=60):raise RuntimeError('Navigator absent')
    deadline=time.monotonic()+90
    while time.monotonic()<deadline:
        future=client.call_async(GetState.Request())
        rclpy.spin_until_future_complete(node,future,timeout_sec=3)
        if future.done() and future.result() and future.result().current_state.id==3:break
    else:raise RuntimeError('Navigator not active')
    count=0;maximum=0.0
    deadline=time.monotonic()+5
    while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.1)
    if count<20 or maximum>.05:raise RuntimeError(f'Idle evidence: samples={count}, max speed={maximum}')
    report={'navigator_active':True,'goal_submitted':False,'stationary_seconds':5,
            'ground_truth_samples':count,'max_ground_truth_speed_m_s':maximum}
    (out/'idle-results.json').write_text(json.dumps(report,indent=2)+'\n')
    print('PASS active navigation remains stationary before idle shutdown')
finally:
    node.destroy_node();rclpy.shutdown()
