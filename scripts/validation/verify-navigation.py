#!/usr/bin/env python3
"""Verify a real sensor-based Nav2 goal, motion and cancel in the RM simulator."""
import argparse
import json
import math
import time
from pathlib import Path

import rclpy
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry, Path as NavPath, OccupancyGrid
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud2, Imu
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener
from action_msgs.msg import GoalStatus
from lifecycle_msgs.srv import GetState

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--output', default='/work/artifacts/stage2/navigation')
parser.add_argument('--goal-x', type=float, default=1.0)
parser.add_argument('--goal-y', type=float, default=0.0)
args = parser.parse_args()
out = Path(args.output).resolve()
if not out.is_relative_to(Path('/work/artifacts')):
    raise ValueError('Output must stay under /work/artifacts')
out.mkdir(parents=True, exist_ok=True)
(out / 'results.json').unlink(missing_ok=True)
rclpy.init(args=['--ros-args', '-r', '/tf:=/red_standard_robot1/tf',
                 '-r', '/tf_static:=/red_standard_robot1/tf_static'])
node = rclpy.create_node('stage2_navigation_verifier')
node.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
buffer = Buffer(node=node)
listener = TransformListener(buffer, node)
ns = '/red_standard_robot1/'
counts, latest, commands, edges = {}, {}, {}, {}
subscriptions = []
trajectory = (out / 'trajectory.jsonl').open('w')

def record(key, msg):
    counts[key] = counts.get(key, 0) + 1
    latest[key] = msg
    if key.startswith('cmd_vel'):
        commands[key] = max(commands.get(key, 0), math.hypot(msg.linear.x, msg.linear.y))
    if key == 'ground_truth':
        p = msg.pose.pose.position
        trajectory.write(json.dumps({'wall_time': time.monotonic(),
            'sim_time': msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9,
            'x': p.x, 'y': p.y, 'speed': math.hypot(msg.twist.twist.linear.x,
                                                     msg.twist.twist.linear.y)}) + '\n')
    if key in ('tf', 'tf_static'):
        for t in msg.transforms:
            edges[t.child_frame_id] = t.header.frame_id

for key, typ, topic in [
    ('ground_truth', Odometry, 'chassis_odometry_gt'), ('odometry', Odometry, 'odometry'),
    ('lio', Odometry, 'aft_mapped_to_init'), ('plan', NavPath, 'plan'),
    ('local_plan', NavPath, 'local_plan'), ('lidar', PointCloud2, 'livox/lidar'),
    ('converted_lidar', PointCloud2, 'velodyne_points'), ('imu', Imu, 'livox/imu'),
    ('terrain', PointCloud2, 'terrain_map'), ('registered', PointCloud2, 'registered_scan'),
    ('cmd_vel', Twist, 'cmd_vel'), ('cmd_vel_controller', Twist, 'cmd_vel_controller'),
    ('cmd_vel_nav2_result', Twist, 'cmd_vel_nav2_result'), ('tf', TFMessage, 'tf')]:
    subscriptions.append(node.create_subscription(typ, ns + topic,
        lambda msg, key=key: record(key, msg), qos_profile_sensor_data))
for key, typ, topic in [('map', OccupancyGrid, 'map'), ('tf_static', TFMessage, 'tf_static')]:
    subscriptions.append(node.create_subscription(typ, ns + topic,
        lambda msg, key=key: record(key, msg),
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)))
client = ActionClient(node, NavigateToPose, ns + 'navigate_to_pose')
active_goal = None

def spin_until(condition, timeout, label):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if condition():
            return
    raise RuntimeError(f'Timeout: {label}; received {counts}')

def wait(future, timeout, label):
    spin_until(future.done, timeout, label)
    return future.result()

def pose():
    t = buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
    return [t.transform.translation.x, t.transform.translation.y]

def tf_age():
    stamp = buffer.lookup_transform('map','base_footprint',rclpy.time.Time()).header.stamp
    return node.get_clock().now().nanoseconds/1e9 - stamp.sec - stamp.nanosec/1e9

def wait_stopped(label):
    since = None
    stopped_at_sim = None
    def stopped():
        nonlocal since, stopped_at_sim
        v = latest['cmd_vel']
        gt = latest['ground_truth'].twist.twist
        quiet = math.hypot(v.linear.x,v.linear.y)<.01 and abs(v.angular.z)<.01 and math.hypot(gt.linear.x,gt.linear.y)<.05
        if quiet and since is None:
            since = time.monotonic()
            stopped_at_sim = node.get_clock().now().nanoseconds/1e9
        elif not quiet:
            since = stopped_at_sim = None
        return since is not None and time.monotonic()-since>2
    spin_until(stopped,15,label)
    # Check a pose sampled after physical stopping, without restamping sensor data.
    spin_until(lambda: node.get_clock().now().nanoseconds/1e9-tf_age()>stopped_at_sim,
               5,'TF sampled after physical stop')

def send(x, y):
    global active_goal
    goal = NavigateToPose.Goal()
    goal.pose.header.frame_id = 'map'
    goal.pose.header.stamp = node.get_clock().now().to_msg()
    goal.pose.pose.position.x = float(x)
    goal.pose.pose.position.y = float(y)
    goal.pose.pose.orientation.w = 1.0
    active_goal = wait(client.send_goal_async(goal), 15, 'goal acceptance')
    if not active_goal.accepted:
        raise RuntimeError('Nav2 rejected goal')
    return active_goal

try:
    spin_until(lambda: all(k in latest for k in ('ground_truth', 'lio', 'odometry',
        'lidar', 'converted_lidar', 'imu', 'terrain', 'registered', 'map')) and
        buffer.can_transform('map', 'base_footprint', rclpy.time.Time()), 120, 'sensor/TF readiness')
    # Action endpoints are discoverable before lifecycle activation completes.
    state_client = node.create_client(GetState, ns + 'bt_navigator/get_state')
    if not state_client.wait_for_service(timeout_sec=30):
        raise RuntimeError('Navigator lifecycle service unavailable')
    ready_deadline = time.monotonic() + 60
    while time.monotonic() < ready_deadline:
        state = wait(state_client.call_async(GetState.Request()), 5, 'navigator lifecycle state')
        if state.current_state.id == 3:
            break
        for _ in range(10):rclpy.spin_once(node, timeout_sec=0.05)
    else:
        raise RuntimeError('Navigator never became lifecycle active')
    node.destroy_client(state_client)
    if not client.wait_for_server(timeout_sec=15):
        raise RuntimeError('NavigateToPose action unavailable')
    initial = pose()
    gt = latest['ground_truth'].pose.pose.position
    initial_gt = [gt.x, gt.y]
    handle = send(args.goal_x, args.goal_y)
    result = wait(handle.get_result_async(), 180, 'goal result')
    if result.status != GoalStatus.STATUS_SUCCEEDED:
        raise RuntimeError(f'Goal failed with action status {result.status}')
    immediate = {'pose':pose(),'tf_age_sim_seconds':tf_age()}
    wait_stopped('zero command and stationary simulator after arrival')
    reached = pose()
    error = math.hypot(reached[0] - args.goal_x, reached[1] - args.goal_y)
    gt = latest['ground_truth'].pose.pose.position
    moved = math.hypot(gt.x - initial_gt[0], gt.y - initial_gt[1])
    if error > 0.2 or moved < 0.5:
        raise RuntimeError(f'Goal evidence insufficient: error={error}, true displacement={moved}')
    if not all(commands.get(k, 0) > 0.05 for k in
               ('cmd_vel', 'cmd_vel_controller', 'cmd_vel_nav2_result')):
        raise RuntimeError('No nonzero output through full velocity chain')
    if not all(counts.get(k, 0) > 0 for k in ('plan', 'local_plan')):
        raise RuntimeError('No actual global/local path')
    first = {'status': result.status, 'initial_pose': initial,
        'goal': [args.goal_x, args.goal_y], 'reached_pose': reached,
        'error_m': error, 'ground_truth_displacement_m': moved,
        'immediate_result_observation':immediate,'final_tf_age_sim_seconds':tf_age(),
        'ground_truth_stopped_for_seconds':2}
    print(json.dumps({'arrival': first}), flush=True)
    gt = latest['ground_truth'].pose.pose.position
    cancel_start = [gt.x, gt.y]
    handle = send(*initial)
    spin_until(lambda: math.hypot(latest['cmd_vel'].linear.x,
        latest['cmd_vel'].linear.y) > 0.05 and math.hypot(
        latest['ground_truth'].pose.pose.position.x - cancel_start[0],
        latest['ground_truth'].pose.pose.position.y - cancel_start[1]) > 0.05,
        20, 'actual motion before cancellation')
    gt = latest['ground_truth'].pose.pose.position
    cancel_movement = math.hypot(gt.x - cancel_start[0], gt.y - cancel_start[1])
    cancel = wait(handle.cancel_goal_async(), 10, 'cancel acknowledgement')
    if not cancel.goals_canceling:
        raise RuntimeError('Cancellation was not accepted')
    canceled = wait(handle.get_result_async(), 15, 'canceled result')
    if canceled.status != GoalStatus.STATUS_CANCELED:
        raise RuntimeError(f'Cancel result status={canceled.status}')
    wait_stopped('zero command and stationary simulator after cancel')
    graph = {'nodes': node.get_node_names_and_namespaces(),
        'topics': node.get_topic_names_and_types(), 'tf_edges': edges,
        'publishers': {topic: [{'name': e.node_name, 'namespace': e.node_namespace}
            for e in node.get_publishers_info_by_topic(ns + topic)]
            for topic in ('tf', 'tf_static', 'odometry', 'chassis_odometry_gt',
                          'cmd_vel', 'cmd_vel_controller', 'cmd_vel_nav2_result', 'plan')}}
    (out / 'graph.json').write_text(json.dumps(graph, indent=2) + '\n')
    report = {'arrival': first, 'cancellation': {'status': canceled.status,
        'zero_command': True, 'movement_before_cancel_m': cancel_movement,
        'ground_truth_stopped_for_seconds': 2},
        'topic_samples': counts, 'max_linear_commands': commands, 'tf_edges': edges,
        'localization_mode': 'Point-LIO + small_gicp (no ground-truth localization input)'}
    (out / 'results.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report), flush=True)
    active_goal = None
finally:
    if active_goal is not None and active_goal.status not in (
            GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED):
        try:
            wait(active_goal.cancel_goal_async(), 5, 'cleanup cancel')
        except Exception as exc:
            print(f'Cleanup cancellation failed: {exc}', flush=True)
    trajectory.close()
    node.destroy_node()
    rclpy.shutdown()
