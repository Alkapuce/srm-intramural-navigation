#!/usr/bin/env python3
"""Verify removable/persistent obstacles and cancellation using independent physical truth."""

import argparse
import json
import math
import signal
import subprocess
import time
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_srvs.srv import Trigger
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from intramural_mission.route import clearance, segments
from intramural_mission.planning import compile_route


p = argparse.ArgumentParser()
p.add_argument("--output", required=True)
p.add_argument("--mode", choices=["ideal", "sensors"], default="sensors")
p.add_argument("--transport", choices=["direct", "serial-sim"], default="serial-sim")
p.add_argument("--scenarios", nargs="+", choices=["entry", "lower", "upper", "cross"],
               default=["entry", "lower", "upper", "cross"])
p.add_argument("--blocked-hold-seconds", type=float, default=1.5)
p.add_argument("--persistent", action="store_true", help="Keep the obstacle until the whole lap completes")
p.add_argument('--cancel-bypass', action='store_true')
p.add_argument('--expect-blocked', action='store_true', help='Insert a wall across the bounded corridor, then cancel')
a = p.parse_args()
if (a.cancel_bypass or a.expect_blocked) and (not a.persistent or a.scenarios != ['entry']):
    p.error('Cancellation/no-route checks require --persistent --scenarios entry')
if not 0 < a.blocked_hold_seconds <= 60:
    p.error("--blocked-hold-seconds must be in (0, 60]")
c = json.loads(Path("/work/config/arena.json").read_text())
if c.get('navigation_engine') == 'trajectory' and a.persistent and not a.expect_blocked:
    p.error('Trajectory engine waits for obstacles; use --expect-blocked or a legacy bypass configuration')
out = Path(a.output).resolve()
if not out.is_relative_to(Path("/work/artifacts")):
    raise ValueError("Evidence must be under artifacts")
out.mkdir(parents=True, exist_ok=True)
subprocess.run(["python3", "/work/scripts/internal/snapshot-evidence.py", "--output", str(out)], check=True)
ns = "/" + c["namespace"] + "/"
route=segments(c)
targets={"entry":(0,route[0][1][3*len(route[0][1])//4]),
         "lower":(2,route[2][1][len(route[2][1])//2]),
         "upper":(4,route[4][1][len(route[4][1])//2]),
         "cross":(3,route[3][1][-1])}
status, latest = {}, {}
records = []
trajectory = (out / "trajectory.jsonl").open("w")
system = (out / "system.log").open("w")
process = subprocess.Popen(["python3", "/work/scripts/internal/arena-process.py",
    "--localization", a.mode, "--transport", a.transport, "--no-gui"],
    stdout=system, stderr=subprocess.STDOUT, start_new_session=True)
rclpy.init()
node = rclpy.create_node("stage6_obstacle_verifier")


def state(msg):
    status.clear()
    status.update(json.loads(msg.data))


def receive(key, msg):
    latest[key] = msg
    if key == "truth":
        p = msg.pose.pose.position
        item = {"wall": time.monotonic(), "run": status.get("run"),
                "phase": status.get("phase"), "segment": status.get("segment"),
                "state": status.get("state"), "x": p.x, "y": p.y,
                "speed": math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)}
        records.append(item)
        trajectory.write(json.dumps(item) + "\n")


node.create_subscription(String, ns + "mission/status", state,
    QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
node.create_subscription(OccupancyGrid,ns+'map',lambda msg: receive('map',msg),
    QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
for key, typ, name in [("truth", Odometry, "chassis_odometry_gt"),
                       ("cmd", Twist, "cmd_vel"),
                       ("transport", Twist, "cmd_vel_transport")]:
    node.create_subscription(typ, ns + name, lambda msg, k=key: receive(k, msg),
                             qos_profile_sensor_data)
clients = {name: node.create_client(Trigger, ns + "mission/" + name)
           for name in ("start", "cancel", "reset")}


def until(check, seconds, label):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError(f"Arena exited during {label}")
        rclpy.spin_once(node, timeout_sec=.05)
        if status.get("state") == "FAULT":
            raise RuntimeError(f"Mission fault during {label}: {status}")
        if check():
            return
    raise RuntimeError(f"{label} timed out: {status}")


def call(name):
    if name == 'start':
        until(lambda: status.get('state') == 'IDLE' and status.get('ready')
              and abs(status.get('tf_age_sim_seconds',1)) < .1,
              30,'fresh localization before start')
    if not clients[name].wait_for_service(timeout_sec=10):
        raise RuntimeError("Missing mission service " + name)
    future = clients[name].call_async(Trigger.Request())
    until(future.done, 10, name)
    result = future.result()
    if not result.success:
        raise RuntimeError(f"{name}: {result.message}")


def move(operation, target=None):
    if a.expect_blocked and operation == 'place':
        sdf = '<sdf version="1.7"><model name="blocked_corridor"><static>true</static><pose>1.5 2.25 .5 0 0 0</pose><link name="wall"><collision name="collision"><geometry><box><size>.5 8.5 1</size></box></geometry></collision><visual name="visual"><geometry><box><size>.5 8.5 1</size></box></geometry></visual></link></model></sdf>'
        request = 'sdf: '+json.dumps(sdf)
        r = subprocess.run(['ign','service','-s','/world/default/create','--reqtype','ignition.msgs.EntityFactory',
            '--reptype','ignition.msgs.Boolean','--timeout','3000','--req',request],capture_output=True,text=True,check=True)
        if 'true' not in r.stdout:
            raise RuntimeError('Wall insertion not acknowledged')
        return
    command = ["python3", "/work/scripts/internal/arena-obstacle.py", operation]
    if target:
        command.extend(["--x", str(target[0]), "--y", str(target[1])])
    subprocess.run(command, timeout=10, check=True)


def stopped(seconds=1.0):
    since = None
    deadline = time.monotonic() + 10
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.05)
        truth, cmd = latest.get("truth"), latest.get("cmd")
        transport = latest.get("transport")
        quiet = (truth and cmd and math.hypot(truth.twist.twist.linear.x,
            truth.twist.twist.linear.y) < .03 and
            math.hypot(cmd.linear.x, cmd.linear.y) < .01)
        if a.transport == "serial-sim":
            quiet = quiet and transport and math.hypot(transport.linear.x,
                transport.linear.y) < .01
        since = (since or time.monotonic()) if quiet else None
        if since and time.monotonic() - since >= seconds:
            return
    raise RuntimeError("Physical stop/zero command not sustained")


def hold_blocked(seconds):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.05)
        if status.get("state") != "BLOCKED":
            raise RuntimeError("Blocked obstacle state was not maintained")
        truth, cmd = latest.get("truth"), latest.get("cmd")
        transport = latest.get("transport")
        if not truth or not cmd or math.hypot(truth.twist.twist.linear.x,
                truth.twist.twist.linear.y) >= .03 or math.hypot(cmd.linear.x,
                cmd.linear.y) >= .01:
            raise RuntimeError("Vehicle moved during sustained obstacle blocking")
        if a.transport == "serial-sim" and (not transport or math.hypot(
                transport.linear.x, transport.linear.y) >= .01):
            raise RuntimeError("Lower-controller command resumed during blocking")


results = []
failure = None
try:
    until(lambda: status.get("state") == "IDLE" and status.get("ready") and latest.get("truth"),
          120, "startup readiness")
    if c.get('navigation_engine')=='trajectory':
        until(lambda: 'map' in latest,10,'static map for obstacle placement')
        plan,route,_=compile_route(c,latest['map'])
        targets={"entry":(0,route[0][1][3*len(route[0][1])//4]),
                 "lower":(2,route[2][1][len(route[2][1])//2]),
                 "upper":(4,route[4][1][len(route[4][1])//2]),
                 "cross":(3,route[3][1][-1])}
    for scenario in a.scenarios:
        segment, target = targets[scenario]
        if status["state"] == "DONE":
            call("reset")
            until(lambda: status.get("state") == "IDLE", 5, "reset")
        call("start")
        until(lambda: status.get("state") == "RUNNING" and status.get("segment",-1) <= segment
              and latest.get("truth") and math.hypot(latest["truth"].twist.twist.linear.x,
                  latest["truth"].twist.twist.linear.y) > .1
              and math.dist((latest["truth"].pose.pose.position.x,
                  latest["truth"].pose.pose.position.y), c["parking"]) > .2,
              120, f"{scenario} entry")
        moving_pose = latest["truth"].pose.pose.position
        moving_pose = (moving_pose.x, moving_pose.y)
        before = list(status["history"])
        move("place", target)
        inserted_at = time.monotonic()
        until(lambda: status.get("state") == "BLOCKED", 120, f"{scenario} block")
        blocked_segment = status["segment"]
        history=status['history']
        if history[:len(before)]!=before or [h['segment']for h in history]!=list(range(len(history))):
            raise RuntimeError("Obstacle skipped an ordered phase")
        segment=blocked_segment
        frozen_history=list(history)
        stopped(1.5)
        hold_blocked(a.blocked_hold_seconds)
        if status['segment']!=segment or status['history']!=frozen_history:
            raise RuntimeError('Progress advanced during blocking')
        p = latest["truth"].pose.pose.position
        stand_off = math.dist((p.x, p.y), target)
        if stand_off < .7:
            raise RuntimeError(f"Unsafe obstacle standoff {stand_off}")
        blocked_pose = (p.x, p.y)
        if a.expect_blocked:
            hold_blocked(12)
            if status.get('bypass_count',0):
                raise RuntimeError('Planner attempted to pass an impassable wall')
            call('cancel')
            until(lambda: status.get('state') == 'CANCELLED',10,'blocked cancellation')
            stopped(2)
            results.append({'scenario':'no-route','state':status['state'],'bypass_count':0,'stopped':True})
            (out/'results.json').write_text(json.dumps(results,indent=2)+'\n')
            break
        if not a.persistent:
            move("park")
        until(lambda: status.get("state") == "RUNNING" and status.get("segment") == segment
              and (not a.persistent or status.get('bypass_count',0) > 0),
              35, f"{scenario} same-segment resume")
        if a.cancel_bypass:
            until(lambda: math.hypot(latest['truth'].twist.twist.linear.x,
                latest['truth'].twist.twist.linear.y) > .08,20,'moving bypass')
            call('cancel')
            until(lambda: status.get('state') == 'CANCELLED',10,'bypass cancellation')
            stopped(2)
            results.append({'scenario':'cancel-bypass','state':status['state'],
                'bypass_count':status['bypass_count'],'stopped':True})
            (out/'results.json').write_text(json.dumps(results,indent=2)+'\n')
            break
        until(lambda: status.get("state") == "DONE", 240, f"{scenario} completion")
        stopped(1.5)
        p = latest["truth"].pose.pose.position
        parking_error = math.dist((p.x, p.y), c["parking"])
        if parking_error > c["parking_tolerance"]:
            raise RuntimeError(f"Parking error {parking_error}")
        run_records = [r for r in records if r["run"] == status["run"]]
        min_clearance = min(min(clearance(c, (r["x"], r["y"]))) for r in run_records)
        if min_clearance <= 0:
            raise RuntimeError("Physical pole/boundary collision")
        dynamic_clearance = min(math.hypot(max(abs(r['x']-target[0])-.25,0),
            max(abs(r['y']-target[1])-.25,0))-c['robot_radius'] for r in run_records
            if r['wall'] >= inserted_at) if a.persistent else None
        if dynamic_clearance is not None and dynamic_clearance < .08:
            raise RuntimeError(f'Insufficient persistent obstacle clearance: {dynamic_clearance}')
        result = {"scenario": scenario, "target": target, "blocked_segment": segment,
                  "blocked_hold_wall_seconds": a.blocked_hold_seconds,
                  "moving_pose_at_insertion": moving_pose,
                  "insertion_distance_m":math.dist(moving_pose,target),
                  "blocked_pose": blocked_pose, "obstacle_standoff_m": stand_off,
                  "parking_error_m": parking_error, "min_static_clearance_m": min_clearance,
                  "history": [h["phase"] for h in status["history"]],
                  "elapsed_sim_seconds": status["elapsed_sim_seconds"],
                  "samples": len(run_records)}
        result.update(persistent=a.persistent,bypass_count=status.get('bypass_count',0),
                      min_dynamic_clearance_m=dynamic_clearance)
        results.append(result)
        (out / "results.json").write_text(json.dumps(results, indent=2) + "\n")
        print(json.dumps(result), flush=True)
        move('park')
except Exception as exc:
    failure = str(exc)
    (out / "failure.json").write_text(json.dumps({"error": failure, "status": status,
        "last_truth": {"x": latest["truth"].pose.pose.position.x,
                       "y": latest["truth"].pose.pose.position.y} if "truth" in latest else None}, indent=2) + "\n")
finally:
    try:
        move("park")
    except Exception:
        pass
    node.destroy_node()
    rclpy.shutdown()
    trajectory.close()
    process.send_signal(signal.SIGINT)
    try:
        process.wait(timeout=120)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait()
        failure = failure or "Forced arena termination"
    system.close()
    if process.returncode:
        failure = failure or f"Arena exit {process.returncode}"
if failure:
    raise RuntimeError(failure)
print('PASS stage 6 '+('persistent obstacle/cancellation' if a.persistent else 'obstacle removal and same-segment resume'), flush=True)
