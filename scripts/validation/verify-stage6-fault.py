#!/usr/bin/env python3
"""Inject real process stalls or a simulated wire break during a moving mission."""

import argparse
import json
import math
import os
import re
import signal
import subprocess
import time
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_srvs.srv import Trigger, SetBool
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data

p = argparse.ArgumentParser()
p.add_argument("--output", required=True)
p.add_argument("--fault", choices=["pointcloud", "localization", "tf", "control", "planner", "transport_link"], required=True)
a = p.parse_args()
out = Path(a.output).resolve()
if not out.is_relative_to(Path("/work/artifacts")):
    raise ValueError("Evidence must be under artifacts")
out.mkdir(parents=True, exist_ok=True)
subprocess.run(["python3", "/work/scripts/internal/snapshot-evidence.py", "--output", str(out)], check=True)
c = json.loads(Path(os.environ.get('INTRAMURAL_ARENA_CONFIG', '/work/config/arena.json')).read_text())
ns = "/" + c["namespace"] + "/"
latest, status = {}, {}
system = (out / "system.log").open("w")
process = subprocess.Popen(["python3", "/work/scripts/internal/arena-process.py",
    "--localization", "sensors", "--transport", "serial-sim", "--no-gui"]+
    ([] if c.get('navigation_engine')=='trajectory' else ['--variant','primitives']),
    stdout=system, stderr=subprocess.STDOUT, start_new_session=True)
rclpy.init()
node = rclpy.create_node("stage6_fault_verifier")
node.create_subscription(String, ns + "mission/status", lambda m: status.update(json.loads(m.data)),
    QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
for key, typ, topic in [("truth", Odometry, "chassis_odometry_gt"),
                         ("cmd", Twist, "cmd_vel"),
                         ("transport", Twist, "cmd_vel_transport")]:
    node.create_subscription(typ, ns + topic,
        lambda m, k=key: latest.__setitem__(k, (m, time.monotonic())),
        qos_profile_sensor_data)
start = node.create_client(Trigger, ns + "mission/start")
cancel = node.create_client(Trigger, ns + "mission/cancel")
link = node.create_client(SetBool, ns + "base/link_enabled")
stopped_pid = None
wire_broken = False
failure = None
result = {"fault": a.fault}


def until(check, seconds, label):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError(f"Arena exited during {label}")
        rclpy.spin_once(node, timeout_sec=.04)
        if check():
            return
    raise RuntimeError(f"Timeout {label}: {status}")


def call(client, request):
    if not client.wait_for_service(timeout_sec=10):
        raise RuntimeError("Service unavailable")
    future = client.call_async(request)
    until(future.done, 180 if client is start else 10, "service reply")
    response = future.result()
    if not response.success:
        raise RuntimeError("Service rejected: " + response.message)


def physical_speed():
    return math.hypot(latest["truth"][0].twist.twist.linear.x,
                      latest["truth"][0].twist.twist.linear.y)


def velocity(key):
    m = latest[key][0]
    return math.hypot(m.linear.x, m.linear.y)


def position():
    p = latest["truth"][0].pose.pose.position
    return (p.x, p.y)


def child_pid(label):
    system.flush()
    text = (out / "system.log").read_text()
    match = re.search(r"\[INFO\] \[" + re.escape(label) + r"-\d+\]: process started with pid \[(\d+)\]", text)
    if not match:
        raise RuntimeError("Exact child PID not found: " + label)
    pid = int(match.group(1))
    if not Path(f"/proc/{pid}").exists():
        raise RuntimeError("Child already exited")
    return pid


try:
    until(lambda: status.get("ready") and status.get("state") == "IDLE" and "truth" in latest,
          120, "arena ready")
    call(start, Trigger.Request())
    until(lambda: status.get("state") == "RUNNING" and physical_speed() > .15,
          30, "actual motion")
    result["position_before"] = position()
    result["speed_before_mps"] = physical_speed()
    if a.fault == "transport_link":
        call(link, SetBool.Request(data=False))
        wire_broken = True
        # Cancellation closes software gate, but an unprotected MCU keeps the last frame.
        call(cancel, Trigger.Request())
        until(lambda: status.get("state") in ("CANCELLED", "FAULT"), 10, "mission cancellation")
        if status.get('state')=='FAULT' and status.get('reason')!='STOP_UNCONFIRMED':
            raise RuntimeError('Unexpected wire-break cancellation fault: '+status.get('reason',''))
        until(lambda: velocity("cmd") < .01, 3, "upstream zero")
        start_pos = position()
        until(lambda: math.dist(position(), start_pos) > .06, 2, "retained physical motion")
        result["unsafe_hold_after_cancel_m"] = math.dist(position(), start_pos)
        result["mcu_speed_after_cancel_mps"] = velocity("transport")
        result["physical_speed_after_cancel_mps"] = physical_speed()
        result["upstream_zero_after_cancel"] = velocity("cmd") < .01
        if result["mcu_speed_after_cancel_mps"] < .05 or not result["upstream_zero_after_cancel"]:
            raise RuntimeError("Wire-break boundary was not reproduced")
        call(link, SetBool.Request(data=True))
        wire_broken = False
        until(lambda: velocity("transport") < .01 and physical_speed() < .03,
              5, "physical recovery after link restored")
        result["stopped_after_link_restored"] = True
    elif a.fault=='planner':
        parent=child_pid('trajectory_mission')
        pid=status.get('planner_pid')
        if not isinstance(pid,int) or pid<=0:raise RuntimeError('Missing planning child identity')
        fields=Path(f'/proc/{pid}/status').read_text().splitlines()
        ppid=int(next(line.split()[1] for line in fields if line.startswith('PPid:')))
        if ppid!=parent:raise RuntimeError('Reported planner is not a mission child')
        result['suspended_planner_pid']=pid
        stopped_pid=pid;os.kill(stopped_pid,signal.SIGSTOP)
        route=json.loads((Path('/work/artifacts/navigation-runs')/status['run_id']/'route.json').read_text())
        target=next(point for distance,point in zip(route['s'],route['points'])
                    if distance>status['continuous_progress']+1.)
        subprocess.run(['python3','/work/scripts/internal/arena-obstacle.py','place',
            '--x',str(target[0]),'--y',str(target[1])],check=True,timeout=10)
        until(lambda:velocity('cmd')<.01 and velocity('transport')<.01 and physical_speed()<.03,
              8,'raw guard stop while planner is suspended')
        result['zero_command_and_stop']=True
        began=time.monotonic();call(cancel,Trigger.Request())
        until(lambda:status.get('state')=='CANCELLED',5,'cancellation with suspended planner')
        result['cancel_seconds']=time.monotonic()-began
        if result['cancel_seconds']>1.:raise RuntimeError('Planner suspension delayed cancellation')
        os.kill(stopped_pid,signal.SIGCONT);stopped_pid=None
    else:
        labels = {"pointcloud": "lidar_adapter", "localization": "small_gicp_relocalization_node",
                  "tf": "sensor_scan_generation_node", "control": "trajectory_mission" if c.get('navigation_engine')=='trajectory' else "velocity_smoother"}
        stopped_pid = child_pid(labels[a.fault])
        os.kill(stopped_pid, signal.SIGSTOP)
        result["suspended_child"] = labels[a.fault]
        if a.fault == "control":
            until(lambda: velocity("cmd") < .01 and velocity("transport") < .01
                  and physical_speed() < .03, 5, "control staleness zero")
            result["zero_command_and_stop"] = True
            result["state_after_injection"] = status.get("state")
            previous_guard_stamp=status.get('guard',{}).get('stamp',0.)
            os.kill(stopped_pid, signal.SIGCONT)
            stopped_pid = None
            if c.get('navigation_engine')=='trajectory':
                # The cached RUNNING report predates SIGSTOP; confirm fresh inputs before judging cancellation.
                until(lambda:status.get('guard',{}).get('stamp',0.)>previous_guard_stamp
                      and status.get('ready'),15,'fresh control status after resume')
                result['state_after_fresh_resume']=status.get('state')
                if status.get('state') not in ('RUNNING','FAULT') or (status.get('state')=='FAULT'
                        and status.get('reason')!='LOCALIZATION_OR_SENSOR_STALE'):
                    raise RuntimeError('Unexpected resumed control state: '+str(status.get('state')))
                call(cancel, Trigger.Request())
                until(lambda:status.get('state')=='CANCELLED',10,'cancel after fresh control resume')
            elif status.get("state") == "RUNNING":
                call(cancel, Trigger.Request())
                until(lambda: status.get("state") == "CANCELLED", 10, "cancel after control resume")
        else:
            until(lambda: status.get("state") == "FAULT", 8, "localization fault")
            result["fault_reason"] = status.get("reason")
            until(lambda: velocity("cmd") < .01 and velocity("transport") < .01
                  and physical_speed() < .03, 5, "fault zero and physical stop")
            result["zero_command_and_stop"] = True
            os.kill(stopped_pid, signal.SIGCONT)
            stopped_pid = None
    result["position_after"] = position()
    result["mission_state_after"] = status.get("state")
    (out / "results.json").write_text(json.dumps(result, indent=2) + "\n")
except Exception as exc:
    failure = str(exc)
    (out / "failure.json").write_text(json.dumps({"error": failure, "result": result,
        "status": status}, indent=2) + "\n")
finally:
    if stopped_pid is not None:
        os.kill(stopped_pid, signal.SIGCONT)
    if wire_broken and link.service_is_ready():
        future = link.call_async(SetBool.Request(data=True))
        rclpy.spin_until_future_complete(node, future, timeout_sec=3)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    process.send_signal(signal.SIGINT)
    try:
        process.wait(timeout=120)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait()
        failure = failure or "Forced arena shutdown"
    system.close()
    if process.returncode:
        failure = failure or f"Arena exit {process.returncode}"
    (out / "shutdown.json").write_text(json.dumps({"exit_code": process.returncode,
        "forced": process.returncode != 0}, indent=2) + "\n")
if failure:
    raise RuntimeError(failure)
print("PASS stage 6 fault boundary: " + a.fault, flush=True)
