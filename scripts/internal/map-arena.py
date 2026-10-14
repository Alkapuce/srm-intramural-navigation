#!/usr/bin/env python3
"""Collect a sensor-derived exploratory PCD over a complete autonomous lap."""

import argparse
import json
import math
import signal
import subprocess
import time
from pathlib import Path

import numpy as np
import rclpy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from std_srvs.srv import Trigger
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from tf2_ros import Buffer, TransformListener

p = argparse.ArgumentParser()
p.add_argument("--output", required=True)
a = p.parse_args()
root = Path(__file__).resolve().parents[2]
out = Path(a.output).resolve()
if not out.is_relative_to(root / "artifacts"):
    raise ValueError("Output must be under artifacts")
out.mkdir(parents=True, exist_ok=True)
subprocess.run(["python3", str(root / "scripts/internal/snapshot-evidence.py"), "--output", str(out)], check=True)
c = json.loads((root / "config/arena.json").read_text())
ns = "/" + c["namespace"] + "/"
log = (out / "system.log").open("w")
proc = subprocess.Popen(["python3", str(root / "scripts/internal/arena-process.py"), "--localization",
    "sensors", "--transport", "direct", "--no-gui"], stdout=log,
    stderr=subprocess.STDOUT, start_new_session=True)
rclpy.init(args=["--ros-args", "-r", "/tf:=" + ns + "tf", "-r", "/tf_static:=" + ns + "tf_static"])
node = rclpy.create_node("arena_mapping_recorder")
tf = Buffer(node=node)
listener = TransformListener(tf, node)
status = {}
voxels = set()
frames = 0
clouds = 0
skipped_tf = 0


def receive_status(msg):
    status.clear()
    status.update(json.loads(msg.data))


def cloud(msg):
    global frames, clouds, skipped_tf
    frames += 1
    if frames % 10 or len(voxels) > 500000:
        return
    fields = {f.name: f for f in msg.fields}
    if any(k not in fields or fields[k].datatype != 7 for k in ("x", "y", "z")):
        return
    try:
        t = tf.lookup_transform("map", msg.header.frame_id, rclpy.time.Time())
    except Exception:
        skipped_tf += 1
        return
    dtype = np.dtype({"names": ["x", "y", "z"], "formats": ["<f4"] * 3,
        "offsets": [fields[k].offset for k in ("x", "y", "z")],
        "itemsize": msg.point_step})
    data = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=msg.data,
        strides=(msg.row_step, msg.point_step))
    xyz = np.column_stack([data[k].reshape(-1)[::8] for k in ("x", "y", "z")])
    xyz = xyz[np.isfinite(xyz).all(axis=1)]
    q = t.transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    rotation = np.array([
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)]])
    xyz = xyz @ rotation.T + np.array([t.transform.translation.x,
        t.transform.translation.y, t.transform.translation.z])
    for point in np.rint(xyz / .1).astype(np.int32):
        voxels.add(tuple(map(int, point)))
    clouds += 1


node.create_subscription(String, ns + "mission/status", receive_status,
    QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
node.create_subscription(PointCloud2, ns + "registered_scan", cloud, qos_profile_sensor_data)
client = node.create_client(Trigger, ns + "mission/start")


def until(check, seconds, label):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if proc.poll() is not None:
            raise RuntimeError("Arena exited during " + label)
        rclpy.spin_once(node, timeout_sec=.05)
        if status.get("state") == "FAULT":
            raise RuntimeError("Mission fault during mapping: " + str(status))
        if check():
            return
    raise RuntimeError(f"Timeout {label}: {status}")


failure = None
try:
    until(lambda: status.get("state") == "IDLE" and status.get("ready"), 120, "ready")
    if not client.wait_for_service(timeout_sec=10):
        raise RuntimeError("Mission start service missing")
    future = client.call_async(Trigger.Request())
    until(future.done, 10, "mission start")
    if not future.result().success:
        raise RuntimeError(future.result().message)
    until(lambda: status.get("state") == "DONE", 180, "full mapping lap")
    if len(voxels) < 1000 or clouds < 10 or skipped_tf > clouds:
        raise RuntimeError("Insufficient transformed sensor cloud")
    points = (np.asarray(sorted(voxels), dtype="<f4") * .1).astype("<f4")
    header = (f"# Sensor-derived exploratory PCD, mapped using current GICP prior\n"
        f"VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
        f"WIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {len(points)}\nDATA binary\n")
    (out / "sensor-survey.pcd").write_bytes(header.encode() + points.tobytes())
    (out / "map-results.json").write_text(json.dumps({"points": len(points),
        "clouds": clouds, "frames": frames, "skipped_tf": skipped_tf,
        "mission_history": [x["phase"] for x in status["history"]],
        "note": "Exploratory registered cloud; navigation still initialized from analytic prior"}, indent=2) + "\n")
except Exception as exc:
    failure = str(exc)
finally:
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    proc.send_signal(signal.SIGINT)
    try:
        proc.wait(timeout=120)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait()
        failure = failure or "Forced shutdown"
    log.close()
    if proc.returncode:
        failure = failure or f"Arena exit {proc.returncode}"
if failure:
    raise RuntimeError(failure)
print("PASS exploratory sensor map: " + str(out / "sensor-survey.pcd"), flush=True)
