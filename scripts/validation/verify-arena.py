#!/usr/bin/env python3
"""Independent Gazebo-truth validation of ordered mission, cancellation and parking."""

import argparse, json, math, time
from pathlib import Path
import rclpy
import numpy as np
from std_srvs.srv import Trigger
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud2, Imu
from rosgraph_msgs.msg import Clock
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from intramural_mission.route import segments, clearance

p = argparse.ArgumentParser(description=__doc__)
p.add_argument("--output", required=True)
p.add_argument("--rounds", type=int, default=5)
p.add_argument("--mode", choices=["ideal", "sensors"], default="ideal")
p.add_argument("--variant", choices=["figure8", "primitives"], default="figure8")
p.add_argument("--transport", choices=["direct", "serial-sim"], default="direct")
p.add_argument("--skip-cancel", action="store_true")
args = p.parse_args()
if args.rounds < 1:
    raise ValueError("At least one actual round is required")
out = Path(args.output).resolve()
if not out.is_relative_to(Path("/work/artifacts")):
    raise ValueError("Invalid output")
out.mkdir(parents=True, exist_ok=True)
c = json.loads(Path("/work/config/arena.json").read_text())
ns = "/" + c["namespace"] + "/"
rclpy.init()
node = rclpy.create_node("arena_verifier")
status = {}
latest = {}
received_at = {}
counts = {}
contracts = {}
point_samples = []
transport_stats = {"maximum_observed_speed_mps": 0.0, "maximum_observed_wz": 0.0}
timing = {}
command_stats = {"maximum_speed_mps": 0.0, "maximum_absolute_wz": 0.0, "nonfinite": 0}
records = []
recording = False
last_gt_wall = 0
log = (out / "trajectory.jsonl").open("w")


def receive(key, msg):
    global last_gt_wall
    counts[key] = counts.get(key, 0) + 1
    latest[key] = msg
    received_at[key] = time.monotonic()
    if key == "transport_command":
        transport_stats["maximum_observed_speed_mps"] = max(
            transport_stats["maximum_observed_speed_mps"], math.hypot(msg.linear.x, msg.linear.y))
        transport_stats["maximum_observed_wz"] = max(
            transport_stats["maximum_observed_wz"], abs(msg.angular.z))
    if key == "command":
        speed = math.hypot(msg.linear.x, msg.linear.y)
        command_stats["maximum_speed_mps"] = max(
            command_stats["maximum_speed_mps"], speed
        )
        command_stats["maximum_absolute_wz"] = max(
            command_stats["maximum_absolute_wz"], abs(msg.angular.z)
        )
        if not all(
            math.isfinite(v) for v in [msg.linear.x, msg.linear.y, msg.angular.z]
        ):
            command_stats["nonfinite"] += 1
    if key in ("lidar", "imu", "converted_lidar"):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        t = timing.setdefault(
            key,
            {
                "samples": 0,
                "backwards": 0,
                "repeated": 0,
                "first": stamp,
                "last": stamp,
            },
        )
        if t["samples"]:
            t["backwards"] += int(stamp < t["last"])
            t["repeated"] += int(stamp == t["last"])
        t["last"] = stamp
        t["samples"] += 1
        contract = {
            "frame": msg.header.frame_id,
            "stamp": msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9,
        }
        if hasattr(msg, "fields"):
            contract.update(
                fields=[
                    {"name": f.name, "offset": f.offset, "datatype": f.datatype}
                    for f in msg.fields
                ],
                point_step=msg.point_step,
                width=msg.width,
                height=msg.height,
            )
        contracts[key] = contract
        if key == "converted_lidar" and counts[key] % 400 == 1:
            fields = {f.name: f for f in msg.fields}
            names = ["x", "y", "z", "intensity", "ring", "time"]
            dtype = np.dtype(
                {
                    "names": names,
                    "formats": ["<f4"] * 4 + ["<u2", "<f4"],
                    "offsets": [fields[k].offset for k in names],
                    "itemsize": msg.point_step,
                }
            )
            data = np.ndarray(
                (msg.height, msg.width),
                dtype=dtype,
                buffer=msg.data,
                strides=(msg.row_step, msg.point_step),
            ).reshape(-1)
            if (
                not len(data)
                or any(
                    not np.isfinite(data[k]).all()
                    for k in ["x", "y", "z", "intensity", "time"]
                )
                or np.any(data["ring"] >= 32)
                or np.any(data["time"] <= 0)
                or np.any(data["time"] > 1e-7 + 1e-14)
            ):
                raise RuntimeError(
                    "Converted instantaneous point values/time are invalid"
                )
            point_samples.append(
                {
                    "stamp": contract["stamp"],
                    "points": len(data),
                    "ring_min": int(data["ring"].min()),
                    "ring_max": int(data["ring"].max()),
                    "time_min_seconds": float(data["time"].min()),
                    "time_max_seconds": float(data["time"].max()),
                    "nonfinite": False,
                }
            )
    if key == "truth":
        last_gt_wall = time.monotonic()
        position = msg.pose.pose.position
        q = msg.pose.pose.orientation
        item = {
            "wall": last_gt_wall,
            "sim": msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9,
            "x": position.x,
            "y": position.y,
            "yaw": math.atan2(
                2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)
            ),
            "speed": math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y),
            "phase": status.get("phase"),
            "segment": status.get("segment"),
            "run": status.get("run"),
        }
        if recording:
            records.append(item)
            log.write(json.dumps(item) + "\n")


def state(msg):
    global status
    status = json.loads(msg.data)


node.create_subscription(
    Clock, "/clock", lambda msg: receive("clock", msg), qos_profile_sensor_data
)
node.create_subscription(
    String,
    ns + "mission/status",
    state,
    QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
)
for key, typ, topic in [
    ("truth", Odometry, "chassis_odometry_gt"),
    ("odom", Odometry, "odometry"),
    ("lio", Odometry, "aft_mapped_to_init"),
    ("command", Twist, "cmd_vel"),
    ("lidar", PointCloud2, "livox/lidar"),
    ("converted_lidar", PointCloud2, "velodyne_points"),
    ("imu", Imu, "livox/imu"),
    ("transport_command", Twist, "cmd_vel_transport"),
    ("bridge_status", String, "base/status"),
    ("mcu_status", String, "base/sim_status"),
    ("feedback", String, "base/feedback"),
]:
    node.create_subscription(
        typ, ns + topic, lambda msg, key=key: receive(key, msg), qos_profile_sensor_data
    )
clients = {
    name: node.create_client(Trigger, ns + "mission/" + name)
    for name in ["start", "cancel", "reset"]
}


def until(condition, timeout, label):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if status.get("state") == "FAULT":
            current = latest.get("clock")
            clock = current.clock.sec + current.clock.nanosec / 1e9 if current else None
            ages = {
                k: {
                    "wall_age": time.monotonic() - received_at[k],
                    "sim_age": clock
                    - (msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9)
                    if clock is not None
                    else None,
                }
                for k, msg in latest.items()
                if hasattr(msg, "header")
            }
            (out / "failure.json").write_text(
                json.dumps(
                    {
                        "status": status,
                        "stream_ages": ages,
                        "counts": counts,
                        "simulation_clock": clock,
                    },
                    indent=2,
                )
                + "\n"
            )
            raise RuntimeError("Mission fault: " + str(status))
        if condition():
            return
    raise RuntimeError(f"{label}: timeout; status={status}; counts={counts}")


def call(name):
    if name == 'start':
        until(lambda: status.get('state') == 'IDLE' and status.get('ready')
              and abs(status.get('tf_age_sim_seconds', 1)) < .1,
              30, 'fresh localization before start')
    if not clients[name].wait_for_service(timeout_sec=10):
        raise RuntimeError("Missing " + name)
    future = clients[name].call_async(Trigger.Request())
    until(future.done, 10, name)
    response = future.result()
    if not response.success:
        (out / "failure.json").write_text(json.dumps({"service": name, "error": response.message, "status": status}, indent=2)+"\n")
        raise RuntimeError(f"{name}: {response.message}")
    return response


def stopped():
    start = None
    deadline = time.monotonic() + 20
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        m = latest.get("truth")
        v = latest.get("command")
        quiet = (
            m
            and v
            and time.monotonic() - last_gt_wall < 0.3
            and math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y) < 0.03
            and math.hypot(v.linear.x, v.linear.y) < 0.005
            and abs(v.angular.z) < 1e-8
        )
        start = (start or time.monotonic()) if quiet else None
        if start and time.monotonic() - start > 2:
            return
    raise RuntimeError("Actual stop not confirmed")


def winding(data, center):
    angles = [math.atan2(r["y"] - center[1], r["x"] - center[0]) for r in data]
    return sum(
        math.atan2(math.sin(b - a), math.cos(b - a)) for a, b in zip(angles, angles[1:])
    )


try:
    until(
        lambda: status.get("ready")
        and all(k in latest for k in ("truth", "odom", "command", "lidar", "imu")),
        120,
        "sensor and controller readiness",
    )
    if args.mode == "sensors":
        until(
            lambda: "lio" in latest and "converted_lidar" in latest,
            60,
            "sensor localization input",
        )
    report = {"mode": args.mode, "variant": args.variant, "rounds": []}
    for round_id in range(args.rounds):
        if round_id:
            call("reset")
            until(lambda: status.get("state") == "IDLE", 5, "reset status")
        start = len(records)
        recording = True
        initial_run = status.get("run", 0)
        call("start")
        until(
            lambda: status.get("run", 0) > initial_run
            and status.get("state") == "DONE",
            900,
            "mission completion",
        )
        stopped()
        recording = False
        data = records[start:]
        if len(data) < 50:
            raise RuntimeError("No actual trajectory")
        truth = latest["truth"].pose.pose.position
        error = math.dist([truth.x, truth.y], c["parking"])
        pc = min(clearance(c, [r["x"], r["y"]])[0] for r in data)
        bc = min(clearance(c, [r["x"], r["y"]])[1] for r in data)
        if error > c["parking_tolerance"] or min(pc, bc) <= 0:
            raise RuntimeError(
                f"Parking/collision failure: error={error}, clearances={pc, bc}"
            )
        if max(abs(r["yaw"]) for r in data) > 0.08:
            raise RuntimeError("Unexpected rotation with wz disabled")
        history = [h["phase"] for h in status["history"]]
        entry = {
            "run": status["run"],
            "duration_sim_seconds": status["elapsed_sim_seconds"],
            "parking_error_m": error,
            "min_pole_clearance_m": pc,
            "min_boundary_clearance_m": bc,
            "maximum_absolute_yaw_rad": max(abs(r["yaw"]) for r in data),
            "phases": history,
            "samples": len(data),
        }
        if args.variant == "figure8":
            expected = (
                ["ENTER"] + ["LOWER"] * 3 + ["UPPER"] * 4 + ["LOWER_EXIT", "RETURN"]
            )
            lo = winding(data, c["lower_pole"])
            hi = winding(data, c["upper_pole"])
            entry.update(lower_winding_rad=lo, upper_winding_rad=hi)
            if history != expected or not 6.0 < lo < 6.6 or not -6.6 < hi < -6.0:
                raise RuntimeError(f"Missed pole/direction/segment: {entry}")
            path = [point for _, points in segments(c) for point in points]
            deviation = max(
                min(math.dist([r["x"], r["y"]], point) for point in path)
                for r in data[::5]
            )
            entry["max_reference_deviation_m"] = deviation
            if deviation > c["max_tracking_error"]:
                raise RuntimeError(f"Tracking deviation {deviation}")
        else:
            if history != ["FORWARD", "SIDEWAYS", "CURVE", "RETURN"]:
                raise RuntimeError("Primitive ordering")
            for phase, axis in [("FORWARD", "x"), ("SIDEWAYS", "y")]:
                phase_data = [r for r in data if r["phase"] == phase]
                if (
                    max(r[axis] for r in phase_data) - min(r[axis] for r in phase_data)
                    < 0.6
                ):
                    raise RuntimeError("No " + phase + " actual motion")
        report["rounds"].append(entry)
        (out / "results.json").write_text(json.dumps(report, indent=2) + "\n")
        print(json.dumps(entry), flush=True)
    if not args.skip_cancel:
        call("reset")
        until(lambda: status.get("state") == "IDLE", 5, "reset before cancel")
        truth = latest["truth"].pose.pose.position
        start_pos = [truth.x, truth.y]
        call("start")
        until(
            lambda: math.dist(
                [
                    latest["truth"].pose.pose.position.x,
                    latest["truth"].pose.pose.position.y,
                ],
                start_pos,
            )
            > 0.15,
            30,
            "motion before cancel",
        )
        call("cancel")
        until(
            lambda: status.get("state") == "CANCELLED", 15, "action cancellation result"
        )
        stopped()
        call("reset")
        until(lambda: status.get("state") == "IDLE", 5, "reset after cancel")
        report["cancel"] = {
            "state": "CANCELLED",
            "actual_motion_before_cancel": True,
            "zero_command_and_stationary": True,
            "reset": "IDLE",
        }
    if (
        command_stats["nonfinite"]
        or command_stats["maximum_speed_mps"] > c["max_speed"] + 1e-6
        or command_stats["maximum_absolute_wz"] > 1e-8
    ):
        raise RuntimeError("Final velocity contract failed: " + str(command_stats))
    if any(
        t["backwards"] or t["samples"] < 2 or t["last"] <= t["first"]
        for t in timing.values()
    ):
        raise RuntimeError("Sensor timestamp contract failed: " + str(timing))
    for key in ["lidar", "converted_lidar"] if args.mode == "sensors" else ["lidar"]:
        fields = {f["name"]: f["datatype"] for f in contracts[key]["fields"]}
        required = {"x": 7, "y": 7, "z": 7, "intensity": 7, "ring": 4}
        if key == "converted_lidar":
            required["time"] = 7
        if contracts[key]["frame"] != "front_mid360" or any(
            fields.get(k) != v for k, v in required.items()
        ):
            raise RuntimeError(
                "Point cloud fields/frame contract failed: " + str(contracts[key])
            )
    if contracts["imu"]["frame"] != "front_mid360":
        raise RuntimeError("IMU frame contract failed")
    report["command_stats"] = command_stats
    report["sensor_timing"] = timing
    lidar_timing = timing['lidar']
    lidar_hz = (lidar_timing['samples']-1)/(lidar_timing['last']-lidar_timing['first'])
    if lidar_hz < 15:
        raise RuntimeError(f'20 Hz simulated lidar reached ROS at only {lidar_hz:.2f} Hz')
    report['observed_lidar_hz'] = lidar_hz
    report["converted_point_samples"] = point_samples
    report["topic_counts"] = counts
    report["sensor_contract"] = contracts
    if args.transport == "serial-sim":
        try:
            bridge = json.loads(latest["bridge_status"].data)
            mcu = json.loads(latest["mcu_status"].data)
            feedback = json.loads(latest["feedback"].data)
        except (KeyError, ValueError) as exc:
            raise RuntimeError("Missing simulated serial telemetry") from exc
        if (bridge["sent"] < 100 or mcu["frames"] < 100 or mcu["seq_gaps"]
            or mcu["bad_bytes"] or not mcu["no_mcu_watchdog"]
            or feedback["cmd_id"] != 1 or feedback["values"]["stage_remain_time"] != 300
            or bridge["max_interval_wall_seconds"] > .03
            or bridge["late_intervals_over_20ms"] > 10
            or transport_stats["maximum_observed_speed_mps"] < 0.1
            or transport_stats["maximum_observed_wz"] > 1e-8
            or any(abs(v) > 0.03 for v in mcu["command"])):
            raise RuntimeError("Serial transport contract failed: " + str((bridge, mcu, transport_stats)))
        report["transport"] = {"bridge": bridge, "sim_mcu": mcu,
            "feedback": feedback, **transport_stats}
    (out / "results.json").write_text(json.dumps(report, indent=2) + "\n")
    print("PASS arena mission and physical validation", flush=True)
finally:
    log.close()
    node.destroy_node()
    rclpy.shutdown()
