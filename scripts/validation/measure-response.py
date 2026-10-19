#!/usr/bin/env python3
"""Reuse field traces, or record bounded ground response steps; never edits active parameters."""

import argparse
from datetime import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts/internal"))
from response_fit import fit_response, stopping_samples


def phases(axis, levels, repeats, seconds=2.):
    result = [(3., [0., 0.], "rest")]
    for repeat in range(repeats):
        for a in ([0, 1] if axis == "both" else [0 if axis == "x" else 1]):
            for level in levels:
                for sign in (1, -1):
                    command = [0., 0.]
                    command[a] = sign * level
                    result += [(seconds, command, f"{repeat}-{'xy'[a]}-{sign*level:g}"), (2., [0., 0.], "coast")]
    return result


def extract(file, clock="source"):
    """Old monitor traces retain topic-receipt times; new traces retain actual write/odom times."""
    rows = [json.loads(line) for line in file.read_text().splitlines() if line.strip()]
    if rows and "event" in rows[0]:
        commands = [[r["ros_time"] if clock == "source" else r["monotonic"], *r["values"][:2]] for r in rows if r["event"] == "tx"]
        poses = [[r["source_stamp"] if clock == "source" else r["monotonic"], *r["position"][:2]] for r in rows if r["event"] == "pose"]
        scope = "actual host write to odometry " + clock + " time"
    else:
        frames = {u["time"]: u["xy"] for r in rows for u in r.get("commands", []) if u["kind"] == "final"}
        commands = [[t, *xy] for t, xy in sorted(frames.items())]
        poses = []
        for r in rows:
            m = r.get("mission", {})
            if not m.get("pose") or "guard_event" in r:
                continue
            stamp = r["time"]
            if clock == "source":
                if not m.get("history") or m.get("tf_age_sim_seconds") is None:
                    continue
                # Terminal elapsed time is frozen while TF keeps updating; do not invent a frozen source clock.
                clock_time = m["history"][0]["sim_time"] + m["elapsed_sim_seconds"]
                if m.get("state") in ("DONE", "FAULT", "CANCELLED", "IDLE"):
                    clock_time = r["time"]
                stamp = clock_time - m["tf_age_sim_seconds"]
            poses.append([stamp, *m["pose"]])
        scope = "final cmd_vel topic receipt to " + ("reconstructed TF source time" if clock == "source" else "aggregated mission pose receipt")
    poses = list({r[0]: r for r in poses}.values())
    return commands, sorted(poses), scope


def fit(files, out, clock="source"):
    out.mkdir(parents=True, exist_ok=False)
    results = []
    for i, file in enumerate(files):
        commands, poses, scope = extract(file, clock)
        try:
            result, predicted = fit_response(commands, poses)
            (out / f"prediction-{i}.json").write_text(json.dumps(predicted) + "\n")
            result.update(scope=scope, clock=clock, source=str(file), sha256=hashlib.sha256(file.read_bytes()).hexdigest())
            result["implementation_sha256"] = {name: hashlib.sha256(path.read_bytes()).hexdigest() for name, path in (
                ("extractor", Path(__file__)), ("fitter", ROOT/"scripts/internal/response_fit.py"))}
            # Old aggregate logs can support an effective model, not a separated actuator latency calibration.
            result["physical_actuator_delay_verified"] = False
            result["source_type"] = "field_recording"
        except ValueError as error:
            result = dict(source=str(file), eligible_for_model=False, error=str(error))
        result["stopping_samples"] = stopping_samples(commands, poses)
        result["braking_acceleration_override"] = None  # Delay and mechanical braking are not separated by this summary.
        results.append(result)
        (out / f"model-{i}.json").write_text(json.dumps(result, indent=2) + "\n")
        print(json.dumps(result), flush=True)
    report = dict(results=results, active_configuration_modified=False,
                  convention="v_dot=(K*u(t-delay)-v)/tau; fit integrated positions, last 30 percent held out")
    (out / "response-model.json").write_text(json.dumps(report, indent=2) + "\n")


def record(args, out, schedule):
    import fcntl
    import signal
    import termios
    import numpy as np
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from intramural_base.protocol import FeedbackParser, command_frame
    from intramural_base.serial_bridge import open_serial, write_frame
    from intramural_base.transport import CommandPolicy, validate_vehicle_config

    if not args.execute or not args.stop_available:
        raise ValueError("Ground recording requires --execute --stop-available; clear the floor and stop navigation first")
    config = json.loads((args.vehicle_config / "base-vehicle.json").read_text())
    validate_vehicle_config(config)
    policy = CommandPolicy(config["body_to_protocol"], config["max_body_speed_mps"],
                           config["max_body_acceleration_mps2"], config["max_protocol_axis"])
    if max(args.levels) > min(policy.speed_limit, policy.protocol_limit):
        raise ValueError("Requested level exceeds the existing vehicle limit")
    candidates = sorted(Path("/dev/serial/by-id").glob("usb-1a86_USB_Single_Serial_*-if00"))
    device = Path(args.device).resolve(strict=True) if args.device else candidates[0].resolve() if len(candidates) == 1 else None
    if device is None or not str(device).startswith("/dev/") or not device.is_char_device():
        raise ValueError("Specify the unique serial character device with --device")
    # Share our navigation lock; unrelated legacy programs must be stopped by the operator.
    runtime = ROOT / "cache/runtime"
    runtime.mkdir(parents=True, exist_ok=True)
    lock = open(runtime / "vehicle-navigation.lock", "a")
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    out.mkdir(parents=True, exist_ok=False)
    trace = open(out / "samples.jsonl", "w", buffering=1)
    def emit(row):
        trace.write(json.dumps(row, allow_nan=False) + "\n")
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node("response_measurement")
    latest = {}
    def pose(message):
        p, q = message.pose.pose.position, message.pose.pose.orientation
        now = time.monotonic()
        stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
        values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w, stamp]
        if not all(math.isfinite(v) for v in values):
            return
        if latest and stamp <= latest["source_stamp"]:
            return
        row = dict(event="pose", monotonic=now, ros_receive=node.get_clock().now().nanoseconds/1e9,
                   source_stamp=stamp, position=[p.x, p.y, p.z], quaternion=[q.x, q.y, q.z, q.w],
                   frame=message.header.frame_id, child=message.child_frame_id)
        latest.update(row)
        emit(row)
    node.create_subscription(Odometry, args.odom_topic, pose, qos_profile_sensor_data)
    stopped = False
    def stop(*_):
        nonlocal stopped
        stopped = True
    previous_signals = {sig: signal.getsignal(sig) for sig in (signal.SIGINT, signal.SIGTERM)}
    for sig in previous_signals:
        signal.signal(sig, stop)
    fd, owned, error, seq, written = None, False, None, 0, 0
    parser = FeedbackParser()
    def send(values, stage):
        nonlocal seq, written
        before = time.monotonic()
        write_frame(fd, command_frame(*values, seq))
        emit(dict(event="tx", monotonic=before, write_completed=time.monotonic(),
                  ros_time=node.get_clock().now().nanoseconds/1e9, values=list(values), seq=seq, stage=stage))
        seq, written = (seq+1) % 256, written+1
    try:
        deadline = time.monotonic() + 8
        while len(latest) == 0 and time.monotonic() < deadline and not stopped:
            rclpy.spin_once(node, timeout_sec=.05)
        if not latest or stopped:
            raise RuntimeError("No fresh odometry before opening serial")
        fd = open_serial(str(device))
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        fcntl.ioctl(fd, termios.TIOCEXCL)
        owned = policy.enabled = True
        origin = np.array(latest["position"][:2])
        for seconds, values, stage in schedule:
            print(stage, seconds, values, flush=True)
            until, due = time.monotonic() + seconds, time.monotonic()
            while time.monotonic() < until:
                rclpy.spin_once(node, timeout_sec=0)
                now = time.monotonic()
                if stopped or now - latest["monotonic"] > .3:
                    raise RuntimeError("Operator stop or odometry stopped; returning to zero")
                if abs(latest["ros_receive"] - latest["source_stamp"]) > .3:
                    raise RuntimeError("Odometry source clock/age invalid; returning to zero")
                if np.linalg.norm(np.array(latest["position"][:2]) - origin) > 1.5:
                    raise RuntimeError("Exceeded the 1.5 m commissioning area; returning to zero")
                if now - due > .05:
                    raise RuntimeError("Serial deadline missed; returning to zero")
                if now >= due:
                    policy.receive(*values, 0., now=now)
                    actual, _ = policy.tick(now)
                    send(actual, stage)
                    due += .01
                try:
                    parser.feed(os.read(fd, 4096))
                except BlockingIOError:
                    pass
                time.sleep(.0005)
    except Exception as exception:
        error = str(exception)
    finally:
        try:
            if owned:
                for _ in range(30):
                    send((0., 0., 0.), "final_zero")
                    rclpy.spin_once(node, timeout_sec=0)
                    time.sleep(.01)
        except Exception as exception:
            error = f"{error or ''}; final zero failed: {exception}"
        finally:
            if fd is not None:
                os.close(fd)
            node.destroy_node()
            rclpy.shutdown()
            trace.close()
            lock.close()
            for sig, handler in previous_signals.items():
                signal.signal(sig, handler)
            (out / "recording.json").write_text(json.dumps(dict(error=error, device=str(device),
                protocol="competition19", frames=written, schedule=schedule,
                physical_response_verified=False, vehicle_config_sha256=hashlib.sha256(json.dumps(config, sort_keys=True).encode()).hexdigest()), indent=2) + "\n")
    if error:
        raise RuntimeError(error)
    fit([out / "samples.jsonl"], out / "fit")


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("mode", choices=["plan", "fit", "record"])
    p.add_argument("--trace", nargs="+", type=Path)
    p.add_argument("--clock", choices=["source", "receipt"], default="source")
    p.add_argument("--output", type=Path)
    p.add_argument("--axis", choices=["x", "y", "both"], default="both")
    p.add_argument("--levels", type=float, nargs="+", default=[.6])
    p.add_argument("--seconds", type=float, default=2.)
    p.add_argument("--repeats", type=int, default=2)
    p.add_argument("--vehicle-config", type=Path, default=ROOT / "cache/vehicle-current")
    p.add_argument("--device")
    p.add_argument("--odom-topic", default="/intramural_robot/aft_mapped_to_init")
    p.add_argument("--execute", action="store_true")
    p.add_argument("--stop-available", action="store_true")
    args = p.parse_args()
    if not args.levels or any(not math.isfinite(x) or not 0 < x <= 1.5 for x in args.levels) or not 1 <= args.repeats <= 3:
        p.error("Levels must be in (0, 1.5] and within vehicle limits; repeats in 1..3")
    if not math.isfinite(args.seconds) or not 0 < args.seconds <= 3:
        p.error("--seconds must be in (0, 3]")
    args.vehicle_config = (ROOT / args.vehicle_config).resolve()
    out = (ROOT / args.output).resolve() if args.output else ROOT / "artifacts/response" / datetime.now().strftime("%Y%m%d-%H%M%S-%f")
    schedule = phases(args.axis, args.levels, args.repeats, args.seconds)
    if args.mode == "plan":
        print(json.dumps(dict(duration_seconds=sum(r[0] for r in schedule), phases=schedule), indent=2))
    elif args.mode == "fit":
        if not args.trace:
            p.error("fit requires --trace")
        fit([(ROOT / f).resolve() for f in args.trace], out, args.clock)
    else:
        record(args, out, schedule)


if __name__ == "__main__":
    main()
