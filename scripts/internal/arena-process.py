#!/usr/bin/env python3
"""Own arena and navigation, cancelling requests before lifecycle shutdown."""

import argparse, json, os, signal, subprocess, time
from pathlib import Path

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--localization", choices=["ideal", "sensors"], default="ideal")
parser.add_argument("--variant", choices=["figure8", "primitives"], default="figure8")
parser.add_argument("--transport", choices=["direct", "serial-sim"], default="direct")
parser.add_argument("--no-gui", action="store_true")
args = parser.parse_args()
root = Path(__file__).resolve().parents[2]
c = json.loads((root / "config/arena.json").read_text())
ns = c["namespace"]
if os.environ.get("INTRAMURAL_TRANSPORT", "direct") != args.transport:
    raise ValueError("Generated arena transport and runtime transport differ")
interrupted = False
failed = False


def interrupt(signum, frame):
    global interrupted
    interrupted = True


signal.signal(signal.SIGINT, interrupt)
signal.signal(signal.SIGTERM, interrupt)


def stop(p, label, timeout):
    global failed
    if p is None:
        return
    if p.poll() is None:
        p.send_signal(signal.SIGINT)
        try:
            p.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            print(label + ": forced termination", flush=True)
            p.kill()
            p.wait()
            failed = True
    if p.returncode:
        failed = True
        print(f"{label}: abnormal exit {p.returncode}", flush=True)
    else:
        print(label + ": clean exit 0", flush=True)


command = [
    "python3",
    str(root / "scripts/internal/gazebo-process.py"),
    "--world",
    str(root / "simulation/generated/arena.sdf"),
    "--gui-config",
    str(root / "config/arena-gui.config"),
    "--engine",
    "ogre" if Path("/dev/dxg").exists() and os.environ.get("LIBGL_ALWAYS_SOFTWARE", "0") != "1" else "ogre2",
    "--nodes-launch",
    str(root / "launch/arena-nodes.launch.py"),
]
if args.no_gui:
    command.append("--no-gui")
sim_mcu = None
if args.transport == "serial-sim":
    device_file = root / "cache/runtime/intramural-sim-tty"
    device_file.unlink(missing_ok=True)
    sim_mcu = subprocess.Popen([str(Path(os.environ.get("INTRAMURAL_INSTALL_PREFIX", str(root / "nav_ws/install"))) / "intramural_base/lib/intramural_base/sim_mcu"),
        "--device-file", str(device_file), "--ros-args", "-r", f"__ns:=/{ns}"],
        start_new_session=True)
    deadline = time.monotonic() + 10
    while not device_file.exists() and sim_mcu.poll() is None and time.monotonic() < deadline:
        time.sleep(0.02)
    if not device_file.exists() or sim_mcu.poll() is not None:
        raise RuntimeError("Simulated MCU failed to provide PTY")
sim = subprocess.Popen(command, start_new_session=True)
nav = None
try:
    env = os.environ.copy()
    env["INTRAMURAL_LOCALIZATION"] = args.localization
    env["INTRAMURAL_MISSION_VARIANT"] = args.variant
    env["INTRAMURAL_TRANSPORT"] = args.transport
    nav = subprocess.Popen(
        ["ros2", "launch", str(root / "launch/arena-navigation.launch.py")],
        env=env,
        start_new_session=True,
    )
    deadline = time.monotonic() + 60
    while not interrupted and time.monotonic() < deadline:
        r = subprocess.run(
            [
                "ign",
                "service",
                "-s",
                "/world/default/control",
                "--reqtype",
                "ignition.msgs.WorldControl",
                "--reptype",
                "ignition.msgs.Boolean",
                "--timeout",
                "1000",
                "--req",
                "pause: false",
            ],
            capture_output=True,
            text=True,
            timeout=5,
        )
        if r.returncode == 0 and "true" in r.stdout:
            break
        if sim.poll() is not None or nav.poll() is not None:
            raise RuntimeError("Arena startup failed")
        time.sleep(0.2)
    else:
        if not interrupted:
            raise RuntimeError("World control unavailable")
    if not interrupted:
        subprocess.run(
            ["python3", str(root / "scripts/internal/start-arena-navigation.py")],
            check=True,
            timeout=100,
        )
    while not interrupted and sim.poll() is None and nav.poll() is None and (sim_mcu is None or sim_mcu.poll() is None):
        time.sleep(0.1)
    if not interrupted:
        failed = True
        print("Arena child exited before requested shutdown", flush=True)
finally:
    if nav and nav.poll() is None:
        try:
            subprocess.run(
                [
                    "ros2",
                    "service",
                    "call",
                    f"/{ns}/mission/cancel",
                    "std_srvs/srv/Trigger",
                    "{}",
                ],
                timeout=8,
                check=True,
            )
            r = subprocess.run(
                [
                    "ros2",
                    "service",
                    "call",
                    f"/{ns}/lifecycle_manager_arena/manage_nodes",
                    "nav2_msgs/srv/ManageLifecycleNodes",
                    "{command: 4}",
                ],
                capture_output=True,
                text=True,
                timeout=25,
                check=True,
            )
            print(r.stdout, flush=True)
            if "success=True" not in r.stdout:
                raise RuntimeError("Lifecycle shutdown not acknowledged")
        except Exception as exc:
            failed = True
            print(f"Arena lifecycle shutdown failed: {exc}", flush=True)
    stop(sim, "Arena simulation", 55)
    stop(nav, "Arena navigation", 25)
    stop(sim_mcu, "Simulated MCU", 15)
raise SystemExit(1 if failed else 0)
