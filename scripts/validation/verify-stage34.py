#!/usr/bin/env python3
"""Own fresh Gazebo and mission processes; verify trajectories and clean shutdown."""

import argparse, json, signal, subprocess, shutil, hashlib
from pathlib import Path

p = argparse.ArgumentParser(description=__doc__)
p.add_argument("--output", required=True)
p.add_argument("--mode", choices=["ideal", "sensors"], default="ideal")
p.add_argument("--variant", choices=["figure8", "primitives"], default="figure8")
p.add_argument("--transport", choices=["direct", "serial-sim"], default="direct")
p.add_argument("--rounds", type=int, default=5)
p.add_argument("--gui", action="store_true")
p.add_argument("--obstacle-check", action="store_true")
args = p.parse_args()
out = Path(args.output).resolve()
if not out.is_relative_to(Path("/work/artifacts")) or args.rounds < 1:
    raise ValueError("Output/rounds invalid")
out.mkdir(parents=True, exist_ok=True)
shutil.copyfile("/work/simulation/generated/manifest.json", out / "arena-manifest.json")
shutil.copyfile("/work/config/arena.json", out / "arena.json")
shutil.copytree(
    "/work/simulation/generated", out / "arena-snapshot", dirs_exist_ok=True
)
shutil.copyfile("/work/docker/sources.lock.json", out / "sources.lock.json")
sources = [file for file in Path("/work/scripts").rglob("*")
           if file.is_file() and file.suffix in (".py", ".sh", ".cpp")]
sources += list(Path("/work/nav_ws/src/intramural_mission").rglob("*.py"))
sources += list(Path("/work/nav_ws/src/intramural_base").rglob("*.py"))
sources += list(Path("/work/launch").glob("arena*.py"))
sources += [
    Path("/work/config/arena.json"),
    Path("/work/config/fastdds-services.xml"),
    Path("/work/scripts/run.sh"),
    Path("/work/config/stage2-nav.yaml"),
    Path('/work/config/arena-gui.config'),
    Path('/work/simulation/models/sentry_robot_cylinder.xacro'),
    Path("/work/nav_ws/src/intramural_mission/package.xml"),
]
(out / "implementation-sha256.json").write_text(
    json.dumps(
        {
            str(f.relative_to("/work")): hashlib.sha256(f.read_bytes()).hexdigest()
            for f in sources
        },
        indent=2,
    )
    + "\n"
)
(out / "shutdown.json").unlink(missing_ok=True)
with (out / "system.log").open("w") as log:
    command = [
        "python3",
        "/work/scripts/internal/arena-process.py",
        "--localization",
        args.mode,
        "--variant",
        args.variant,
        "--transport",
        args.transport,
    ]
    if not args.gui:
        command.append("--no-gui")
    proc = subprocess.Popen(
        command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True
    )
    try:
        with (out / "verification.log").open("w") as record:
            subprocess.run(
                [
                    "python3",
                    "/work/scripts/validation/verify-arena.py",
                    "--output",
                    str(out),
                    "--rounds",
                    str(args.rounds),
                    "--mode",
                    args.mode,
                    "--variant",
                    args.variant,
                    "--transport",
                    args.transport,
                ],
                stdout=record,
                stderr=subprocess.STDOUT,
                check=True,
                timeout=1800 * args.rounds,
            )
        if args.obstacle_check:
            with (out / "obstacle.log").open("w") as record:
                subprocess.run(
                    [
                        "python3",
                        "/work/scripts/validation/verify-obstacle.py",
                        "--output",
                        str(out),
                    ],
                    stdout=record,
                    stderr=subprocess.STDOUT,
                    check=True,
                    timeout=120,
                )
        with (out / "interfaces.log").open("w") as record:
            subprocess.run(
                [
                    "python3",
                    "/work/scripts/validation/inspect-arena.py",
                    "--output",
                    str(out),
                    "--mode",
                    args.mode,
                    "--transport",
                    args.transport,
                ],
                stdout=record,
                stderr=subprocess.STDOUT,
                check=True,
                timeout=120,
            )
        subprocess.run(
            ["python3", "/work/scripts/validation/plot-arena.py", "--input", str(out)],
            check=True,
            timeout=60,
        )
    finally:
        if proc.poll() is None:
            proc.send_signal(signal.SIGINT)
            try:
                proc.wait(timeout=120)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()
                raise RuntimeError("Arena supervisor required forced shutdown")
    log.flush()
text = (out / "system.log").read_text()
markers = [
    "process has died",
    "forced termination",
    "abnormal exit",
    "escalating to",
    "Traceback (most recent call last)",
    "terminate called",
    "Segmentation fault",
    "client will not receive response",
]
if proc.returncode or any(m in text for m in markers):
    raise RuntimeError("Arena child failure: inspect system.log")
for marker in [
    "Gazebo server: clean exit 0",
    "ROS simulation nodes: clean exit 0",
    "Arena navigation: clean exit 0",
]:
    if marker not in text:
        raise RuntimeError("Missing shutdown marker " + marker)
if args.transport == "serial-sim" and "Simulated MCU: clean exit 0" not in text:
    raise RuntimeError("Missing simulated MCU exit evidence")
if args.gui and "Gazebo GUI: clean exit 0" not in text:
    raise RuntimeError("Missing GUI exit evidence")
(out / "shutdown.json").write_text(
    json.dumps(
        {
            "exit_code": proc.returncode,
            "child_failures": False,
            "forced_termination": False,
            "gui": args.gui,
            "transport": args.transport,
        },
        indent=2,
    )
    + "\n"
)
print(
    f"PASS arena: {args.mode}/{args.variant}/{args.transport}, {args.rounds} rounds, cancel, reset and clean shutdown; {out}"
)
