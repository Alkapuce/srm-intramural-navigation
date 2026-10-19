#!/usr/bin/env python3
"""Offline closed-loop comparison: actual PB/MPPI plugins and the production tracker."""

import argparse
from collections import deque
from dataclasses import asdict
import hashlib
import json
import math
import os
from pathlib import Path
import select
import subprocess
import time

import numpy as np
from PIL import Image
from scipy.ndimage import distance_transform_edt
import yaml
from nav_msgs.msg import OccupancyGrid
from ament_index_python.packages import get_package_prefix
from intramural_base.transport import CommandPolicy
from intramural_mission.planning import GridMap, compile_route
from intramural_mission.trajectory import OmniTracker
from intramural_mission.response import Plant

ROOT = Path(__file__).resolve().parents[2]
DT = .025
SCENARIOS = {"nominal": (.2, .15), "delayed": (.4, .3)}


class Worker:
    def __init__(self, config, log):
        self.log = open(log, "w")
        env = dict(os.environ, RCUTILS_LOGGING_USE_STDOUT="0")
        if config["backend"].startswith("mppi"):
            # PB and Nav2 both ship liblayers.so. Resolve the Nav2 critic's dependency in this worker only.
            env["LD_LIBRARY_PATH"] = get_package_prefix("nav2_costmap_2d") + "/lib:" + env.get("LD_LIBRARY_PATH", "")
        self.process = subprocess.Popen(
            [str(ROOT / "cache/controller-benchmark/controller-worker")],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=self.log, text=True,
            env=env, bufsize=1,
        )
        try:
            if not self.call(config).get("ready"):
                raise RuntimeError("Controller did not become ready")
        except BaseException as error:
            try:
                self.close()
            except RuntimeError as close_error:
                raise RuntimeError(f"{error}; {close_error}") from error
            raise

    def call(self, request):
        self.process.stdin.write(json.dumps(request, allow_nan=False) + "\n")
        self.process.stdin.flush()
        if not select.select([self.process.stdout], [], [], 30)[0]:
            raise RuntimeError("Controller response timed out; see worker log")
        line = self.process.stdout.readline()
        if not line:
            raise RuntimeError("Controller exited; see worker log")
        result = json.loads(line)
        if "error" in result:
            raise RuntimeError(result["error"])
        return result

    def close(self):
        self.process.stdin.close()
        try:
            code = self.process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait()
            raise RuntimeError("Offline worker failed to exit normally")
        finally:
            self.process.stdout.close()
            self.log.close()
        if code:
            raise RuntimeError(f"Offline worker exit code {code}")


def load_map(folder, radius):
    metadata = yaml.safe_load((folder / "map.yaml").read_text())
    if metadata["origin"][2] != 0 or metadata.get("negate", 0):
        raise ValueError("Benchmark requires an unrotated, non-negated exported map")
    pixels = np.asarray(Image.open(folder / metadata["image"]).convert("L")) / 255.
    probability = 1 - pixels[::-1]
    cells = np.where(probability > metadata.get("occupied_thresh", .65), 100,
                     np.where(probability < metadata.get("free_thresh", .196), 0, -1))
    g = OccupancyGrid()
    g.header.frame_id = "map"
    g.info.width, g.info.height = pixels.shape[1], pixels.shape[0]
    g.info.resolution = metadata["resolution"]
    g.info.origin.position.x, g.info.origin.position.y = metadata["origin"][:2]
    g.info.origin.orientation.w = 1.
    g.data = cells.astype(np.int8).ravel().tolist()
    # Both Nav2 backends use the same circular-footprint inflation; the physical grid stays unchanged.
    distance = distance_transform_edt(np.pad(cells == 0, 1))[1:-1, 1:-1] * g.info.resolution
    inflated = np.where(distance <= radius, 253,
                        np.where(distance <= radius + .3, np.maximum(1, 252 * np.exp(-10 * (distance-radius))), 0))
    inflated = np.where(cells == 0, inflated, np.where(cells < 0, 255, 254)).astype(np.uint8)
    worker_grid = dict(width=g.info.width, height=g.info.height, resolution=g.info.resolution,
                       origin=metadata["origin"][:2],
                       data=inflated.ravel().tolist())
    return g, worker_grid


def run_case(c, plan, grid, worker_grid, backend, scenario, out):
    delay, tau = SCENARIOS[scenario]
    tracker = OmniTracker(plan, c)
    plant = Plant(plan.points[0], delay, tau)
    policy = CommandPolicy([[1, 0], [0, 1]], c["max_speed"], c["max_acceleration"], c["max_speed"])
    policy.enabled, policy.last_tick = True, -DT
    history = deque([np.zeros(2).tolist()] * round(c["response_delay"] / DT),
                    maxlen=max(1, round(c["response_delay"] / DT)))
    worker = None
    traces, timing, winding = [], [], np.zeros(2)
    reason, finished_at, settled = "timeout", None, 0
    try:
        if backend != "tracker":
            worker = Worker(dict(backend=backend, dt=DT, max_speed=c["max_speed"],
                                 max_acceleration=c["max_acceleration"], robot_radius=c["robot_radius"],
                                 model_delay=c["response_delay"], model_tau=c.get("response_tau", .15), map=worker_grid),
                            out.with_suffix(".log"))
        for tick in range(round(90 / DT)):
            t = tick * DT
            previous = plant.pose.copy()
            started = time.perf_counter()
            tracker.observe(plant.pose, DT, t)
            if backend == "tracker":
                command, report = tracker.step(plant.pose, 0., DT, stamp=t)
                error = report["error"]
                compute_ms = (time.perf_counter()-started)*1000
            else:
                error = tracker.progress.project(plant.pose, c["max_speed"] * DT + .12,
                                                 c["max_tracking_error"])
                s = tracker.progress.s
                start, _, _ = plan.sample(s)
                # Ordered local path prevents MPPI/PB from jumping to the later crossing.
                local = [start.tolist()] + [list(p) for p, distance in zip(plan.points, plan.s)
                                           if s + 1e-6 < distance <= s + 2.]
                if len(local) < 2:
                    local.append(list(plan.points[-1]))
                cap = speed_cap(plan, s, c)
                result = worker.call(dict(pose=plant.pose.tolist(), yaw=0., velocity=tracker.velocity.tolist(),
                                          path=local, speed_cap=cap,
                                          previous_command=list(policy.previous), pending_commands=list(history)))
                command, compute_ms = np.asarray(result["command"]), result["compute_ms"]
            cap = speed_cap(plan, tracker.progress.s, c)
            command *= min(1., cap / max(1e-9, np.linalg.norm(command)))
            if finished_at is not None:
                command = np.zeros(2)
            policy.receive(*command, 0., now=t)
            sent, _ = policy.tick(t)
            history.append(list(sent[:2]))
            plant.step(sent[:2])
            for i, key in enumerate(("lower_pole", "upper_pole")):
                a, b = previous - c[key], plant.pose - c[key]
                winding[i] += math.atan2(a[0] * b[1] - a[1] * b[0], np.dot(a, b))
            gap = min(float(np.linalg.norm(plant.pose - c[k]) - c["robot_radius"] - c["pole_radius"])
                      for k in ("lower_pole", "upper_pole"))
            # The grid check retains the configured map/discretization margin, separately from physical poles.
            map_gap = float(grid.clearance([plant.pose])[0])
            traces.append(dict(time=t + DT, pose=plant.pose.tolist(), velocity=plant.velocity.tolist(),
                               command=list(sent[:2]), s=tracker.progress.s, phase=tracker.progress.phase,
                               error=error, pole_clearance=gap, map_clearance=map_gap, compute_ms=compute_ms,
                               host_step_ms=(time.perf_counter()-started)*1000))
            timing.append(compute_ms)
            if gap <= 0 or map_gap <= 0 or error > c["max_tracking_error"]:
                reason = "pole_contact" if gap <= 0 else "map_clearance" if map_gap <= 0 else "tracking_error"
                break
            remaining = plan.s[-1] - tracker.progress.s
            parked = np.linalg.norm(plant.pose - c["parking"]) < c["parking_tolerance"]
            if finished_at is None and remaining < c["finish_distance"] and parked and np.linalg.norm(tracker.velocity) < .06:
                finished_at = t + DT
            if finished_at is not None:
                settled += 1
                if settled * DT >= delay + 5 * tau:
                    reason = "done" if parked and np.linalg.norm(plant.velocity) < .06 else "parking_after_coast"
                    break
    except (RuntimeError, ValueError, BrokenPipeError) as exc:
        reason = f"controller_error: {exc}"
    finally:
        if worker:
            try:
                worker.close()
            except RuntimeError as exc:
                reason = f"worker_shutdown_error: {exc}"
    out.with_suffix(".jsonl").write_text("".join(json.dumps(row) + "\n" for row in traces))
    turns = (winding / (2 * math.pi)).tolist()
    if reason == "done" and any(abs(a - b) > .15 for a, b in zip(turns, (1, -1))):
        reason = "winding_order"
    return dict(backend=backend, scenario=scenario, profile=c["profile"], reason=reason, passed=reason == "done",
                completion_seconds=finished_at, simulated_seconds=len(traces) * DT,
                max_tracking_error=max((x["error"] for x in traces), default=0),
                minimum_pole_clearance=min((x["pole_clearance"] for x in traces), default=None),
                minimum_map_clearance=min((x["map_clearance"] for x in traces), default=None),
                parking_error=float(np.linalg.norm(plant.pose - c["parking"])), winding=turns,
                solve_ms_p50=float(np.percentile(timing, 50)) if timing else None,
                solve_ms_p95=float(np.percentile(timing, 95)) if timing else None,
                host_step_ms_p95=float(np.percentile([x["host_step_ms"] for x in traces], 95)) if traces else None,
                solver_overruns=sum(ms > 1000*DT for ms in timing),
                trace=out.with_suffix(".jsonl").name)


def speed_cap(plan, s, c):
    # A small launch floor lets position-based progress leave s=0; the same floor applies to all backends.
    return max(.05, min(c["max_speed"], float(np.interp(min(s + .06, plan.s[-1]), plan.s, plan.speed))))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--map-dir", required=True, type=Path, help="Exported map.yaml/map image and arena.json")
    p.add_argument("--arena-config", type=Path, help="Explicit geometry for a generated public simulation map")
    p.add_argument("--output", required=True, type=Path)
    p.add_argument("--profiles", nargs="+", choices=["safe", "candidate"], default=["safe", "candidate"])
    p.add_argument("--backends", nargs="+", choices=["tracker", "pb", "mppi", "mppi-response"],
                   default=["tracker", "pb", "mppi", "mppi-response"])
    p.add_argument("--scenarios", nargs="+", choices=list(SCENARIOS), default=list(SCENARIOS))
    p.add_argument("--repeats", type=int, default=1)
    p.add_argument("--response-model", type=Path, help="Eligible source-clock model-N.json from measure-response.sh fit")
    args = p.parse_args()
    args.map_dir = (ROOT / args.map_dir).resolve()
    args.output = (ROOT / args.output).resolve()
    if args.repeats < 1:
        p.error("--repeats must be positive")
    args.output.mkdir(parents=True, exist_ok=False)
    binary = ROOT / "cache/controller-benchmark/controller-worker"
    if set(args.backends) != {"tracker"}:
        subprocess.run(["cmake", "-S", str(ROOT / "scripts/internal/controller-benchmark"),
                        "-B", str(binary.parent), "-DCMAKE_BUILD_TYPE=Release"], check=True, stdout=subprocess.DEVNULL)
        subprocess.run(["cmake", "--build", str(binary.parent), "-j2"], check=True, stdout=subprocess.DEVNULL)
    arena_file = (ROOT / args.arena_config).resolve() if args.arena_config else args.map_dir / "arena.json"
    base = json.loads(arena_file.read_text())
    profiles = json.loads((ROOT / "config/navigation-profiles.json").read_text())
    model = None
    if args.response_model:
        args.response_model = (ROOT / args.response_model).resolve()
        model = json.loads(args.response_model.read_text())
        if not model.get("eligible_for_model") or model.get("clock") != "source":
            raise ValueError("Use an eligible source-clock model; receipt delay would double-count localization age")
        SCENARIOS["nominal"] = (model["response_delay"], model["response_tau"])
    g, worker_grid = load_map(args.map_dir, base["robot_radius"])
    sources = [ROOT / "scripts/internal/controller-benchmark/main.cpp", Path(__file__),
               ROOT / "scripts/internal/controller-benchmark/CMakeLists.txt",
               ROOT / "config/navigation-profiles.json", ROOT / "docker/sources.lock.json",
               ROOT / "nav_ws/src/intramural_mission/intramural_mission/trajectory.py",
               ROOT / "nav_ws/src/intramural_mission/intramural_mission/planning.py",
               ROOT / "nav_ws/src/intramural_mission/intramural_mission/route_optimization.py",
               ROOT / "nav_ws/src/intramural_mission/intramural_mission/response.py",
               ROOT / "nav_ws/src/intramural_base/intramural_base/transport.py"]
    pb = ROOT / "nav_ws/src/pb2025_sentry_nav/pb_omni_pid_pursuit_controller"
    sources += list(pb.rglob("*.cpp")) + list(pb.rglob("*.hpp"))
    metadata = yaml.safe_load((args.map_dir / "map.yaml").read_text())
    sources += [args.map_dir / name for name in ("map.yaml", metadata["image"])] + [arena_file]
    if binary.exists():
        sources.append(binary)
    if args.response_model:
        sources.append(args.response_model)
    for package in ("nav2_mppi_controller", "pb_omni_pid_pursuit_controller"):
        sources += list((Path(get_package_prefix(package))/"lib").glob("*.so"))
    def fingerprints():
        return {str(f.relative_to(ROOT)) if f.is_relative_to(ROOT) else str(f): hashlib.sha256(f.read_bytes()).hexdigest() for f in sources}
    hashes = fingerprints()
    results = []
    for name in args.profiles:
        c = dict(base, **profiles[name], profile=name)
        if model:
            c.update(response_delay=model["response_delay"], response_tau=model["response_tau"])
        plan, _, _ = compile_route(c, g)
        (args.output / f"route-{name}.json").write_text(json.dumps(asdict(plan)) + "\n")
        grid = GridMap(g, c)
        for scenario in args.scenarios:
            for backend in args.backends:
                for repeat in range(args.repeats):
                    label = f"{name}-{scenario}-{backend}-{repeat}"
                    result = run_case(c, plan, grid, worker_grid, backend, scenario, args.output / label)
                    results.append(result)
                    print(json.dumps(result), flush=True)
                    (args.output / "results.json").write_text(json.dumps(results, indent=2) + "\n")
    if hashes != fingerprints():
        raise RuntimeError("Sources or map changed during the comparison; results are not a frozen run")
    manifest = dict(scope="offline controllers, same C2 route/ordered progress/CommandPolicy/assumed plant; no motion_guard or hardware",
                    source_commit=os.environ.get("INTRAMURAL_SOURCE_REVISION", "not supplied; use compare-controllers.sh"),
                    scenarios=SCENARIOS, model_delay=model["response_delay"] if model else .2,
                    model_tau=model["response_tau"] if model else .15, dt=DT, plant_gain="identity; gain identification not applied",
                    response_model=model,
                    solver=dict(batch_size=512, time_steps=80, iterations=1),
                    mppi_randomness="Nav2 noise generator; independent processes; no deterministic seed claimed",
                    nav2_mppi_version=Path("/opt/ros/humble/share/nav2_mppi_controller/package.xml").read_text().split("<version>")[1].split("</version>")[0],
                    hashes=hashes, sources_unchanged_during_run=True, results=len(results))
    (args.output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    plot(results, args.output)
    return 0  # Failed candidates are comparison data, explicitly reported in results.json.


def plot(results, folder):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    profiles = list(dict.fromkeys(r["profile"] for r in results))
    scenarios = list(dict.fromkeys(r["scenario"] for r in results))
    fig, axes = plt.subplots(len(profiles), len(scenarios), figsize=(6*len(scenarios), 5*len(profiles)), squeeze=False)
    colors = dict(tracker="C0", pb="C1", mppi="C2", **{"mppi-response": "C3"})
    for row, profile in enumerate(profiles):
        route = json.loads((folder / f"route-{profile}.json").read_text())
        reference = np.array(route["points"])
        for col, scenario in enumerate(scenarios):
            ax = axes[row, col]
            ax.plot(*reference.T, "k--", alpha=.5, label="reference")
            labeled = set()
            for result in results:
                if (result["profile"], result["scenario"]) != (profile, scenario):
                    continue
                trace = [json.loads(line) for line in (folder/result["trace"]).read_text().splitlines()]
                if not trace:
                    continue
                xy = np.array([r["pose"] for r in trace])
                backend = result["backend"]
                label = backend if backend not in labeled else None
                labeled.add(backend)
                ax.plot(*xy.T, color=colors[backend], alpha=.8, label=label)
                if not result["passed"]:
                    ax.plot(*xy[-1], "x", color=colors[backend], markersize=9)
            ax.set(title=f"{profile} / {scenario} (x = failed endpoint)", xlabel="map x (m)", ylabel="map y (m)", aspect="equal")
            ax.grid(alpha=.2)
            ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(folder / "trajectories.png", dpi=160)
    plt.close(fig)


if __name__ == "__main__":
    raise SystemExit(main())
