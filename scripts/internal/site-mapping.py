#!/usr/bin/env python3
"""One foreground mapping session; all hardware processes are owned and cleaned up."""
import argparse
from datetime import datetime
import fcntl
import json
import math
import os
from pathlib import Path
import select
import shutil
import signal
import socket
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
RUNTIME = ROOT / "cache/runtime"
CALLER = Path(os.environ.get("INTRAMURAL_MAP_CALLER", os.getcwd()))


def path(value):
    p = Path(value).expanduser()
    return (p if p.is_absolute() else CALLER / p).resolve()


def processes(group):
    found = []
    for p in Path("/proc").glob("[0-9]*"):
        try:
            fields = (p / "stat").read_text().split(") ", 1)[1].split()
            if int(fields[2]) == group and fields[0] != "Z":
                found.append(int(p.name))
        except (FileNotFoundError, ProcessLookupError):
            pass
    return found


def stop_group(pid):
    if processes(pid):
        leader = Path("/proc") / str(pid)
        if leader.exists() and (leader / "stat").read_text().split(") ", 1)[1].split()[0] != "Z":
            os.kill(pid, signal.SIGINT)
        else:
            os.killpg(pid, signal.SIGINT)
    deadline = time.monotonic() + 20
    while processes(pid) and time.monotonic() < deadline:
        time.sleep(.1)
    if processes(pid):
        os.killpg(pid, signal.SIGTERM)
        deadline = time.monotonic() + 5
        while processes(pid) and time.monotonic() < deadline:
            time.sleep(.1)
        raise RuntimeError(f"Process group {pid} did not shut down normally; remaining: {processes(pid)}")


def shutdown_navigation():
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from nav2_msgs.srv import ManageLifecycleNodes
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node("mapping_shutdown")
    try:
        client = node.create_client(ManageLifecycleNodes, "/intramural_robot/lifecycle_manager_vehicle/manage_nodes")
        if not client.wait_for_service(timeout_sec=3):
            return
        request = ManageLifecycleNodes.Request()
        request.command = 4
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=15)
        if not future.done() or not future.result() or not future.result().success:
            raise RuntimeError("Navigation lifecycle shutdown did not succeed")
    finally:
        node.destroy_node()
        rclpy.shutdown()


def stop_previous():
    boot = Path("/proc/sys/kernel/random/boot_id").read_text().strip()
    for filename, launch in [("vehicle-navigation-state.json", "vehicle-navigation.launch.py"),
                             ("real-sensors-state.json", "vehicle-sensors.launch.py")]:
        file = RUNTIME / filename
        if not file.exists():
            continue
        state = json.loads(file.read_text())
        proc = Path("/proc") / str(state.get("pid", -1))
        if state.get("running") is False or state.get("boot_id") != boot or not proc.exists():
            continue
        if state.get("physical_serial_opened", True) or state.get("motion_permitted", False):
            raise RuntimeError("An existing session may control motors; stop it explicitly first")
        cmd = (proc / "cmdline").read_bytes()
        expected = str(ROOT / "launch" / launch).encode()
        if proc.stat().st_uid != os.getuid() or expected not in cmd or os.getpgid(state["pid"]) != state["pid"]:
            raise RuntimeError("Existing process identity changed; refusing to stop it")
        print("正常关闭前一次待机进程:", launch, flush=True)
        if filename.startswith("vehicle-navigation"):
            shutdown_navigation()
        stop_group(state["pid"])
        state["running"] = False
        file.write_text(json.dumps(state, indent=2) + "\n")


def preflight(vehicle):
    from occupancy import raycast_binary
    network = json.loads((vehicle / "MID360_config.json").read_text())["MID360"]["host_net_info"]
    endpoints = [("0.0.0.0", 56000)] + [(network[k + "_ip"], network[k + "_port"])
        for k in ("cmd_data", "push_msg", "point_data", "imu_data")]
    for host, port in endpoints:
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
                probe.bind((host, port))
        except OSError as error:
            raise RuntimeError(f"雷达端口 {host}:{port} 不可用: {error}。检查本机 IP，或先在原用户终端停止旧雷达/哨兵程序；不会自动终止他人进程。") from error
    raycast_binary(ROOT)


class Streams:
    def __init__(self):
        import rclpy
        from rclpy.signals import SignalHandlerOptions
        from rclpy.qos import qos_profile_sensor_data
        from livox_ros_driver2.msg import CustomMsg
        from sensor_msgs.msg import Imu, PointCloud2
        from nav_msgs.msg import Odometry
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        self.node = rclpy.create_node("mapping_stream_monitor")
        self.seen = {}
        self.subscriptions = []
        for topic, typ in [("/livox/lidar", CustomMsg), ("/livox/imu", Imu),
                           ("/intramural_robot/cloud_registered", PointCloud2),
                           ("/intramural_robot/aft_mapped_to_init", Odometry)]:
            self.subscriptions.append(self.node.create_subscription(
                typ, topic, lambda raw, t=topic: self.receive(t), qos_profile_sensor_data, raw=True))

    def receive(self, topic):
        now = time.monotonic()
        data = self.seen.setdefault(topic, {"first": now, "last": now, "count": 0})
        data["last"] = now
        data["count"] += 1

    def spin(self):
        import rclpy
        rclpy.spin_once(self.node, timeout_sec=.05)

    def healthy(self):
        return len(self.seen) == 4 and all(time.monotonic()-s["last"] < 1 for s in self.seen.values())

    def close(self):
        import rclpy
        self.node.destroy_node()
        rclpy.shutdown()


def main():
    p = argparse.ArgumentParser(description="采集 → 校验 → 导图 → 地图质量检查 → 重启传感器 → 定位验证；不控制电机")
    p.add_argument("--duration", type=float, default=0, help="录制秒数；默认按 Enter 结束，Ctrl-C 取消整个流程")
    p.add_argument("--bag", type=path, help="复用已正常结束的 bag，跳过新采集")
    p.add_argument("--vehicle-config", type=path, default=path(os.environ.get("INTRAMURAL_VEHICLE_CONFIG", str(ROOT / "cache/vehicle-current"))))
    p.add_argument("--output", type=path, help="新的 artifacts/ 或 cache/ 子目录；默认自动按时间命名")
    p.add_argument("--arena-config", type=path, help="已测量的比赛场地几何；不提供时保持实验几何")
    args = p.parse_args()
    if not math.isfinite(args.duration) or args.duration < 0 or 0 < args.duration < 5:
        p.error("--duration 须为 0 或至少 5 秒")
    if not args.bag and args.duration == 0 and not sys.stdin.isatty():
        p.error("非交互运行请指定 --duration 秒数，或 --bag 已完成的录制")
    for name in ("handoff.json", "MID360_config.json", "livox-driver.yaml", "point-lio.yaml", "base-vehicle.json"):
        if not (args.vehicle_config / name).is_file():
            p.error("缺少实车配置: " + str(args.vehicle_config / name))
    if args.bag and not (args.bag / "metadata.yaml").is_file():
        p.error("bag 尚未正常收尾或缺少 metadata.yaml；请先恢复副本")
    out = args.output or ROOT / "artifacts/mapping" / datetime.now().strftime("%Y%m%d-%H%M%S-%f")
    if out.exists() or not any(out.is_relative_to(ROOT / d) and out != ROOT / d for d in ("cache", "artifacts")):
        p.error("输出必须是项目 cache/ 或 artifacts/ 下尚不存在的新目录")
    if shutil.disk_usage(ROOT).free < 2 * 1024**3:
        p.error("可用磁盘不足 2 GiB，未启动采集")
    RUNTIME.mkdir(parents=True, exist_ok=True)
    lock = (RUNTIME / "site-mapping.lock").open("w")
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        p.error("已有 srm-map 正在运行，请先结束它")
    out.mkdir(parents=True)
    result = {"status": "running", "output": str(out), "vehicle_config": str(args.vehicle_config),
              "physical_serial_opened": False, "motion_permitted": False, "stages": []}
    children = {}
    groups = []
    monitor = None
    os.environ["INTRAMURAL_VEHICLE_CONFIG"] = str(args.vehicle_config)

    def save():
        result["owner_pid"] = os.getpid()
        result["boot_id"] = Path("/proc/sys/kernel/random/boot_id").read_text().strip()
        result["child_pids"] = {name: child.pid for name, child in children.items()}
        temporary = out / "result.json.tmp"
        temporary.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n")
        temporary.replace(out / "result.json")
        temporary = RUNTIME / "site-mapping-cli-state.json.tmp"
        temporary.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n")
        temporary.replace(RUNTIME / "site-mapping-cli-state.json")

    def start(name, command):
        with (out / (name + ".log")).open("w") as log:
            process = subprocess.Popen(command, stdin=subprocess.DEVNULL, stdout=log,
                                       stderr=subprocess.STDOUT, start_new_session=True)
        children[name] = process
        groups.append(process.pid)
        save()
        return process

    def check():
        for name, process in children.items():
            if process.poll() is not None:
                raise RuntimeError(f"{name} 提前退出 ({process.returncode})，见 {out / (name + '.log')}")

    def stop(name, cancelling=False):
        process = children.pop(name)
        stop_group(process.pid)
        code = process.wait(timeout=2)
        if code != 0 and not (cancelling and code in (-signal.SIGINT, 130)):
            raise RuntimeError(f"{name} 异常退出 ({code})，见对应日志")

    def run(name, command, timeout=300):
        print(name, flush=True)
        process = start(name, command)
        try:
            code = process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            raise RuntimeError(f"{name} 超时，见 {out / (name + '.log')}")
        children.pop(name)
        if code != 0:
            raise RuntimeError(f"{name} 未通过 ({code})，见 {out / (name + '.log')}")
        result["stages"].append(name)
        save()

    def sensors(name):
        nonlocal monitor
        start(name, [str(ROOT / "scripts/vehicle-sensors.sh")])
        monitor = Streams()
        deadline = time.monotonic() + 40
        while time.monotonic() < deadline:
            check()
            monitor.spin()
            if monitor.healthy() and all(s["count"] >= (500 if t == "/livox/imu" else 25) for t, s in monitor.seen.items()):
                if any(monitor.node.count_publishers(t) != 1 for t in monitor.seen):
                    raise RuntimeError("同一 ROS domain 存在重复传感器发布者，请停止冲突进程")
                return
        raise RuntimeError("40 秒内未收到连续雷达、IMU 与定位数据；检查供电、网线和雷达占用")

    def interrupted(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupted)
    signal.signal(signal.SIGINT, interrupted)
    signal.signal(signal.SIGHUP, interrupted)
    save()
    exit_code = 1
    try:
        stop_previous()
        preflight(args.vehicle_config)
        print("输出目录:", out, "\n保持车辆静止初始化；本命令不会启动电机。", flush=True)
        bag = args.bag
        if not bag:
            sensors("sensors-capture")
            bag = out / "bag"
            start("record", [str(ROOT / "scripts/site-mapping.sh"), str(bag)])
            started = time.monotonic()
            finish = f"{args.duration:g} 秒后自动收尾" if args.duration else "完成后按 Enter 收尾"
            print(f"录制中，{finish}。收尾前回到起始位置和朝向并停车；Ctrl-C 取消。", flush=True)
            while True:
                check()
                monitor.spin()
                if not monitor.healthy():
                    raise RuntimeError("传感器数据停更，已终止本轮采集")
                if shutil.disk_usage(out).free < 512 * 1024**2:
                    raise RuntimeError("磁盘余量不足，停止录制以保留已有数据")
                if args.duration and time.monotonic()-started >= args.duration:
                    break
                if not args.duration and select.select([sys.stdin], [], [], 0)[0]:
                    if not sys.stdin.readline():
                        raise RuntimeError("终端输入断开，取消本轮采集")
                    break
            stop("record")
            monitor.close(); monitor = None
            stop("sensors-capture")
            result["stages"].append("capture_and_clean_shutdown")
        result["bag"] = str(bag)
        run("bag-quality", [sys.executable, str(ROOT / "scripts/validation/verify-mapping-bag.py"), "--bag", str(bag), "--output", str(out / "bag-quality.json")], 900)
        command = [sys.executable, str(ROOT / "scripts/prepare-vehicle-navigation.py"), "--bag", str(bag), "--vehicle-config", str(args.vehicle_config), "--output", str(out / "map")]
        if args.arena_config:
            command += ["--arena-config", str(args.arena_config)]
        run("export-map", command)
        run("map-quality", [sys.executable, str(ROOT / "scripts/validation/verify-map-quality.py"), "--map-config", str(out / "map"), "--output", str(out / "map-quality.json")])
        sensors("sensors-reload")
        monitor.close(); monitor = None
        os.environ["INTRAMURAL_NAVIGATION_CONFIG"] = str(out / "map")
        start("navigation", [str(ROOT / "scripts/vehicle-navigation.sh")])
        run("reload-check", [sys.executable, str(ROOT / "scripts/validation/verify-vehicle-standby.py"), "--output", str(out / "reload-check"), "--map-config", str(out / "map")], 150)
        result.update(status="complete", localization_reload_verified=True,
                      map_quality=json.loads((out / "map-quality.json").read_text()), map_config=str(out / "map"))
        exit_code = 0
    except KeyboardInterrupt:
        result.update(status="cancelled", error="用户中断；已录制数据保留，未宣称验收通过")
        exit_code = 130
    except Exception as error:
        result.update(status="failed", error=str(error))
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        signal.signal(signal.SIGHUP, signal.SIG_IGN)
        if monitor:
            monitor.close()
        cleanup_errors = []
        if "navigation" in children:
            try:
                shutdown_navigation()
            except Exception as error:
                cleanup_errors.append(str(error))
        for name in reversed(list(children)):
            try:
                stop(name, cancelling=result["status"] != "complete")
            except Exception as error:
                cleanup_errors.append(str(error))
        if cleanup_errors:
            result.update(status="failed", cleanup_errors=cleanup_errors)
            exit_code = 1
        result["remaining_pids"] = [pid for group in groups for pid in processes(group)]
        result["processes_released"] = not result["remaining_pids"]
        save()
        lock.close()
    print(json.dumps(result, ensure_ascii=False, indent=2), flush=True)
    if result["status"] == "complete":
        print("流程完成，重启定位检查通过；地图目录:", out / "map", flush=True)
        if not result["map_quality"]["navigation_grid_has_clearance"]:
            print("注意：二维栅格没有满足车体半径的空闲区域，不能据此开放自主导航。", flush=True)
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
