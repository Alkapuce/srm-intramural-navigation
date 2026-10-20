"""Start a mission and keep its progress and final result in the same terminal."""
import json
import math
from pathlib import Path
import sys
import time
from datetime import datetime
from zoneinfo import ZoneInfo

ROOT = Path(__file__).resolve().parents[2]
TERMINAL = ('DONE', 'FAULT', 'CANCELLED')


def read_run(folder):
    rows = []
    with (folder/'status.jsonl').open() as stream:
        for line in stream:
            if not line.endswith('\n'):  # The live writer may not have finished this line.
                break
            rows.append(json.loads(line))
            if rows[-1]['state'] in TERMINAL:
                break
    if not rows:
        raise ValueError('运行记录还没有完整状态')
    config = json.loads((folder/'manifest.json').read_text())['config']
    return rows, config


def report(rows, folder=None):
    config = {}
    if folder is not None and (folder/'status.jsonl').is_file():
        saved, config = read_run(folder)
        # A terminal status can arrive just before its log line is flushed.
        rows = saved if saved[-1]['state'] in TERMINAL else saved + rows[-1:]
    final = rows[-1]
    elapsed = final['elapsed_sim_seconds']
    history = final.get('history', [])
    route_file = folder/'route.json' if folder is not None else None
    stages = f"{len(history)}/{len(json.loads(route_file.read_text())['gates'])}" if route_file and route_file.is_file() else str(len(history))
    print(f"\n结果：{final['state']}  阶段记录：{stages}  阻挡：{final.get('blocked_count', 0)} 次")
    if final.get('reason'):
        print('原因：', final['reason'])
    if final.get('input_failure'):
        print('输入故障：', json.dumps(final['input_failure'], ensure_ascii=False))
    if history:
        started = history[0]['sim_time']
        clock = lambda stamp: datetime.fromtimestamp(stamp, ZoneInfo('Asia/Shanghai')).strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
        print(f"开始：{clock(started)}（北京时间）")
        if final['state'] in TERMINAL:
            print(f"结束：{clock(started+elapsed)}（北京时间）")
    print(f"总计时：{elapsed:.2f} 秒（从任务接受开始，包含等待）")
    first = next((r for r in rows if r.get('pose')), None)
    if first:
        departed = next((r['elapsed_sim_seconds'] for r in rows
                         if r.get('pose') and math.dist(r['pose'], first['pose']) > .05), None)
        if departed is not None:
            print(f"离开起步点 5 cm 前：{departed:.2f} 秒；之后到{'结束' if final['state'] in TERMINAL else '当前'}：{elapsed-departed:.2f} 秒（定位估计）")
    peak = max((r.get('tracking', {}).get('error', 0.) for r in rows), default=0.)
    print(f"最大跟踪误差：{peak:.3f} m")
    if config.get('parking') and final.get('pose'):
        print(f"终点位置偏差：{math.dist(config['parking'],final['pose']):.3f} m（定位估计）")
    print('运行 ID：', final.get('run_id', '未提供'))
    if folder is not None:
        print('记录目录：', folder)
    return 0 if final['state'] == 'DONE' else 130 if final['state'] == 'CANCELLED' else 2


def start(operation):
    import rclpy
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from std_msgs.msg import String
    from std_srvs.srv import Trigger

    # Keep ROS alive while Ctrl-C sends the cancellation request.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('mission_console')
    latest, received = None, time.monotonic()
    requested = False
    rows = []

    def receive(message):
        nonlocal latest, received
        latest, received = json.loads(message.data), time.monotonic()

    node.create_subscription(String, '/intramural_robot/mission/status', receive,
                             QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def call(name):
        nonlocal requested
        client = node.create_client(Trigger, '/intramural_robot/mission/'+name)
        try:
            if not client.wait_for_service(timeout_sec=5.):
                raise RuntimeError(f'任务服务 {name} 不可用')
            future = client.call_async(Trigger.Request())
            if name in ('start', 'start_here'):
                requested = True
            planning = name in ('start', 'start_here')
            began = time.monotonic()
            deadline = began+(90 if planning else 30)
            notice = began
            while not future.done() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.1)
                now = time.monotonic()
                if planning and not future.done() and now-notice >= 5:
                    print(f'规划接入中，已等待 {now-began:.0f} 秒…', flush=True)
                    notice = now
            if not future.done():
                raise RuntimeError(f'任务服务 {name} 响应超时')
            response = future.result()
            if not response.success:
                if name in ('start', 'start_here'):
                    requested = False
                raise RuntimeError(response.message)
            return response.message
        finally:
            node.destroy_client(client)

    def cancel():
        try:
            print('请求取消：', call('cancel'), flush=True)
            return True
        except Exception as error:
            print(f'取消未确认：{error}；请现场停机。', file=sys.stderr, flush=True)
            return False

    try:
        print('连接任务状态…', flush=True)
        deadline = time.monotonic()+5
        while latest is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        if latest is None:
            raise RuntimeError('没有收到任务状态，请确认 drive 正在运行')
        previous = (latest.get('run_id'), latest.get('run'))
        if latest['state'] == 'DONE':
            print('上一轮已完成，复位后开始新一轮。', flush=True)
            call('reset')
        print('开始任务，正在规划接入…', flush=True)
        print(call(operation.replace('-', '_')), flush=True)
        # Planning pauses status publication; start its watchdog after service acceptance.
        received = time.monotonic()
        print('持续显示进度；Ctrl-C 取消任务。', flush=True)
        shown, printed = None, 0.
        while True:
            rclpy.spin_once(node, timeout_sec=.1)
            if time.monotonic()-received > 2:
                raise RuntimeError('任务状态已停更，无法确认当前执行状态')
            if (latest.get('run_id'), latest.get('run')) == previous:
                continue
            rows.append(latest)
            key = (latest['state'], latest['segment'], latest.get('blocked_count'), latest.get('reason'))
            if key != shown or time.monotonic()-printed >= 1:
                print(f"{latest['state']:10} {latest.get('phase','')} 段 {latest['segment']+1} | "
                      f"{latest['elapsed_sim_seconds']:.1f} s | "
                      f"误差 {latest.get('tracking',{}).get('error',0.):.3f} m | "
                      f"挡停 {latest.get('blocked_count',0)} | {latest.get('reason','')}", flush=True)
                shown, printed = key, time.monotonic()
            if latest['state'] in TERMINAL:
                requested = False
                run_id = latest.get('run_id', '')
                folder = ROOT/'artifacts/navigation-runs'/run_id if run_id and Path(run_id).name == run_id else None
                print('\a', end='', flush=True)
                return report(rows, folder)
    except KeyboardInterrupt:
        print('\n操作者按 Ctrl-C 终止。', flush=True)
        if requested and cancel():
            deadline = time.monotonic()+3
            def ended():
                return ((latest.get('run_id'),latest.get('run')) != previous
                        and latest['state'] in TERMINAL)
            while not ended() and time.monotonic()<deadline:
                rclpy.spin_once(node, timeout_sec=.1)
            if ended():
                run_id = latest.get('run_id', '')
                folder = ROOT/'artifacts/navigation-runs'/run_id if run_id and Path(run_id).name == run_id else None
                return report(rows+[latest], folder)
            print('取消已请求，但尚未收到停机终态；请现场确认。', file=sys.stderr)
            return 2
        return 130
    except Exception as error:
        print(f'任务未正常完成：{error}', file=sys.stderr, flush=True)
        if requested:
            cancel()
        return 2
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def main():
    operation = sys.argv[1]
    if operation in ('start', 'start-here'):
        return start(operation)
    folder = Path(sys.argv[2]).expanduser().resolve() if len(sys.argv)>2 else max(
        (ROOT/'artifacts/navigation-runs').glob('*/manifest.json'), key=lambda p:p.stat().st_mtime_ns).parent
    rows, _ = read_run(folder)
    return report(rows, folder)


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (OSError, ValueError) as error:
        print(f'无法读取运行结果：{error}', file=sys.stderr)
        sys.exit(2)
