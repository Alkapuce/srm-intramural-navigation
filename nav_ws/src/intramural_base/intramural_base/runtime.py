"""Finish the active callback before releasing ROS during process shutdown."""

import signal
import os
import time
import gc
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.signals import SignalHandlerOptions


def shutdown_before_peers(process):
    """Quiesce the owned serial process before launch tears down its DDS peers."""
    from launch.actions import RegisterEventHandler
    from launch.event_handlers import OnShutdown

    def stop_process(event, context):
        pid = process.process_details.get('pid')
        if pid is None:
            return []
        try:
            os.kill(pid, signal.SIGINT)
        except ProcessLookupError:
            return []
        deadline = time.monotonic()+5
        while time.monotonic()<deadline:
            try:
                if os.waitid(os.P_PID,pid,os.WEXITED|os.WNOHANG|os.WNOWAIT) is not None:
                    return []
            except ChildProcessError:
                return []
            time.sleep(.01)
        raise RuntimeError('Serial bridge did not finish zero/close before peer shutdown')

    # Register after the processes: launch prepends handlers, so this one runs first.
    return RegisterEventHandler(OnShutdown(on_shutdown=stop_process))


def run(factory, cleanup=None, args=None):
    stopping = False

    def stop(signum, frame):
        nonlocal stopping
        stopping = True

    # Raising KeyboardInterrupt inside native message conversion can become a pybind error.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    handlers = {s: signal.signal(s, stop) for s in (signal.SIGINT, signal.SIGTERM)}
    executor = SingleThreadedExecutor()
    node = None
    frozen = False
    try:
        node = factory()
        executor.add_node(node)
        # Startup objects are long-lived; repeated full-heap scans caused 10–39 ms control stalls.
        gc.freeze()
        frozen = True
        while not stopping and rclpy.ok():
            executor.spin_once(timeout_sec=.1)
    finally:
        try:
            executor.shutdown()
            if node is not None:
                try:
                    if cleanup is not None:
                        cleanup(node)
                finally:
                    node.destroy_node()
        finally:
            rclpy.try_shutdown()
            if frozen:gc.unfreeze()
            for sig, handler in handlers.items():
                signal.signal(sig, handler)
