"""Finish the active callback before releasing ROS during process shutdown."""

import signal
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.signals import SignalHandlerOptions


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
    try:
        node = factory()
        executor.add_node(node)
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
            for sig, handler in handlers.items():
                signal.signal(sig, handler)
