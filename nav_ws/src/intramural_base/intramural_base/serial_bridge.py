"""One 19-byte command every 10 ms; the mission guard remains sole cmd_vel owner."""

import json
import os
import termios
import time
import tty
from pathlib import Path

import rclpy
from .runtime import run
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String

from .protocol import FeedbackParser, command_frame
from .transport import CommandPolicy, validate_vehicle_config


def open_serial(path):
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    tty.setraw(fd)
    attrs = termios.tcgetattr(fd)
    attrs[2] |= termios.CLOCAL | termios.CREAD
    attrs[2] &= ~termios.PARENB
    attrs[2] &= ~termios.CSTOPB
    if hasattr(termios, "CRTSCTS"):
        attrs[2] &= ~termios.CRTSCTS
    attrs[4] = attrs[5] = termios.B115200
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    return fd


def write_frame(fd, frame):
    # Never queue another frame behind a partial frame. A write failure is fatal.
    count = os.write(fd, frame)
    if count != 19:
        raise OSError(f"Partial serial frame: {count}/19")


class SerialBridge(Node):
    def __init__(self):
        super().__init__("serial_bridge")
        self.declare_parameter("device", "")
        self.declare_parameter("device_file", "")
        self.declare_parameter("calibration", "/work/config/base-sim.json")
        self.declare_parameter("simulation", False)
        config = json.loads(Path(self.get_parameter("calibration").value).read_text())
        self.simulation = bool(self.get_parameter("simulation").value)
        validate_vehicle_config(config, self.simulation)
        self.policy = CommandPolicy(config["body_to_protocol"], config["max_body_speed_mps"],
            config["max_body_acceleration_mps2"], config["max_protocol_axis"],
            config["input_timeout_seconds"])
        name = self.get_parameter("device").value
        file = self.get_parameter("device_file").value
        if file:
            deadline = time.monotonic() + 10
            while not Path(file).exists() and time.monotonic() < deadline:
                time.sleep(0.02)
            name = Path(file).read_text().strip()
        if not name.startswith("/dev/"):
            raise ValueError("Explicit serial device under /dev required")
        self.fd = open_serial(name)
        self.seq = 0
        self.sent = 0
        self.zero = 0
        self.last_write = None
        self.max_interval = 0
        self.min_interval = None
        self.late_intervals = 0
        self.parser = FeedbackParser()
        self.feedback_pub = self.create_publisher(String, "base/feedback", 10)
        self.status_pub = self.create_publisher(String, "base/status", 10)
        self.create_subscription(Twist, "cmd_vel", self.command, 10)
        self.create_subscription(Bool, "mission/enabled", self.enable,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_timer(0.01, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.create_timer(1.0, self.status, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def enable(self, msg):
        self.policy.enabled = msg.data

    def command(self, msg):
        self.policy.receive(msg.linear.x, msg.linear.y, msg.angular.z)

    def tick(self):
        now = time.monotonic()
        values, reason = self.policy.tick(now)
        write_frame(self.fd, command_frame(*values, self.seq))
        self.seq = (self.seq + 1) & 255
        self.sent += 1
        self.zero += int(values == (0.0, 0.0, 0.0))
        if self.last_write is not None:
            interval = now - self.last_write
            self.max_interval = max(self.max_interval, interval)
            self.min_interval = interval if self.min_interval is None else min(self.min_interval, interval)
            self.late_intervals += int(interval > 0.02)
        self.last_write = now
        try:
            while True:
                chunk = os.read(self.fd, 1024)
                if not chunk:
                    break
                for packet in self.parser.feed(chunk):
                    self.feedback_pub.publish(String(data=json.dumps(packet.__dict__)))
        except BlockingIOError:
            pass
        self.last_reason = reason

    def status(self):
        self.status_pub.publish(String(data=json.dumps({"sent": self.sent, "zero": self.zero,
            "seq_next": self.seq, "max_interval_wall_seconds": self.max_interval,
            "min_interval_wall_seconds": self.min_interval,
            "late_intervals_over_20ms": self.late_intervals,
            "reason": self.last_reason if self.sent else "startup",
            "bad_feedback_bytes": self.parser.discarded, "simulation": self.simulation})))

    def close(self):
        if hasattr(self, "fd"):
            try:
                for _ in range(3):
                    write_frame(self.fd, command_frame(0.0, 0.0, 0.0, self.seq))
                    self.seq = (self.seq + 1) & 255
                    time.sleep(0.01)
            finally:
                os.close(self.fd)
                del self.fd


def main():
    run(SerialBridge, SerialBridge.close)
