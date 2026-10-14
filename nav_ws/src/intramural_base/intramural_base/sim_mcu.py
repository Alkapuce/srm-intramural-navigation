"""Explicit no-watchdog pseudo-terminal MCU; last valid command is held."""

import argparse
import errno
import json
import math
import os
import struct
import time
import tty
from pathlib import Path

import rclpy
from .runtime import run
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_srvs.srv import SetBool

from .protocol import decode_command, feedback_frame


class SimMCU(Node):
    def __init__(self, device_file):
        super().__init__("sim_mcu")
        self.master, self.slave = os.openpty()
        tty.setraw(self.slave)
        os.set_blocking(self.master, False)
        path = Path(device_file)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(os.ttyname(self.slave) + "\n")
        self.path = path
        self.link = True
        self.buffer = bytearray()
        self.command = (0.0, 0.0, 0.0)
        self.frames = 0
        self.bad_bytes = 0
        self.seq_gaps = 0
        self.last_seq = None
        self.last_frame = None
        self.min_interval = None
        self.max_interval = 0.0
        self.last_feedback = 0.0
        self.pub = self.create_publisher(Twist, "cmd_vel_transport", 10)
        self.status_pub = self.create_publisher(String, "base/sim_status", 10)
        self.create_service(SetBool, "base/link_enabled", self.set_link)
        self.create_timer(0.002, self.poll, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.create_timer(0.025, self.publish, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.create_timer(1.0, self.status, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def set_link(self, request, response):
        self.link = request.data
        response.success = True
        response.message = "Simulated link enabled" if self.link else "Simulated link disconnected; command held"
        return response

    def poll(self):
        try:
            while True:
                part = os.read(self.master, 4096)
                if not part:
                    break
                if self.link:
                    self.buffer.extend(part)
                # A disconnected wire drops bytes; it does not queue old commands.
        except BlockingIOError:
            pass
        except OSError as exc:
            if exc.errno != errno.EIO:
                raise
        if len(self.buffer) > 4096:
            self.bad_bytes += len(self.buffer) - 4096
            del self.buffer[:-4096]
        while len(self.buffer) >= 19:
            if self.buffer[:3] != b"\xa5\x0c\x00":
                del self.buffer[0]
                self.bad_bytes += 1
                continue
            try:
                seq, values = decode_command(self.buffer[:19])
            except ValueError:
                del self.buffer[0]
                self.bad_bytes += 1
                continue
            del self.buffer[:19]
            now = time.monotonic()
            if self.last_seq is not None and seq != (self.last_seq + 1) & 255:
                self.seq_gaps += 1
            self.last_seq = seq
            if self.last_frame is not None:
                dt = now - self.last_frame
                self.min_interval = dt if self.min_interval is None else min(self.min_interval, dt)
                self.max_interval = max(self.max_interval, dt)
            self.last_frame = now
            self.command = values  # No MCU timeout: intentionally hold after link loss.
            self.frames += 1
            if now - self.last_feedback > 0.1:
                payload = struct.pack("<BHQ", 0xAA, 300, 0x1122334455667788)
                try:
                    os.write(self.master, feedback_frame(1, payload, seq))
                except BlockingIOError:
                    pass
                self.last_feedback = now

    def publish(self):
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = self.command
        self.pub.publish(msg)

    def status(self):
        self.status_pub.publish(String(data=json.dumps({"frames": self.frames,
            "last_seq": self.last_seq, "seq_gaps": self.seq_gaps,
            "bad_bytes": self.bad_bytes, "command": self.command,
            "link_enabled": self.link, "min_interval_wall_seconds": self.min_interval,
            "max_interval_wall_seconds": self.max_interval,
            "seconds_since_last_frame": None if self.last_frame is None else time.monotonic() - self.last_frame,
            "no_mcu_watchdog": True})))

    def close(self):
        self.path.unlink(missing_ok=True)
        os.close(self.master)
        os.close(self.slave)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--device-file", required=True)
    arguments, ros_args = parser.parse_known_args()
    run(lambda: SimMCU(arguments.device_file), SimMCU.close, args=ros_args)
