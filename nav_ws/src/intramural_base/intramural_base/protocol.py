"""SRM navigation frames; byte layouts follow navigation_protocol.md."""

import math
import struct
from dataclasses import dataclass


def crc8(data):
    value = 0xFF
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = (value >> 1) ^ (0x8C if value & 1 else 0)
    return value


def crc16(data):
    value = 0xFFFF
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = (value >> 1) ^ (0x8408 if value & 1 else 0)
    return value


def command_frame(vx, vy, wz, seq):
    if not all(math.isfinite(v) for v in (vx, vy, wz)):
        raise ValueError("Nonfinite velocity")
    if not 0 <= seq <= 255:
        raise ValueError("Sequence outside uint8")
    prefix = bytes((0xA5, 12, 0, seq))
    content = prefix + bytes((crc8(prefix),)) + struct.pack("<fff", vx, vy, wz)
    return content + struct.pack("<H", crc16(content))


def decode_command(frame):
    if len(frame) != 19 or frame[:3] != b"\xa5\x0c\x00":
        raise ValueError("Invalid command length/header")
    if crc8(frame[:4]) != frame[4] or crc16(frame[:17]) != struct.unpack_from("<H", frame, 17)[0]:
        raise ValueError("Invalid command CRC")
    values = struct.unpack_from("<fff", frame, 5)
    if not all(map(math.isfinite, values)):
        raise ValueError("Nonfinite command")
    return frame[3], values


def feedback_frame(cmd_id, payload, seq):
    if not 0 <= seq <= 255 or not 0 <= cmd_id <= 65535 or len(payload) > 255:
        raise ValueError("Invalid feedback fields")
    prefix = bytes((0xA5, len(payload), 0, seq))
    body = prefix + bytes((crc8(prefix),)) + struct.pack("<H", cmd_id) + payload
    return body + struct.pack("<H", crc16(body))


@dataclass(frozen=True)
class Feedback:
    seq: int
    cmd_id: int
    values: dict


class FeedbackParser:
    """Bounded stream parser; resynchronize one byte at a time after corrupt input."""

    LENGTHS = {1: 11, 11: 13}

    def __init__(self):
        self.buffer = bytearray()
        self.discarded = 0

    def feed(self, chunk):
        self.buffer.extend(chunk)
        if len(self.buffer) > 4096:
            self.discarded += len(self.buffer) - 4096
            del self.buffer[:-4096]
        packets = []
        while len(self.buffer) >= 5:
            if self.buffer[0] != 0xA5:
                del self.buffer[0]
                self.discarded += 1
                continue
            length = self.buffer[1] | self.buffer[2] << 8
            if length not in self.LENGTHS.values() or crc8(self.buffer[:4]) != self.buffer[4]:
                del self.buffer[0]
                self.discarded += 1
                continue
            total = length + 9
            if len(self.buffer) < total:
                break
            frame = self.buffer[:total]
            cmd_id = struct.unpack_from("<H", frame, 5)[0]
            if self.LENGTHS.get(cmd_id) != length or crc16(frame[:-2]) != struct.unpack_from("<H", frame, total - 2)[0]:
                del self.buffer[0]
                self.discarded += 1
                continue
            data = frame[7:-2]
            if cmd_id == 1:
                first, remaining, stamp = struct.unpack("<BHQ", data)
                values = {"game_type": first & 15, "game_progress": first >> 4,
                          "stage_remain_time": remaining, "sync_time_stamp": stamp}
            else:
                fields = struct.unpack("<BBHHHHHB", data)
                values = dict(zip(("robot_id", "robot_level", "current_HP", "maximum_HP",
                    "shooter_barrel_cooling_value", "shooter_barrel_heat_limit",
                    "chassis_power_limit", "power_outputs"), fields))
            packets.append(Feedback(frame[3], cmd_id, values))
            del self.buffer[:total]
        return packets
