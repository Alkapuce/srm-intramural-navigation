"""Physical-unit command boundary, independent of the frame codec."""

import math
import time


def validate_vehicle_config(config, simulation=False):
    """Require separate recorded validations before opening a real serial port."""
    if simulation:
        return
    for key in ("firmware_protocol_verified", "link_loss_stop_verified", "calibrated_for_vehicle"):
        if config.get(key) is not True:
            raise ValueError(f"Vehicle validation is missing: {key}")


class CommandPolicy:
    def __init__(self, matrix, speed_limit, acceleration_limit, protocol_limit, timeout=0.3):
        if len(matrix) != 2 or any(len(row) != 2 for row in matrix):
            raise ValueError("Expected 2x2 body-to-protocol matrix")
        self.matrix = [[float(v) for v in row] for row in matrix]
        det = self.matrix[0][0] * self.matrix[1][1] - self.matrix[0][1] * self.matrix[1][0]
        if not all(math.isfinite(v) for row in self.matrix for v in row) or abs(det) < 1e-8:
            raise ValueError("Body-to-protocol matrix must be finite and invertible")
        if not all(math.isfinite(v) and v > 0 for v in (speed_limit, acceleration_limit, protocol_limit, timeout)):
            raise ValueError("Positive finite safety limits required")
        self.speed_limit = speed_limit
        self.acceleration_limit = acceleration_limit
        self.protocol_limit = protocol_limit
        self.timeout = timeout
        self.command = (0.0, 0.0)
        self.updated = None
        self.previous = (0.0, 0.0)
        self.last_tick = None
        self.enabled = False
        self.invalid = False

    def receive(self, x, y, wz, now=None):
        now = time.monotonic() if now is None else now
        self.invalid = not all(math.isfinite(v) for v in (x, y, wz))
        self.command = (0.0, 0.0) if self.invalid else (x, y)
        self.updated = now

    def tick(self, now=None):
        now = time.monotonic() if now is None else now
        stale = self.updated is None or now - self.updated > self.timeout
        if stale or not self.enabled or self.invalid:
            self.previous = (0.0, 0.0)
            self.last_tick = now
            return (0.0, 0.0, 0.0), "stale" if stale else "disabled_or_invalid"
        x, y = self.command
        norm = math.hypot(x, y)
        if norm > self.speed_limit:
            x, y = x * self.speed_limit / norm, y * self.speed_limit / norm
        dt = 0.0 if self.last_tick is None else max(0.0, min(now - self.last_tick, 0.1))
        dx, dy = x - self.previous[0], y - self.previous[1]
        change = math.hypot(dx, dy)
        cap = self.acceleration_limit * dt
        if change > cap:
            x = self.previous[0] + dx * cap / change
            y = self.previous[1] + dy * cap / change
        self.previous = (x, y)
        self.last_tick = now
        a = self.matrix[0][0] * x + self.matrix[0][1] * y
        b = self.matrix[1][0] * x + self.matrix[1][1] * y
        if max(abs(a), abs(b)) > self.protocol_limit:
            scale = self.protocol_limit / max(abs(a), abs(b))
            a *= scale
            b *= scale
        return (a, b, 0.0), "active"
