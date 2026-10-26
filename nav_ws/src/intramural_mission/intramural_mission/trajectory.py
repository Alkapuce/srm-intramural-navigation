"""Immutable ordered route, arc-length projection and vector-acceleration profile."""

from dataclasses import dataclass
from collections import deque
from functools import cached_property
import hashlib
import json
import math
import numpy as np
from intramural_mission.safety import response_horizon


@dataclass(frozen=True)
class RoutePlan:
    route_id: str
    points: tuple
    s: tuple
    tangent: tuple
    curvature: tuple
    speed: tuple
    gates: tuple
    clearance: tuple

    def __post_init__(self):
        # NumPy scalar reducers held the queue feeder's GIL while control waited to resume.
        for name in ('points','tangent'):
            object.__setattr__(self,name,tuple(tuple(map(float,row)) for row in getattr(self,name)))
        for name in ('s','curvature','speed','clearance'):
            object.__setattr__(self,name,tuple(map(float,getattr(self,name))))

    def __getstate__(self):
        state=self.__dict__.copy()
        state.pop('_arrays',None)
        return state

    @cached_property
    def _arrays(self):
        return tuple(np.asarray(v) for v in (self.points,self.s,self.tangent,self.speed))

    @classmethod
    def build(cls, route, limits, clearance=None, map_id=""):
        for key in (
            "max_speed",
            "max_acceleration",
            "lateral_acceleration",
            "braking_acceleration",
        ):
            if not math.isfinite(limits[key]) or limits[key] <= 0:
                raise ValueError("Positive finite motion limits required")
        for key in ("max_jerk", "parking_tolerance", "finish_distance", "terminal_deceleration"):
            if key in limits and (not math.isfinite(limits[key]) or limits[key] <= 0):
                raise ValueError("Positive finite trajectory constraints required")
        for key in ("response_delay", "response_tau", "tracking_prediction_delay", "normal_gain", "tangent_gain", "velocity_gain"):
            if key in limits and (not math.isfinite(limits[key]) or limits[key] < 0):
                raise ValueError("Nonnegative finite response parameters required")
        if limits["lateral_acceleration"] > limits["max_acceleration"]:
            raise ValueError("Lateral acceleration exceeds vector limit")
        points, gates = [], []
        for phase, path in route:
            for p in path:
                if not points or math.dist(p, points[-1]) > 1e-7:
                    points.append(tuple(float(v) for v in p))
            gates.append((phase, len(points) - 1))
        p = np.asarray(points)
        if len(p) < 3 or not np.isfinite(p).all():
            raise ValueError("Route needs finite, nonduplicate geometry")
        ds = np.linalg.norm(np.diff(p, axis=0), axis=1)
        s = np.r_[0.0, np.cumsum(ds)]
        d = np.gradient(p, s, axis=0)
        norm = np.linalg.norm(d, axis=1)
        if min(norm) < 1e-6:
            raise ValueError("Route contains a reversal; replan the connector")
        tangent = d / norm[:, None]
        curvature = np.zeros(len(p))
        a, b = np.diff(p, axis=0)[:-1], np.diff(p, axis=0)[1:]
        curvature[1:-1] = (
            2
            * (a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0])
            / (ds[:-1] * ds[1:] * np.linalg.norm(a + b, axis=1)).clip(1e-12)
        )
        curvature[[0, -1]] = curvature[[1, -2]]
        speed_scale = limits.get('route_speed_scale', 1.)
        if not math.isfinite(speed_scale) or not 0 < speed_scale <= 1:
            raise ValueError('Route speed scale must be in (0, 1]')
        cap = np.full(len(p), limits["max_speed"], dtype=float)
        cap = np.minimum(
            cap,
            speed_scale * np.sqrt(
                limits["lateral_acceleration"] / np.maximum(np.abs(curvature), 1e-9)
            ),
        )
        curvature_rate = np.abs(np.gradient(curvature, s)) + curvature**2
        cap = np.minimum(
            cap, speed_scale * np.cbrt(limits.get("max_jerk", 6.0) / np.maximum(curvature_rate, 1e-9))
        )
        # Planned parking uses commanded deceleration; emergency stopping stays independent.
        braking = min(limits.get("terminal_deceleration", limits["braking_acceleration"]), limits["max_acceleration"])
        # Tracker preview and lag inversion already advance planned parking deceleration.
        delay = (0.0 if "terminal_deceleration" in limits
                 else limits.get("response_delay", 0.2) + 0.3)
        cap = np.minimum(
            cap,
            np.sqrt((braking * delay) ** 2 + 2 * braking * (s[-1] - s))
            - braking * delay,
        )
        gaps = (
            np.full(len(p), math.inf)
            if clearance is None
            else np.asarray(clearance(p), dtype=float)
        )
        if np.any(gaps <= 0) or np.isnan(gaps).any():
            raise ValueError("Route footprint intersects occupied or unknown space")
        # Reserve curvature acceleration before allocating tangential acceleration.
        speed = cap.copy()
        speed[0] = speed[-1] = 0.0
        for _ in range(6):
            for order in (range(len(p) - 2, -1, -1), range(1, len(p))):
                backward = order.step < 0
                for i in order:
                    j = i + 1 if backward else i - 1
                    lateral = (
                        max(abs(curvature[i]), abs(curvature[j]))
                        * max(speed[i], speed[j]) ** 2
                    )
                    acc = math.sqrt(
                        max(0.0, limits["max_acceleration"] ** 2 - lateral**2)
                    )
                    speed[i] = min(
                        speed[i], math.sqrt(speed[j] ** 2 + 2 * acc * abs(s[i] - s[j]))
                    )
        payload = {"points": points, "gates": gates, "limits": limits, "map": map_id}
        identity = hashlib.sha256(
            json.dumps(payload, sort_keys=True, allow_nan=False).encode()
        ).hexdigest()[:20]
        return cls(
            identity,
            tuple(points),
            tuple(s),
            tuple(map(tuple, tangent)),
            tuple(curvature),
            tuple(speed),
            tuple(gates),
            tuple(gaps),
        )

    def sample(self, s):
        points, lengths, tangents, speeds = self._arrays
        s = min(max(float(s), 0.), self.s[-1])
        i = min(
            len(lengths) - 2, max(0, int(np.searchsorted(lengths, s, side="right") - 1))
        )
        f = (s - lengths[i]) / (lengths[i + 1] - lengths[i])
        p = (1 - f) * points[i] + f * points[i + 1]
        t = (1 - f) * tangents[i] + f * tangents[i + 1]
        t /= max(1e-9, np.linalg.norm(t))
        return p, t, float(np.interp(s, lengths, speeds))


class Progress:
    """Project only inside the physically reachable ordered window, never the later crossing."""

    def __init__(self, plan):
        self.plan, self.s, self.phase = plan, 0.0, 0
        self.points,self.lengths = plan._arrays[:2]

    def project(self, pose, travel, max_distance=None):
        p = np.asarray(pose)
        ss,points = self.lengths,self.points
        lo = max(0, int(np.searchsorted(ss, self.s, side="right") - 2))
        hi = min(len(ss) - 1, int(np.searchsorted(ss, self.s + travel, side="right")))
        end = max(lo + 1, hi)
        a = points[lo:end]
        v = points[lo+1:end+1] - a
        f = np.clip(np.sum((p-a)*v,axis=1)/np.sum(v*v,axis=1),0.,1.)
        distances = np.linalg.norm(p-a-f[:,None]*v,axis=1)
        nearest = int(np.argmin(distances))
        distance = float(distances[nearest])
        if max_distance is not None and distance > max_distance:
            return distance
        i = lo + nearest
        self.s = max(self.s, float(ss[i]+f[nearest]*(ss[i+1]-ss[i])))
        while (
            self.phase + 1 < len(self.plan.gates)
            and self.s >= self.plan.s[self.plan.gates[self.phase][1]] - 1e-6
        ):
            self.phase += 1
        return distance


class OmniTracker:
    """Tangent feedforward plus bounded position/velocity feedback; body yaw remains observed."""

    def __init__(self, plan, limits):
        self.plan, self.limits, self.progress = plan, limits, Progress(plan)
        self.command = np.zeros(2)
        self.acceleration = np.zeros(2)
        self.previous_pose = None
        self.previous_stamp = None
        self.elapsed = 0.0
        self.velocity = np.zeros(2)
        self.correction = np.zeros(2)
        self.parking_samples = None
        self.speed_scale = 1.
        models = limits.get('response_models', [])
        self.response_tau = limits.get('response_tau', float(np.median(models,axis=0)[1]) if models else .15)

    def stop(self):
        self.command[:] = 0.0
        self.acceleration[:] = 0.0
        self.correction[:] = 0.0

    def observe(self, pose, dt, stamp=None):
        dt = min(max(float(dt), 0.005), 0.1)
        p = np.asarray(pose)
        if stamp is None:
            self.elapsed += dt
            stamp = self.elapsed
        if self.previous_pose is not None and stamp > self.previous_stamp:
            observed_dt = stamp - self.previous_stamp
            observed = (p - self.previous_pose) / observed_dt
            if np.linalg.norm(observed) < 2 * self.limits["max_speed"]:
                # Source-rate exponential filtering still smooths 10 Hz pose differences.
                self.velocity += -math.expm1(-observed_dt / 0.05) * (
                    observed - self.velocity
                )
        if self.previous_stamp is None or stamp > self.previous_stamp:
            self.previous_pose = p.copy()
            self.previous_stamp = stamp
            if self.parking_samples is not None:
                self.parking_samples.append((stamp, p.copy()))
                while len(self.parking_samples) > 2 and stamp-self.parking_samples[1][0] >= .25:
                    self.parking_samples.popleft()

    def step(self, pose, yaw, dt, pose_age=0.0, stamp=None):
        dt = min(max(float(dt), 0.005), 0.1)
        p = np.asarray(pose)
        self.observe(p, dt, stamp)
        error = self.progress.project(
            p,
            self.limits["max_speed"] * (dt + pose_age) + 0.12,
            self.limits["max_tracking_error"],
        )
        s = self.progress.s
        remaining = self.plan.s[-1] - s
        delay = self.limits.get("tracking_prediction_delay", self.limits.get("response_delay", 0.2)) + max(0.0, pose_age)
        predicted = p + self.velocity * delay
        lookahead = max(.02, np.linalg.norm(self.velocity)*delay)
        lo = max(0,int(np.searchsorted(self.progress.lengths,s))-1)
        hi = min(len(self.plan.s),int(np.searchsorted(self.progress.lengths,s+lookahead))+1)
        curvature = max(map(abs,self.plan.curvature[lo:hi]),default=0.)
        ahead = min(self.plan.s[-1],s+min(lookahead,.5/max(curvature,1e-9)))
        target, tangent, reference = self.plan.sample(ahead)
        reference *= self.speed_scale
        offset = target - predicted
        along = float(np.dot(offset, tangent))
        normal = offset - along * tangent
        feedback = (
            self.limits.get("normal_gain", 1.5) * normal
            + self.limits.get("tangent_gain", 1.0) * along * tangent
        )
        feedback += self.limits.get("velocity_gain", 0.1) * (
            reference * tangent - self.velocity
        )
        # Smooth noisy corrections independently of reference feedforward and immediate stops.
        self.correction += -math.expm1(-dt / .05) * (feedback-self.correction)
        feedback = self.correction.copy()
        desired = reference * tangent + feedback
        # Invert the fitted velocity lag for the reference acceleration, before bounded correction.
        _, next_tangent, next_reference = self.plan.sample(ahead + max(.05,reference)*.05)
        next_reference *= self.speed_scale
        feedforward_acceleration = (next_reference*next_tangent-reference*tangent)/.05
        feedforward_acceleration *= min(1., self.limits['max_acceleration']/max(1e-9,np.linalg.norm(feedforward_acceleration)))
        desired += self.response_tau * feedforward_acceleration
        cap = self.limits["max_speed"]
        if error > 0.08:
            cap *= max(0.25, 1 - (error - 0.08) / 0.25)
        desired *= min(1.0, cap / max(1e-9, np.linalg.norm(desired)))
        acceleration = (desired - self.command) / dt
        acceleration *= min(
            1.0,
            self.limits["max_acceleration"] / max(1e-9, np.linalg.norm(acceleration)),
        )
        # Jerk shapes the reference profile; filtering corrective acceleration again adds control lag.
        self.acceleration = acceleration
        command = self.command + self.acceleration * dt
        command *= min(
            1.0, self.limits["max_speed"] / max(1e-9, np.linalg.norm(command))
        )
        self.command = command
        parking_brake = self.parking_samples is not None or (
            remaining < max(self.limits.get("finish_distance", 0.06),
                            self.limits.get("parking_tolerance", 0.08))
            and np.linalg.norm(p - np.asarray(self.plan.points[-1]))
            < self.limits.get("parking_tolerance", 0.08)
            and np.linalg.norm(p + self.velocity*(response_horizon(self.limits)+max(0.,pose_age))
                               - np.asarray(self.plan.points[-1]))
            < self.limits.get("parking_tolerance", 0.08)
        )
        # Command zero when the stopping tail fits; do not creep until speed is already zero.
        if parking_brake:
            if self.parking_samples is None:
                self.parking_samples = deque([(self.previous_stamp, p.copy())])
            self.stop()
            command = self.command
        # Hold zero through pose jitter; only distinct source poses can establish rest.
        parked, stationary, window, parking_speed = p, False, 0., None
        if parking_brake and len(self.parking_samples) >= 3:
            times = np.array([t for t, _ in self.parking_samples])
            positions = np.array([xy for _, xy in self.parking_samples])
            window = float(times[-1]-times[0])
            if window >= .25:
                parked = positions.mean(axis=0)
                times -= times.mean()
                parking_speed = float(np.linalg.norm(np.sum(times[:,None]*(positions-parked),axis=0)/np.sum(times**2)))
                stationary = parking_speed < .06 and np.max(np.linalg.norm(positions-parked,axis=1)) < .02
        parking_error = float(np.linalg.norm(parked-np.asarray(self.plan.points[-1])))
        done = stationary and parking_error < self.limits.get('parking_tolerance', .08)
        co, si = math.cos(yaw), math.sin(yaw)
        body = np.array(
            [co * command[0] + si * command[1], -si * command[0] + co * command[1]]
        )
        return body, dict(
            s=s,
            remaining=remaining,
            phase=self.progress.phase,
            error=error,
            done=bool(done),
            parking_brake=bool(parking_brake),
            parking_window_seconds=window,
            parking_speed=parking_speed,
            parking_error=parking_error,
            parking_problem='PARKING_OUTSIDE_TOLERANCE' if stationary and not done else '',
            speed_reference=reference,
            velocity_observed=self.velocity.tolist(),
            feedforward=(reference * tangent).tolist(),
            feedforward_acceleration=feedforward_acceleration.tolist(),
            feedback=feedback.tolist(),
            command_map=command.tolist(),
        )
