"""Device-free planar response model shared by planning and backend comparisons."""

from collections import deque
import math
import numpy as np
from intramural_base.transport import CommandPolicy
from intramural_mission.trajectory import OmniTracker


class Plant:
    def __init__(self, pose, delay, tau, dt=.025):
        if not all(math.isfinite(v) for v in (delay,tau,dt)) or min(delay,tau) < 0 or dt <= 0:
            raise ValueError("Invalid simulated response")
        self.pose, self.velocity = np.array(pose,dtype=float), np.zeros(2)
        self.dt, self.tau = dt,tau
        self.queue = deque(np.zeros(2) for _ in range(round(delay/dt)))

    def step(self, command):
        u = np.asarray(command,dtype=float)
        if u.shape != (2,) or not np.isfinite(u).all():
            raise ValueError("Finite planar command required")
        self.queue.append(u.copy())
        applied = self.queue.popleft()
        alpha = 1. if self.tau == 0 else 1-math.exp(-self.dt/self.tau)
        self.velocity += alpha*(applied-self.velocity)
        self.pose += self.velocity*self.dt


def predict_execution(plan, c, measured, plant_response=None):
    """Score executable model time, including response and the final zero-command tail."""
    if plant_response is None:
        models = c.get("response_models")
        # Recorded response predicts normal driving; collision latency is a separate bound.
        plant_response = np.median(models,axis=0) if models else (c.get("response_delay",.2),c.get("response_tau",.15))
    delay,tau = plant_response
    plant = Plant(plan.points[0],delay,tau)
    tracker = OmniTracker(plan,c)
    policy = CommandPolicy([[1,0],[0,1]],c["max_speed"],c["max_acceleration"],c["max_speed"])
    policy.enabled,policy.last_tick = True,-plant.dt
    minimum, maximum, finished = math.inf,0.,None
    turns = np.zeros(2)
    observed,source_stamp = plant.pose.copy(),0.
    pose_period = c.get("pose_update_period",.1)
    reference_turns = []
    for key in ("lower_pole","upper_pole"):
        relative = np.asarray(plan.points)-c[key]
        a,b = relative[:-1],relative[1:]
        reference_turns.append(np.sum(np.arctan2(a[:,0]*b[:,1]-a[:,1]*b[:,0],np.sum(a*b,axis=1)))/(2*math.pi))
    for tick in range(7200):
        t = tick*plant.dt
        if t-source_stamp >= pose_period-1e-9:
            observed,source_stamp = plant.pose.copy(),t
        command,detail = tracker.step(observed,0.,plant.dt,pose_age=t-source_stamp,stamp=source_stamp)
        if finished is not None:command = np.zeros(2)
        policy.receive(*command,0.,now=t)
        sent,_ = policy.tick(t)
        previous = plant.pose.copy()
        plant.step(sent[:2])
        for i,key in enumerate(("lower_pole","upper_pole")):
            a,b = previous-c[key],plant.pose-c[key]
            turns[i] += math.atan2(a[0]*b[1]-a[1]*b[0],np.dot(a,b))
        gap = float(measured.clearance([plant.pose])[0])
        minimum,maximum = min(minimum,gap),max(maximum,detail["error"])
        if gap <= 0 or detail["error"] > c["max_tracking_error"]:
            return dict(passed=False,reason="predicted_clearance" if gap <= 0 else "predicted_tracking",minimum_clearance=minimum)
        if finished is None and detail["done"]:finished = t+plant.dt
        if finished is not None and t-finished >= delay+5*tau:
            parking = float(np.linalg.norm(plant.pose-c["parking"]))
            winding = turns/(2*math.pi)
            passed = (parking < c.get("parking_tolerance",.08) and np.linalg.norm(plant.velocity) < .06
                      and np.max(np.abs(winding-reference_turns)) < .15)
            return dict(passed=bool(passed),duration=finished,parking_error=parking,
                        minimum_clearance=minimum,max_tracking_error=maximum,winding=winding.tolist(),pose_update_period=pose_period,
                        response_delay=float(delay),response_tau=float(tau))
    return dict(passed=False,reason="predicted_timeout",minimum_clearance=minimum)
