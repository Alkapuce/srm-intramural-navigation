"""Device-free planar response model shared by planning and backend comparisons."""

from collections import deque
import math
import numpy as np
from intramural_base.transport import CommandPolicy
from intramural_mission.trajectory import OmniTracker
from intramural_mission.safety import response_horizon,stopping_horizon


class Plant:
    def __init__(self, pose, delay, tau, dt=.025, acceleration_limit=None):
        if not all(math.isfinite(v) for v in (delay,tau,dt)) or min(delay,tau) < 0 or dt <= 0:
            raise ValueError("Invalid simulated response")
        self.pose, self.velocity = np.array(pose,dtype=float), np.zeros(2)
        self.dt, self.tau = dt,tau
        if acceleration_limit is not None and (not math.isfinite(acceleration_limit) or acceleration_limit<=0):
            raise ValueError('Positive drive acceleration limit required')
        self.acceleration_limit=acceleration_limit
        self.queue = deque(np.zeros(2) for _ in range(round(delay/dt)))

    def step(self, command):
        u = np.asarray(command,dtype=float)
        if u.shape != (2,) or not np.isfinite(u).all():
            raise ValueError("Finite planar command required")
        self.queue.append(u.copy())
        applied = self.queue.popleft()
        alpha = 1. if self.tau == 0 else 1-math.exp(-self.dt/self.tau)
        change=alpha*(applied-self.velocity)
        if self.acceleration_limit is not None:
            change*=min(1.,self.acceleration_limit*self.dt/max(1e-9,np.linalg.norm(change)))
        self.velocity += change
        self.pose += self.velocity*self.dt


def predict_motion(plan, limits, pose, velocity, command, acceleration, progress, horizon=2., models=None):
    """Production tracker and transport from a moving boundary, including stop tails."""
    models = models or limits.get('response_models', [(limits.get('response_delay', .2), limits.get('response_tau', .15))])
    trajectories = []
    for delay, tau in models:
        plant = Plant(pose, delay, tau, acceleration_limit=limits.get('drive_acceleration_limit'))
        plant.velocity = np.asarray(velocity).copy()
        plant.queue = deque(np.asarray(command).copy() for _ in plant.queue)
        tracker = OmniTracker(plan, limits)
        tracker.progress.s = progress
        tracker.velocity = np.asarray(velocity).copy()
        tracker.command = np.asarray(command).copy()
        tracker.acceleration = np.asarray(acceleration).copy()
        policy = CommandPolicy([[1, 0], [0, 1]], limits['max_speed'], limits['max_acceleration'], limits['max_speed'])
        policy.enabled, policy.last_tick = True, -plant.dt
        policy.previous = tuple(command)
        points, velocities, sent_commands, errors = [plant.pose.copy()], [plant.velocity.copy()], [], []
        observed,source_stamp=plant.pose.copy(),0.
        for i in range(math.ceil(horizon/plant.dt)):
            t = i*plant.dt
            if t-source_stamp>=limits.get('pose_update_period',.1)-1e-9:
                observed,source_stamp=plant.pose.copy(),t
            u, detail = tracker.step(observed, 0., plant.dt, pose_age=t-source_stamp, stamp=source_stamp)
            policy.receive(*u, 0., now=t)
            sent, _ = policy.tick(t)
            sent_commands.append(sent[:2])
            plant.step(sent[:2])
            points.append(plant.pose.copy())
            velocities.append(plant.velocity.copy())
            errors.append(detail['error'])
        # Batch a zero-command branch at every future state, retaining its actual delay queue.
        positions,brake_velocity=np.asarray(points).copy(),np.asarray(velocities).copy()
        braking=[positions.copy()];offsets=[0.];commands=np.asarray(sent_commands or [command])
        count=len(points);delay_steps=round(delay/plant.dt)
        force=plant.acceleration_limit
        duration=delay+5*tau+(float(max(np.linalg.norm(brake_velocity,axis=1)))/force if force else 0.)
        steps=max(2,math.ceil(duration/plant.dt));alpha=1. if tau==0 else 1-math.exp(-plant.dt/tau)
        for j in range(steps):
            if j<delay_steps:
                indices=np.arange(count)+j-delay_steps
                u=commands[np.clip(indices,0,len(commands)-1)].copy()
                u[indices<0]=command
            else:u=np.zeros_like(brake_velocity)
            change=alpha*(u-brake_velocity)
            if force is not None:
                change*=np.minimum(1.,force*plant.dt/np.maximum(1e-9,np.linalg.norm(change,axis=1)))[:,None]
            brake_velocity+=change;positions+=brake_velocity*plant.dt
            if j%2==1 or j==steps-1:
                braking.append(positions.copy());offsets.append((j+1)*plant.dt)
        timing=np.arange(count)*plant.dt
        trajectories.append((np.asarray(points),timing,np.stack(braking,axis=1),
            timing[:,None]+np.asarray(offsets),max(errors,default=0.)))
    return trajectories


def predict_execution(plan, c, measured, plant_response=None, required_clearance=0.):
    """Score executable model time, including response and the final zero-command tail."""
    if not math.isfinite(required_clearance) or required_clearance < 0:
        raise ValueError('Finite nonnegative execution clearance required')
    if plant_response is None:
        models = c.get("response_models")
        # Recorded response predicts normal driving; collision latency is a separate bound.
        plant_response = np.median(models,axis=0) if models else (c.get("response_delay",.2),c.get("response_tau",.15))
    delay,tau = plant_response
    plant = Plant(plan.points[0],delay,tau,acceleration_limit=c.get('drive_acceleration_limit'))
    tracker = OmniTracker(plan,c)
    policy = CommandPolicy([[1,0],[0,1]],c["max_speed"],c["max_acceleration"],c["max_speed"])
    policy.enabled,policy.last_tick = True,-plant.dt
    minimum, maximum, finished = math.inf,0.,None
    stopping_clearance = math.inf
    coast_horizon = response_horizon(c)
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
        if detail.get('parking_problem'):
            return dict(passed=False,reason=detail['parking_problem'],minimum_clearance=minimum,
                        minimum_stopping_clearance=stopping_clearance,parking_error=detail['parking_error'])
        if finished is not None:command = np.zeros(2)
        policy.receive(*command,0.,now=t)
        sent,_ = policy.tick(t)
        previous = plant.pose.copy()
        plant.step(sent[:2])
        for i,key in enumerate(("lower_pole","upper_pole")):
            a,b = previous-c[key],plant.pose-c[key]
            turns[i] += math.atan2(a[0]*b[1]-a[1]*b[0],np.dot(a,b))
        coast = plant.velocity*stopping_horizon(c,np.linalg.norm(plant.velocity))
        sweep = plant.pose + np.linspace(0.,1.,max(2,math.ceil(np.linalg.norm(coast)/(.5*measured.res))))[:,None]*coast
        clearances = measured.clearance(sweep)
        gap = float(clearances[0])
        stopping_clearance = min(stopping_clearance,float(min(clearances)))
        minimum,maximum = min(minimum,gap),max(maximum,detail["error"])
        if gap <= 0 or stopping_clearance <= 0 or detail["error"] > c["max_tracking_error"] or min(gap,stopping_clearance) < required_clearance:
            return dict(passed=False,reason="predicted_clearance" if gap <= 0 else "predicted_stopping_clearance" if stopping_clearance <= 0 else "predicted_tracking" if detail['error'] > c['max_tracking_error'] else "predicted_execution_clearance",
                        minimum_clearance=minimum,minimum_stopping_clearance=stopping_clearance)
        if finished is None and detail["done"]:finished = t+plant.dt
        if finished is not None and t-finished >= delay+5*tau:
            parking = float(np.linalg.norm(plant.pose-c["parking"]))
            winding = turns/(2*math.pi)
            passed = (parking < c.get("parking_tolerance",.08) and np.linalg.norm(plant.velocity) < .06
                      and np.max(np.abs(winding-reference_turns)) < .15)
            return dict(passed=bool(passed),duration=finished,parking_error=parking,
                        minimum_clearance=minimum,max_tracking_error=maximum,winding=winding.tolist(),pose_update_period=pose_period,
                        minimum_stopping_clearance=stopping_clearance,
                        response_delay=float(delay),response_tau=float(tau))
    return dict(passed=False,reason="predicted_timeout",minimum_clearance=minimum)
