"""Position-domain identification, with independent held-out validation."""

import numpy as np
from scipy.optimize import least_squares
from scipy.signal import lfilter


def fit_response(commands, observations):
    """Rows: [time, ux, uy], [time, x, y]. Times must share one clock."""
    u, p = np.array(commands, dtype=float), np.array(observations, dtype=float)
    if u.ndim != 2 or p.ndim != 2 or u.shape[1] != 3 or p.shape[1] != 3 or min(len(u), len(p)) < 20:
        raise ValueError("At least 20 timestamped commands and poses required")
    if not np.isfinite(u).all() or not np.isfinite(p).all() or min(np.diff(u[:, 0])) <= 0 or min(np.diff(p[:, 0])) <= 0:
        raise ValueError("Finite samples and strictly increasing times required")
    start = max(u[0, 0], p[0, 0] - 2)
    p = p[(p[:, 0] >= start) & (p[:, 0] <= u[-1, 0])].copy()
    if len(p) < 20 or p[-1, 0] - p[0, 0] < 3:
        raise ValueError("Too little overlapping motion data")
    u[:, 0] -= start
    p[:, 0] -= start
    t = np.arange(0, p[-1, 0] + .01, .01)
    active = np.max(np.abs(u[:, 1:]), axis=0) > .02
    if not active.any():
        raise ValueError("No input excitation")
    train = p[:, 0] < p[0, 0] + .7 * (p[-1, 0]-p[0, 0])
    if min(sum(train), sum(~train)) < 5:
        raise ValueError("Too few independent samples for a time-based held-out interval")
    target = p[:, 1:] - p[0, 1:]

    def design(parameters):
        delay, tau = parameters
        # Interpolate the recorded input, never infer it from controller targets or desired velocities.
        sampled = np.column_stack([np.interp(t-delay, u[:, 0], u[:, j], left=0) for j in (1, 2)])[:, active]
        alpha = 1 - np.exp(-.01 / tau)
        velocity = lfilter([alpha], [1, -(1-alpha)], sampled, axis=0)
        position = np.cumsum(velocity, axis=0) * .01
        return np.column_stack([np.ones(len(p))] + [np.interp(p[:, 0], t, x) for x in position.T])

    def solve(parameters):
        x = design(parameters)
        coefficients = np.linalg.lstsq(x[train], target[train], rcond=None)[0]
        return x @ coefficients, coefficients, x

    candidates = [least_squares(lambda parameters: (solve(parameters)[0][train] - target[train]).ravel(),
                                initial, bounds=([0, .005], [.8, .8]), diff_step=.02)
                  for initial in ([.05, .1], [.2, .15], [.4, .3])]
    fitted = min(candidates, key=lambda r: np.sum(r.fun**2))
    predicted, coefficients, x = solve(fitted.x)
    errors = np.linalg.norm(predicted-target, axis=1)
    jacobian_condition = float(np.linalg.cond(fitted.jac.T @ fitted.jac))
    gains = np.zeros((2, 2))
    gains[:, active] = coefficients[1:].T
    bound = bool(fitted.x[0] < .001 or fitted.x[0] > .79 or fitted.x[1] < .006 or fitted.x[1] > .79)
    identifiable = jacobian_condition < 1000 and not bound
    validation_rmse = float(np.sqrt(np.mean(errors[~train]**2)))
    result = dict(response_delay=float(fitted.x[0]), response_tau=float(fitted.x[1]),
                  gain_world_from_input=gains.tolist(), excited_axes=np.flatnonzero(active).tolist(),
                  training_rmse_m=float(np.sqrt(np.mean(errors[train]**2))),
                  validation_rmse_m=validation_rmse, validation_max_error_m=float(max(errors[~train])),
                  jacobian_condition=jacobian_condition, identifiable=identifiable,
                  parameters_at_bound=bound, train_samples=int(sum(train)), validation_samples=int(sum(~train)),
                  validation_span_s=float(np.ptp(p[~train, 0])),
                  input_condition=float(np.linalg.cond(x[train])),
                  eligible_for_model=bool(identifiable and validation_rmse < .04 and np.linalg.cond(x[train]) < 100))
    return result, np.column_stack((p[:, 0], predicted + p[0, 1:])).tolist()


def stopping_samples(commands, observations):
    """Empirical zero-to-rest displacement; do not mistake it for pure mechanical deceleration."""
    u, p = np.asarray(commands, dtype=float), np.asarray(observations, dtype=float)
    if len(u) < 2 or len(p) < 8:
        return []
    samples = []
    for i in range(1, len(u)):
        if np.linalg.norm(u[i, 1:]) > 1e-8 or np.linalg.norm(u[i-1, 1:]) < .04:
            continue
        t = u[i, 0]
        later = next((r[0] for r in u[i+1:] if np.linalg.norm(r[1:]) > .01), u[-1, 0])
        before = p[(p[:, 0] >= t-.3) & (p[:, 0] <= t)]
        after = p[(p[:, 0] >= t) & (p[:, 0] <= min(t+2, later))]
        if len(before) < 3 or len(after) < 5 or before[-1, 0]-before[0, 0] < .1:
            continue
        velocity = np.linalg.lstsq(np.column_stack((before[:, 0]-t, np.ones(len(before)))), before[:, 1:], rcond=None)[0][0]
        speed = float(np.linalg.norm(velocity))
        if speed < .06:
            continue
        stop = None
        for j in range(3, len(after)):
            tail = after[(after[:, 0] >= after[j, 0]-.3) & (after[:, 0] <= after[j, 0])]
            if len(tail) >= 3 and tail[-1, 0]-tail[0, 0] >= .2 and np.linalg.norm(np.ptp(tail[:, 1:], axis=0)) <= .015:
                stop = j
                break
        if stop is None:
            continue
        origin = np.array([np.interp(t, p[:, 0], p[:, k]) for k in (1, 2)])
        distance = float(max(np.linalg.norm(after[:stop+1, 1:]-origin, axis=1)))
        samples.append(dict(zero_time=float(t), observed_speed_before_mps=speed,
                            zero_to_confirmed_rest_s=float(after[stop, 0]-t),
                            max_displacement_after_zero_m=distance,
                            effective_deceleration_including_delay_mps2=speed**2/(2*distance) if distance > .005 else None))
    return samples
