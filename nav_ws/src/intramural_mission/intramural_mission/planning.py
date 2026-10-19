"""Compile the measured map and route together; unknown cells never become free."""

import hashlib
import math
import time
import numpy as np
from scipy.ndimage import distance_transform_edt
from scipy.interpolate import CubicSpline
from intramural_mission.route import segments, smooth_route, line, primitives
from intramural_mission.bypass import entry_path, winding
from intramural_mission.trajectory import RoutePlan


class GridMap:
    def __init__(self, grid, c):
        if grid.header.frame_id != "map":
            raise ValueError("Static grid must be in map frame")
        self.grid, self.c = grid, c
        self.res = grid.info.resolution
        if self.res <= 0 or grid.info.width * grid.info.height != len(grid.data):
            raise ValueError("Invalid map dimensions")
        self.free = (
            np.asarray(grid.data).reshape(grid.info.height, grid.info.width) == 0
        )
        self.distance = (
            distance_transform_edt(np.pad(self.free, 1))[1:-1, 1:-1] * self.res
        )
        o = grid.info.origin
        self.origin = np.array([o.position.x, o.position.y])
        q = o.orientation
        self.yaw = math.atan2(
            2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)
        )
        self.co, self.si = math.cos(self.yaw), math.sin(self.yaw)
        self.map_id = hashlib.sha256(
            self.free.tobytes()
            + repr((self.res, self.origin.tolist(), self.yaw)).encode()
        ).hexdigest()

    def clearance(self, points):
        p = np.asarray(points) - self.origin
        x = (self.co * p[:, 0] + self.si * p[:, 1]) / self.res
        y = (-self.si * p[:, 0] + self.co * p[:, 1]) / self.res
        col, row = np.floor(x).astype(int), np.floor(y).astype(int)
        inside = (
            (row >= 0)
            & (row < self.free.shape[0])
            & (col >= 0)
            & (col < self.free.shape[1])
        )
        distance = np.zeros(len(p))
        distance[inside] = self.distance[row[inside], col[inside]]
        distance = np.minimum(
            distance,
            np.minimum.reduce((x, y, self.free.shape[1] - x, self.free.shape[0] - y))
            * self.res,
        )
        margin = (
            self.c["robot_radius"]
            + self.res / math.sqrt(2)
            + self.c.get("minimum_clearance", 0.02)
        )
        result = distance - margin
        if self.c.get("mission_variant", "figure8") == "figure8":
            world = np.asarray(points)
            for key in ("lower_pole", "upper_pole"):
                result = np.minimum(result, np.linalg.norm(world-self.c[key], axis=1)
                                    - self.c["robot_radius"]-self.c["pole_radius"]
                                    - self.c.get("minimum_clearance", .02))
        return result

    def shortcut(self, path):
        result = [path[0]]
        i = 0
        while i < len(path) - 1:
            j = len(path) - 1
            while (
                j > i + 1
                and min(self.clearance(line(path[i], path[j], self.res / 2))) <= 0
            ):
                j -= 1
            result.extend(line(path[i], path[j], self.c["path_spacing"])[1:])
            i = j
        return result

    def connectors(self, start, gate, tangent):
        """Try both sides of a tangent approach, and accept only observed, bounded-curvature curves."""
        t = np.asarray(tangent)
        normal = np.array([-t[1], t[0]])
        distance = min(self.c["loop_radius"], 0.8)
        gate = np.asarray(gate)
        waypoint_sets = [[gate.tolist()]] + [
            [
                (gate + side * distance * normal).tolist(),
                (gate + side * distance * normal - distance * t).tolist(),
                (gate - distance * t).tolist(),
                gate.tolist(),
            ]
            for side in (-1.0, 1.0)
        ]
        for waypoints in waypoint_sets:
            points = [start]
            for waypoint in waypoints:
                # A* and the continuous checker must agree on the complete pole bases.
                leg = entry_path(self.c, points[-1], waypoint, self.grid, self.clearance)
                if leg is None:
                    break
                points.extend(self.shortcut(leg)[1:])
            if leg is None:
                continue
            vertices = [points[0]]
            for a, b, c in zip(points, points[1:], points[2:]):
                u = np.asarray(b) - a
                v = np.asarray(c) - b
                if np.dot(u, v) < 0.999 * np.linalg.norm(u) * np.linalg.norm(v):
                    vertices.append(b)
            vertices.append(points[-1])
            p = np.asarray(vertices)
            ds = np.linalg.norm(np.diff(p, axis=0), axis=1)
            if min(ds) < 1e-6:
                continue
            s = np.r_[0.0, np.cumsum(ds)]
            curve = CubicSpline(s, p, bc_type=((2, [0.0, 0.0]), (1, t)))
            sample = np.linspace(
                0, s[-1], max(8, math.ceil(s[-1] / (self.c["path_spacing"] / 2)))
            )
            d, dd = curve(sample, 1), curve(sample, 2)
            k = (
                np.abs(d[:, 0] * dd[:, 1] - d[:, 1] * dd[:, 0])
                / np.linalg.norm(d, axis=1).clip(1e-9) ** 3
            )
            path = curve(sample)
            if (self.c.get("optimize_route") or max(k) <= 2 / self.c["robot_radius"]) and min(self.clearance(path)) > 0:
                yield path.tolist()


def _compile(c, measured, start=None, route=None, geometry=None, scales=None):
    route = route or (
        primitives(c)
        if c.get("mission_variant") == "primitives"
        else segments(c, smooth=False)
    )
    if c.get("mission_variant") == "return":
        route = [("RETURN", c["return_path"])]
    routes = [route]
    if start is not None or (geometry is not None and min(measured.clearance(route[0][1])) <= 0):
        start = c["parking"] if start is None else start
        gate = route[0][1][-1]
        a, b = route[1][1][:2]
        t = np.asarray(b) - a
        t /= np.linalg.norm(t)
        routes = (
            [[("ENTER", [start, gate])] + route[1:]]
            if math.dist(start, gate) < c["path_spacing"]
            else [
                [("ENTER", path)] + route[1:]
                for path in measured.connectors(start, gate, t)
            ]
        )
        if not routes:
            raise ValueError("No observed smooth tangent approach from current pose")
        if geometry is not None and math.dist(start,c["parking"]) < 1e-6:
            routes = [raw[:-1]+[("RETURN",list(reversed(raw[0][1])))] for raw in routes]
    candidates = []
    for raw in routes:
        for scale in (scales or (
            (1.25, 0.9, 0.6, 0.35)
            if c.get("smooth_transitions")
            and c.get("mission_variant", "figure8") == "figure8"
            else (None,)
        )):
            candidate = smooth_route(raw, c, scale, geometry) if scale is not None else raw
            try:
                plan = RoutePlan.build(
                    candidate,
                    c,
                    lambda p: measured.clearance(p) - c["path_spacing"] / 2,
                    measured.map_id,
                )
            except ValueError:
                continue
            # Omni steering has no radius-derived curvature limit; speed and predicted tracking decide feasibility.
            if geometry is None and max(map(abs, plan.curvature)) > 2 / c["robot_radius"]:
                continue
            # Validate both topology and footprint after the actual map-aware smoothing.
            if c.get("mission_variant", "figure8") == "figure8":
                loop = [p for _, points in candidate[1:-1] for p in points]
                if [winding(loop, c[k]) for k in ("lower_pole", "upper_pole")] != [
                    1,
                    -1,
                ]:
                    continue
            duration = sum(
                2*(b - a) / max(1e-9, x + y)
                for a, b, x, y in zip(plan.s, plan.s[1:], plan.speed, plan.speed[1:])
            )
            candidates.append((duration, plan, candidate, scale))
    if not candidates:
        raise ValueError("No smooth ordered route fits the observed free space")
    duration, plan, route, scale = min(candidates, key=lambda v: v[0])
    return (
        plan,
        route,
        dict(
            map_id=measured.map_id,
            transition_scale=scale,
            minimum_clearance=min(plan.clearance),
            curvature_peak=max(map(abs, plan.curvature)),
            expected_duration=duration,
        ),
    )


def compile_route(c, grid, start=None):
    measured = GridMap(grid, c)
    if not c.get("optimize_route") or c.get("mission_variant", "figure8") != "figure8":
        return _compile(c,measured,start)
    from intramural_mission.route_optimization import spline_route, shapes, refine
    started = time.monotonic()
    results, evaluated = [], 0
    def evaluate(shape, angle, scale=None):
        nonlocal evaluated
        evaluated += 1
        config = dict(c,entry_angle=angle)
        raw,geometry = (None,None) if shape is None else spline_route(config,shape,angle)
        scales = ((1.25,.9,.7,.6,.35) if start is not None else
                  (scale,) if scale is not None else None if shape is None else (1.25,.7))
        try:
            plan,route,report = _compile(config,measured,start,raw,geometry,scales)
        except ValueError:
            return
        report["optimized_route"] = dict(shape=shape,entry_angle=angle,transition_scale=report["transition_scale"])
        results.append((plan,route,report))
    fixed = c.get("optimized_route")
    if fixed:
        evaluate(fixed["shape"],fixed["entry_angle"],fixed["transition_scale"])
    else:
        for angle in (0.,math.pi):
            evaluate(None,angle)
        baseline = min((x[2]["expected_duration"] for x in results),default=None)
        for shape in shapes(c):
            for angle in (0.,math.pi):
                evaluate(shape,angle)
        if results:
            best = min(results,key=lambda x:x[2]["expected_duration"])[2]["optimized_route"]
            if best["shape"] is not None:
                for shape in refine(best["shape"],c):
                    evaluate(shape,best["entry_angle"])
    if not results:
        raise ValueError("No ordered route fits the map and complete pole/vehicle envelopes")
    if not fixed:
        from intramural_mission.response import predict_execution
        reserve = c.get('execution_clearance', 0.)
        def fits(prediction):
            return prediction['passed'] and prediction['minimum_clearance'] >= reserve
        for plan,_,report in results:
            report["execution_prediction"] = predict_execution(plan,c,measured)
        feasible = [x for x in results if fits(x[2]["execution_prediction"])]
        if not feasible:
            raise ValueError("No geometrically valid route can be tracked with the configured response model")
        for plan,route,report in sorted(feasible,key=lambda x:x[2]['execution_prediction']['duration']):
            predictions=[]
            for delay,tau in c.get('response_models',[]):
                prediction=predict_execution(plan,c,measured,plant_response=(delay,tau))
                predictions.append(dict(delay=delay,tau=tau,**prediction))
                if not fits(prediction):break
            if all(fits(p) for p in predictions):
                report['response_predictions']=predictions
                break
        else:
            raise ValueError('No route fits the configured response models and complete vehicle envelope')
    else:
        plan,route,report = results[0]
    report.update(optimization="bounded joint geometry/speed search",candidates_evaluated=evaluated,
                  planning_seconds=time.monotonic()-started)
    if not fixed:
        report["fixed_circle_duration"]=baseline
    return plan,route,report
