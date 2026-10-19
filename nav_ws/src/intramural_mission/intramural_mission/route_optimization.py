"""A bounded family of C2 figure eights; the compiler scores each with its speed profile."""

import math
import numpy as np
from scipy.interpolate import CubicSpline
from intramural_mission.route import line


def spline_route(c, shape, entry_angle):
    lower, upper = np.asarray(c["lower_pole"]), np.asarray(c["upper_pole"])
    distance = float(np.linalg.norm(upper-lower))
    direction = (upper-lower)/distance
    normal = np.array([direction[1], -direction[0]])
    width_lower, width_upper, fraction = shape
    h = fraction*distance
    anchors = np.array([[0, h], [-width_lower, 0], [0, -h], [width_lower, 0],
                        [0, h], [-width_upper, distance], [0, 2*distance-h],
                        [width_upper, distance], [0, h]])
    anchors = lower+anchors[:, :1]*normal+anchors[:, 1:]*direction
    knots = np.r_[0., np.cumsum(np.linalg.norm(np.diff(anchors, axis=0), axis=1))]
    curve = CubicSpline(knots, anchors, bc_type="periodic")
    period = knots[-1]
    dense = np.linspace(0, period, max(128, math.ceil(period/c["path_spacing"])*8))
    arc_length = np.r_[0., np.cumsum(np.linalg.norm(np.diff(curve(dense), axis=0), axis=1))]
    geometry = {}
    def sample(a, b):
        def length(t):
            return math.floor(t/period)*arc_length[-1]+np.interp(t % period, dense, arc_length)
        sa, sb = length(a), length(b)
        ss = np.linspace(sa, sb, max(2, math.ceil((sb-sa)/c["path_spacing"])+1))
        tt = np.interp(ss % arc_length[-1], arc_length, dense)
        points, derivative, second = curve(tt), curve(tt, 1), curve(tt, 2)
        curvature = (derivative[:, 0]*second[:, 1]-derivative[:, 1]*second[:, 0])/np.linalg.norm(derivative, axis=1)**3
        for p, d, k in zip(points, derivative, curvature):
            geometry[tuple(p)] = (d.tolist(), float(k))
        return points.tolist()
    entry = knots[1 if entry_angle == math.pi else 3]
    result = [("ENTER", line(c["parking"], curve(entry).tolist(), c["path_spacing"]))]
    for a,b in zip(np.linspace(entry,knots[4],4)[:-1], np.linspace(entry,knots[4],4)[1:]):
        result.append(("LOWER",sample(a,b)))
    for a,b in zip(np.linspace(knots[4],period,5)[:-1],np.linspace(knots[4],period,5)[1:]):
        result.append(("UPPER",sample(a,b)))
    result.append(("LOWER_EXIT",sample(period,period+entry)))
    result.append(("RETURN",list(reversed(result[0][1]))))
    return result, lambda point: geometry[tuple(point)]


def shapes(c):
    """Coarse joint search, followed by a small coordinate refinement around its winner."""
    r = math.dist(c["lower_pole"],c["upper_pole"])/2
    for width in (.85*r, r, 1.2*r):
        for fraction in (.45,.5,.55):
            yield (width,width,fraction)


def refine(shape, c):
    r = math.dist(c["lower_pole"],c["upper_pole"])/2
    for i,step in enumerate((.1*r,.1*r,.04)):
        for sign in (-1,1):
            candidate = list(shape)
            candidate[i] += sign*step
            if min(candidate[:2]) > c["robot_radius"]+c["pole_radius"] and .35 <= candidate[2] <= .65:
                yield tuple(candidate)
