"""Geometry shared by the mission and independent simulation configuration."""

import math


def line(a, b, spacing):
    n = max(1, math.ceil(math.dist(a, b) / spacing))
    return [
        [a[0] + (b[0] - a[0]) * i / n, a[1] + (b[1] - a[1]) * i / n]
        for i in range(n + 1)
    ]


def arc(center, r, start, end, spacing):
    n = max(1, math.ceil(abs(end - start) * r / spacing))
    return [
        [
            center[0] + r * math.cos(start + (end - start) * i / n),
            center[1] + r * math.sin(start + (end - start) * i / n),
        ]
        for i in range(n + 1)
    ]


def segments(c):
    r = c["loop_radius"]
    lo = c["lower_pole"]
    hi = c["upper_pole"]
    s = c["path_spacing"]
    if abs(hi[0] - lo[0]) > 1e-9 or abs(hi[1] - lo[1] - 2 * r) > 1e-9:
        raise ValueError(
            "Tangent-circle route requires aligned poles separated by 2*loop_radius"
        )
    entry = [lo[0] - r, lo[1]]
    result = [("ENTER", line(c["parking"], entry, s))]
    for i in range(3):
        result.append(
            (
                "LOWER",
                arc(
                    lo, r, math.pi + i * math.pi / 2, math.pi + (i + 1) * math.pi / 2, s
                ),
            )
        )
    for i in range(4):
        result.append(
            (
                "UPPER",
                arc(
                    hi,
                    r,
                    -math.pi / 2 - i * math.pi / 2,
                    -math.pi / 2 - (i + 1) * math.pi / 2,
                    s,
                ),
            )
        )
    result.append(("LOWER_EXIT", arc(lo, r, math.pi / 2, math.pi, s)))
    result.append(("RETURN", line(entry, c["parking"], s)))
    return result


def clearance(c, p):
    radius = c.get("robot_radius", math.hypot(c["robot_length"], c["robot_width"]) / 2)
    xmin, xmax, ymin, ymax = c["bounds"]
    pole = min(
        math.dist(p, c[k]) - c["pole_radius"] - radius
        for k in ("lower_pole", "upper_pole")
    )
    boundary = min(p[0] - xmin, xmax - p[0], p[1] - ymin, ymax - p[1]) - radius
    return pole, boundary


def primitives(c):
    s = c["path_spacing"]
    x, y = c["parking"]
    return [
        ("FORWARD", line([x, y], [x + 0.8, y], s)),
        ("SIDEWAYS", line([x + 0.8, y], [x + 0.8, y + 0.8], s)),
        ("CURVE", arc([x, y + 0.8], 0.8, 0, math.pi / 2, s)),
        ("RETURN", line([x, y + 1.6], [x, y], s)),
    ]


def projected_progress(points, index, pose):
    """Continuous position near the current sample, without searching later arcs."""
    candidates = []
    for k in range(max(0, index - 1), min(len(points) - 1, index + 1)):
        a, b = points[k : k + 2]
        dx, dy = b[0] - a[0], b[1] - a[1]
        length2 = dx * dx + dy * dy
        fraction = (
            min(
                1.0, max(0.0, ((pose[0] - a[0]) * dx + (pose[1] - a[1]) * dy) / length2)
            )
            if length2
            else 0.0
        )
        projected = [a[0] + fraction * dx, a[1] + fraction * dy]
        candidates.append((math.dist(pose, projected), k + fraction))
    return min(candidates)[1]
