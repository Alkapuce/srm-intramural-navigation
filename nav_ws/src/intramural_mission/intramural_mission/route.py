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


def segments(c, smooth=True):
    r = c["loop_radius"]
    lo = c["lower_pole"]
    hi = c["upper_pole"]
    s = c["path_spacing"]
    if r <= 0 or abs(math.dist(hi, lo) - 2 * r) > 1e-9:
        raise ValueError(
            "Tangent-circle route requires poles separated by 2*loop_radius"
        )
    rotation = math.atan2(hi[1] - lo[1], hi[0] - lo[0]) - math.pi / 2
    start = c.get("entry_angle", math.pi)
    if start not in (0.0, math.pi):
        raise ValueError("Entry angle must be 0 or pi")
    def circle(center, a, b):
        return arc(center, r, a + rotation, b + rotation, s)
    entry = circle(lo, start, start)[0]
    crossing = math.pi / 2 + (2 * math.pi if start > math.pi / 2 else 0)
    result = [("ENTER", line(c["parking"], entry, s))]
    for i in range(3):
        result.append(
            (
                "LOWER",
                circle(lo, start + i * (crossing-start)/3, start + (i+1) * (crossing-start)/3),
            )
        )
    for i in range(4):
        result.append(
            (
                "UPPER",
                circle(
                    hi,
                    -math.pi / 2 - i * math.pi / 2,
                    -math.pi / 2 - (i + 1) * math.pi / 2,
                ),
            )
        )
    result.append(("LOWER_EXIT", circle(lo, math.pi / 2, start + (2 * math.pi if start < math.pi / 2 else 0))))
    result.append(("RETURN", line(entry, c["parking"], s)))
    return smooth_route(result,c) if smooth else result


def quintic_curve(a,b,u,v,ka,kb,spacing):
    length=math.dist(a,b)
    control=[a,[a[k]+length*u[k]/5 for k in range(2)],
        [a[k]+2*length*u[k]/5+length*length*ka*[-u[1],u[0]][k]/20 for k in range(2)],
        [b[k]-2*length*v[k]/5+length*length*kb*[-v[1],v[0]][k]/20 for k in range(2)],
        [b[k]-length*v[k]/5 for k in range(2)],b]
    n=max(12,math.ceil(5*max(math.dist(p,q) for p,q in zip(control,control[1:]))/spacing))
    return [[sum(math.comb(5,h)*(1-t)**(5-h)*t**h*control[h][k] for h in range(6))
        for k in range(2)] for t in (i/n for i in range(n+1))]


def smooth_route(route,c,scale=1.25,geometry=None):
    """Spread each entry/exit turn with curvature-continuous quintic transitions."""
    if not c.get('smooth_transitions',False):
        return route
    result=[(name,list(points)) for name,points in route]
    for index,count in ((0,3),(len(route)-2,1)):
        name,left=result[index]
        right=[p for offset,(_,points) in enumerate(result[index+1:index+1+count])
               for p in points[1 if offset else 0:]]
        if len(left)<3 or len(right)<3 or math.dist(left[-1],right[0])>1e-6:
            continue
        trim=min(scale*c['loop_radius'],
            .75*sum(math.dist(a,b)for a,b in zip(left,left[1:])),
            .75*sum(math.dist(a,b)for a,b in zip(right,right[1:])))
        i,j,d=len(left)-1,0,0.
        while i>1 and d<trim:
            d+=math.dist(left[i],left[i-1]);i-=1
        d=0.
        while j<len(right)-2 and d<trim:
            d+=math.dist(right[j],right[j+1]);j+=1
        a,b=left[i],right[j]
        incoming=[x-y for x,y in zip(a,left[i-1])]
        outgoing=[y-x for x,y in zip(b,right[j+1])]
        if geometry is not None:
            curve_point=b if index==0 else a
            direction,curvature=geometry(curve_point)
            if index==0:outgoing=direction
            else:incoming=direction
        elif index==0:
            outgoing=[-(b[1]-c['lower_pole'][1]),b[0]-c['lower_pole'][0]]
        else:
            incoming=[-(a[1]-c['lower_pole'][1]),a[0]-c['lower_pole'][0]]
        na,nb=math.hypot(*incoming),math.hypot(*outgoing)
        if min(na,nb)<1e-6:
            continue
        u=[v/na for v in incoming];v=[w/nb for w in outgoing]
        length=math.dist(a,b)
        ka,kb=(0.,1/c['loop_radius']) if index==0 else (1/c['loop_radius'],0.)
        if geometry is not None:
            ka,kb=(0.,curvature) if index==0 else (curvature,0.)
        curve=quintic_curve(a,b,u,v,ka,kb,c['path_spacing'])
        n=len(curve)-1
        result[index]=(name,left[:i]+curve[:n//2+1])
        following=curve[n//2:]+right[j+1:]
        for offset in range(count):
            lo=offset*(len(following)-1)//count
            hi=(offset+1)*(len(following)-1)//count
            result[index+1+offset]=(route[index+1+offset][0],following[lo:hi+1])
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


def controller_segment(route, index, allow_corners=False):
    """Overlap goals before their stop; rate corners only with a speed envelope."""
    points = route[index][1]
    if index + 1 == len(route) or len(points) < 2:
        return points, False
    following = route[index + 1][1]
    if len(following) < 2 or math.dist(points[-1], following[0]) > 1e-6:
        return points, False
    incoming = [b-a for a,b in zip(points[-2],points[-1])]
    outgoing = [b-a for a,b in zip(following[0],following[1])]
    scale = math.hypot(*incoming)*math.hypot(*outgoing)
    if scale == 0 or sum(a*b for a,b in zip(incoming,outgoing)) < (-.95 if allow_corners else .95)*scale:
        return points, False
    extended, distance = list(points), 0.
    for a,b in zip(following,following[1:]):
        distance += math.dist(a,b)
        extended.append(b)
        if distance >= .35:
            break
    return extended, True


def curve_speed_limit(points, maximum, lateral_acceleration, braking_acceleration):
    """Backward braking envelope for the short ordered path in body coordinates."""
    points = [p for i,p in enumerate(points) if i == 0 or math.dist(points[i-1],p) > 1e-6]
    if len(points) < 3:
        return maximum
    # Ignore passed samples; this window is already restricted to ordered progress.
    nearest=0
    while nearest+1<len(points) and math.hypot(*points[nearest+1])<=math.hypot(*points[nearest])+1e-9:
        nearest+=1
    distance, limit = 0., maximum
    for i in range(max(1, nearest), len(points)-1):
        if i > nearest:
            distance += math.dist(points[i-1], points[i])
        a, b, c = points[i-1:i+2]
        ab, bc, ac = math.dist(a,b), math.dist(b,c), math.dist(a,c)
        if min(ab,bc,ac) < 1e-6:
            continue
        curvature = 2*abs((b[0]-a[0])*(c[1]-b[1])-(b[1]-a[1])*(c[0]-b[0]))/(ab*bc*ac)
        if curvature > 1e-6:
            # The pursuit direction starts changing one lookahead before a corner.
            available = max(0., distance-.18)
            limit = min(limit, math.sqrt(lateral_acceleration/curvature+2*braking_acceleration*available))
    return limit


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
