"""Look only at occupied cells near the current ordered path segment."""

import math
from functools import lru_cache
import numpy as np


@lru_cache(maxsize=32)
def footprint_cells(radius,resolution):
    n=math.ceil(radius/resolution)
    y,x=np.mgrid[-n:n+1,-n:n+1]
    keep=np.hypot(x,y)*resolution<=radius
    return x[keep],y[keep]


def first_blocked(points, progress, position, grid, width, height, resolution,
                  origin, map_to_grid=(0.0, 0.0, 0.0), lookahead=1.2, radius=0.45, threshold=90):
    if resolution <= 0 or width * height != len(grid):
        raise ValueError("Invalid occupancy grid")
    tx, ty, yaw = map_to_grid
    cy, sy = math.cos(yaw), math.sin(yaw)
    # Nav2 rolling costmaps normally have zero grid-origin rotation; support it explicitly.
    ox, oy, oyaw = origin
    co, so = math.cos(oyaw), math.sin(oyaw)
    p=np.asarray(points[progress:],dtype=float).reshape(-1,2)
    if not len(p):return None
    reach=np.cumsum(np.linalg.norm(np.diff(np.vstack((position,p)),axis=0),axis=1))
    p=p[reach<=lookahead]
    if not len(p):return None
    gx=cy*p[:,0]-sy*p[:,1]+tx-ox
    gy=sy*p[:,0]+cy*p[:,1]+ty-oy
    ix=np.floor((co*gx+so*gy)/resolution).astype(int)
    iy=np.floor((-so*gx+co*gy)/resolution).astype(int)
    dx,dy=footprint_cells(radius,resolution)
    col,row=ix[:,None]+dx,iy[:,None]+dy
    inside=(row>=0)&(row<height)&(col>=0)&(col<width)
    hits=np.zeros(inside.shape,bool)
    hits[inside]=np.asarray(grid)[row[inside]*width+col[inside]]>=threshold
    indices=np.flatnonzero(np.any(hits,axis=1))
    return progress+int(indices[0]) if len(indices) else None
