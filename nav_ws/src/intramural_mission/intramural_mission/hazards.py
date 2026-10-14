"""Look only at occupied cells near the current ordered path segment."""

import math


def first_blocked(points, progress, position, grid, width, height, resolution,
                  origin, map_to_grid=(0.0, 0.0, 0.0), lookahead=1.2, radius=0.45, threshold=90):
    if resolution <= 0 or width * height != len(grid):
        raise ValueError("Invalid occupancy grid")
    tx, ty, yaw = map_to_grid
    cy, sy = math.cos(yaw), math.sin(yaw)
    # Nav2 rolling costmaps normally have zero grid-origin rotation; support it explicitly.
    ox, oy, oyaw = origin
    co, so = math.cos(oyaw), math.sin(oyaw)
    reach = 0.0
    previous = position
    cell_radius = math.ceil(radius / resolution)
    for index in range(progress, len(points)):
        x, y = points[index]
        reach += math.dist(previous, (x, y))
        if reach > lookahead:
            break
        previous = (x, y)
        gx = cy * x - sy * y + tx - ox
        gy = sy * x + cy * y + ty - oy
        ix = math.floor((co * gx + so * gy) / resolution)
        iy = math.floor((-so * gx + co * gy) / resolution)
        for dy in range(-cell_radius, cell_radius + 1):
            row = iy + dy
            if not 0 <= row < height:
                continue
            for dx in range(-cell_radius, cell_radius + 1):
                col = ix + dx
                if not 0 <= col < width or math.hypot(dx, dy) * resolution > radius:
                    continue
                if grid[row * width + col] >= threshold:
                    return index
    return None
