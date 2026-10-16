#!/usr/bin/env python3
"""Audit observed free space without treating unknown cells as traversable."""
import argparse
import json
from pathlib import Path

import numpy as np
from PIL import Image
from scipy.ndimage import distance_transform_edt, label
import yaml


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--map-config", required=True)
    p.add_argument("--output", required=True)
    args = p.parse_args()
    folder, output = Path(args.map_config).resolve(), Path(args.output).resolve()
    if output.exists():
        p.error("Choose a new report file")
    config = json.loads((folder / "arena.json").read_text())
    metadata = yaml.safe_load((folder / "map.yaml").read_text())
    grid = np.asarray(Image.open(folder / metadata["image"]))
    if grid.ndim != 2 or not set(np.unique(grid)) <= {0, 205, 254}:
        raise ValueError("Expected generated occupied/unknown/free grayscale map")
    resolution, radius = float(metadata["resolution"]), float(config["robot_radius"])
    if not (np.isfinite(resolution) and resolution > 0 and np.isfinite(radius) and radius > 0):
        raise ValueError("Invalid map resolution or vehicle radius")
    free = grid == 254
    clearance = distance_transform_edt(np.pad(free, 1))[1:-1, 1:-1] * resolution
    # Subtract half a cell diagonal rather than treating a cell centre as its boundary.
    safe = clearance > radius + resolution / np.sqrt(2)
    regions, count = label(safe)
    areas = np.bincount(regions.ravel())[1:] * resolution**2
    report = {
        "map_config": str(folder), "resolution_m": resolution, "robot_radius_m": radius,
        "free_area_m2": float(free.sum() * resolution**2),
        "robot_clear_area_m2": float(safe.sum() * resolution**2),
        "robot_clear_components": int(count),
        "largest_robot_clear_area_m2": float(areas.max()) if len(areas) else 0,
        "unknown_fraction": float(np.mean(grid == 205)),
        "navigation_grid_has_clearance": bool(safe.any()),
        "competition_route_verified": False,
        "radius_sensitivity": [{"radius_m": r, "clear_area_m2": float(np.count_nonzero(clearance > r) * resolution**2)}
                               for r in sorted({radius, .27, .25, .20, .15}, reverse=True)],
        "limits": "Unknown is blocked. Clearance alone does not verify a route or physical localization precision.",
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
