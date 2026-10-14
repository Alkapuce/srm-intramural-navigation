#!/usr/bin/env python3
"""Observe actual sensor terrain and costmap after obstacle placement and removal."""

import argparse, json, subprocess, time, math
from pathlib import Path
import numpy as np
import rclpy
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from tf2_ros import Buffer, TransformListener

p = argparse.ArgumentParser()
p.add_argument("--output", required=True)
args = p.parse_args()
out = Path(args.output)
c = json.loads(Path("/work/config/arena.json").read_text())
ns = "/" + c["namespace"]
target = [1.5, 1.5]
rclpy.init(
    args=[
        "--ros-args",
        "-r",
        "/tf:=" + ns + "/tf",
        "-r",
        "/tf_static:=" + ns + "/tf_static",
    ]
)
node = rclpy.create_node("arena_obstacle_verifier")
buffer = Buffer(node=node)
listener = TransformListener(buffer, node)
observed = 0
cost = None
last = 0


def points(msg):
    global observed, last
    fields = {f.name: f for f in msg.fields}
    if not all(k in fields for k in ("x", "y", "intensity")):
        return
    dtype = np.dtype(
        {
            "names": ["x", "y", "intensity"],
            "formats": ["<f4"] * 3,
            "offsets": [fields[k].offset for k in ("x", "y", "intensity")],
            "itemsize": msg.point_step,
        }
    )
    data = np.ndarray(
        (msg.height, msg.width),
        dtype=dtype,
        buffer=msg.data,
        strides=(msg.row_step, msg.point_step),
    )
    x = data["x"].reshape(-1)
    y = data["y"].reshape(-1)
    try:
        t = buffer.lookup_transform("map", msg.header.frame_id, rclpy.time.Time())
        q = t.transform.rotation
        angle = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        mx = x * math.cos(angle) - y * math.sin(angle) + t.transform.translation.x
        my = x * math.sin(angle) + y * math.cos(angle) + t.transform.translation.y
    except Exception:
        return
    observed = int(
        np.count_nonzero(
            (abs(mx - target[0]) < 0.4)
            & (abs(my - target[1]) < 0.4)
            & (data["intensity"].reshape(-1) > 0.15)
        )
    )
    last = time.monotonic()


def grid(msg):
    global cost
    try:
        t = buffer.lookup_transform(msg.header.frame_id, "map", rclpy.time.Time())
        q = t.transform.rotation
        angle = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        x = (
            target[0] * math.cos(angle)
            - target[1] * math.sin(angle)
            + t.transform.translation.x
        )
        y = (
            target[0] * math.sin(angle)
            + target[1] * math.cos(angle)
            + t.transform.translation.y
        )
        ix = int((x - msg.info.origin.position.x) / msg.info.resolution)
        iy = int((y - msg.info.origin.position.y) / msg.info.resolution)
        if 0 <= ix < msg.info.width and 0 <= iy < msg.info.height:
            # Lidar marks visible surfaces, not the occluded center of a solid box.
            radius = int(math.ceil(0.4 / msg.info.resolution))
            costs = np.asarray(msg.data).reshape(msg.info.height, msg.info.width)
            cost = int(
                costs[
                    max(0, iy - radius) : min(msg.info.height, iy + radius + 1),
                    max(0, ix - radius) : min(msg.info.width, ix + radius + 1),
                ].max()
            )
    except Exception:
        pass


node.create_subscription(
    PointCloud2, ns + "/terrain_map", points, qos_profile_sensor_data
)
node.create_subscription(
    OccupancyGrid,
    ns + "/local_costmap/costmap",
    grid,
    QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
)


def wait(condition, label, timeout=20):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if condition() and time.monotonic() - last < 0.5:
            return
    raise RuntimeError(f"{label}: terrain points={observed}, cost={cost}")


try:
    subprocess.run(["python3", "/work/scripts/internal/arena-obstacle.py", "park"], check=True)
    wait(
        lambda: observed < 5 and cost is not None and cost < 90,
        "clear before placement",
    )
    subprocess.run(
        [
            "python3",
            "/work/scripts/internal/arena-obstacle.py",
            "place",
            "--x",
            str(target[0]),
            "--y",
            str(target[1]),
        ],
        check=True,
    )
    wait(lambda: observed >= 5 and cost is not None and cost >= 90, "obstacle observed")
    inserted = {"terrain_points": observed, "cost": cost}
    subprocess.run(["python3", "/work/scripts/internal/arena-obstacle.py", "park"], check=True)
    wait(lambda: observed < 5 and cost is not None and cost < 90, "obstacle cleared")
    (out / "obstacle-results.json").write_text(
        json.dumps(
            {
                "inserted": inserted,
                "removed": {"terrain_points": observed, "cost": cost},
                "target": target,
            },
            indent=2,
        )
        + "\n"
    )
    print("PASS obstacle placement/removal observed in actual terrain and costmap")
finally:
    subprocess.run(["python3", "/work/scripts/internal/arena-obstacle.py", "park"], check=False)
    node.destroy_node()
    rclpy.shutdown()
