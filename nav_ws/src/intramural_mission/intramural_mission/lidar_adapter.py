"""Adapt instantaneous Gazebo GPU scans to Point-LIO's Velodyne schema."""

import copy
import numpy as np
from array import array
import rclpy
from intramural_base.runtime import run
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2, PointField

# Point-LIO uses time both to detect supplied timestamps and to group point
# updates. Tiny per-column offsets preserve deterministic update ordering while
# approximating an instantaneous scan: 100 ns at 0.35 m/s is only 35 nm.
# These are numerical simulation offsets, never real driver timing.
MAX_INSTANT_OFFSET_SECONDS = 1e-7
OUTPUT_DTYPE = np.dtype(
    [
        ("x", "<f4"),
        ("y", "<f4"),
        ("z", "<f4"),
        ("intensity", "<f4"),
        ("ring", "<u2"),
        ("pad", "<u2"),
        ("time", "<f4"),
    ]
)


def convert(msg):
    if msg.header.frame_id != "front_mid360" or msg.is_bigendian:
        raise ValueError("Expected little-endian simulated front_mid360 cloud")
    fields = {f.name: f for f in msg.fields}
    expected = {"x": 7, "y": 7, "z": 7, "intensity": 7, "ring": 4}
    if any(
        k not in fields or fields[k].datatype != v or fields[k].count != 1
        for k, v in expected.items()
    ):
        raise ValueError("Unexpected Gazebo point fields")
    dtype = np.dtype(
        {
            "names": list(expected),
            "formats": ["<f4"] * 4 + ["<u2"],
            "offsets": [fields[k].offset for k in expected],
            "itemsize": msg.point_step,
        }
    )
    data = np.ndarray(
        (msg.height, msg.width),
        dtype=dtype,
        buffer=msg.data,
        strides=(msg.row_step, msg.point_step),
    ).reshape(-1)
    valid = (
        np.isfinite(data["x"])
        & np.isfinite(data["y"])
        & np.isfinite(data["z"])
        & np.isfinite(data["intensity"])
        & (data["ring"] < 32)
    )
    filtered = data[valid]
    packed = np.zeros(len(filtered), dtype=OUTPUT_DTYPE)
    for k in expected:
        packed[k] = filtered[k]
    offsets = np.tile(np.arange(1, msg.width + 1, dtype=np.float64), msg.height) * (
        MAX_INSTANT_OFFSET_SECONDS / msg.width
    )
    packed["time"] = offsets[valid]
    result = PointCloud2()
    result.header = copy.deepcopy(msg.header)
    result.height = 1
    result.width = len(packed)
    result.fields = [
        PointField(name=k, offset=OUTPUT_DTYPE.fields[k][1], datatype=typ, count=1)
        for k, typ in {**expected, "time": 7}.items()
    ]
    result.point_step = OUTPUT_DTYPE.itemsize
    result.row_step = result.width * result.point_step
    result.is_dense = True
    result.data = array("B", packed.tobytes())
    return result


class LidarAdapter(Node):
    def __init__(self):
        super().__init__("lidar_adapter")
        self.gravity_since = None
        self.gravity_ready = False
        self.pub = self.create_publisher(
            PointCloud2, "velodyne_points", qos_profile_sensor_data
        )
        self.create_subscription(
            PointCloud2, "livox/lidar", self.receive, qos_profile_sensor_data
        )
        self.create_subscription(Imu, "livox/imu", self.receive_imu, qos_profile_sensor_data)
        self.get_logger().info(
            "Instantaneous simulated scan approximation; point ordering offsets <= 100 ns, units seconds"
        )

    def receive(self, msg):
        if not self.gravity_ready:
            return
        cloud = convert(msg)
        if cloud.width:
            self.pub.publish(cloud)

    def receive_imu(self, msg):
        if self.gravity_ready:
            return
        a = msg.linear_acceleration
        g = msg.angular_velocity
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        # Spawn settling includes free fall; Point-LIO needs stationary gravity at initialization.
        if not (7.8 < np.linalg.norm([a.x, a.y, a.z]) < 11.8
                and np.linalg.norm([g.x, g.y, g.z]) < .1):
            self.gravity_since = None
            return
        if self.gravity_since is None:
            self.gravity_since = stamp
        self.gravity_ready = stamp - self.gravity_since >= .5


def main():
    run(LidarAdapter)
