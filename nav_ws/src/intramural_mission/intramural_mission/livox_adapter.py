"""Preserve real Livox coordinates and timestamps for raw collision protection."""

from array import array
import copy
import numpy as np
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from livox_ros_driver2.msg import CustomMsg
from intramural_base.runtime import run


def convert(msg):
    if msg.header.frame_id != "livox_frame" or msg.point_num != len(msg.points) or not msg.point_num:
        raise ValueError("Expected a nonempty real Livox frame with matching point count")
    data = np.array([(p.x, p.y, p.z, p.reflectivity) for p in msg.points], dtype="<f4")
    if not np.isfinite(data).all():
        raise ValueError("Nonfinite real Livox points")
    out = PointCloud2()
    out.header = copy.deepcopy(msg.header)
    out.height, out.width = 1, len(data)
    out.fields = [PointField(name=k, offset=i*4, datatype=7, count=1)
                  for i, k in enumerate(("x", "y", "z", "intensity"))]
    out.point_step, out.row_step, out.is_dense = 16, len(data)*16, True
    out.data = array("B", data.tobytes())
    return out


class LivoxAdapter(Node):
    def __init__(self):
        super().__init__("livox_raw_adapter")
        self.pub = self.create_publisher(PointCloud2, "livox/lidar", qos_profile_sensor_data)
        self.create_subscription(CustomMsg, "/livox/lidar", self.receive, qos_profile_sensor_data)

    def receive(self, msg):
        try:
            self.pub.publish(convert(msg))
        except ValueError as error:
            self.get_logger().error(str(error))  # No replacement data: freshness guard will stop.


def main():
    run(LivoxAdapter)
