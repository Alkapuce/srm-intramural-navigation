"""Guard input contracts on an isolated ROS graph, without any serial or simulator."""

from array import array
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace as N
import unittest
import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from intramural_mission.adapter import Adapter
from intramural_mission.trajectory_mission import TrajectoryMission
from intramural_mission.trajectory import RoutePlan, OmniTracker
from intramural_mission.route import segments

ROOT = Path(__file__).resolve().parents[1]


class TrajectorySafetyTests(unittest.TestCase):
    def test_missing_localization_latches_fault_and_clears_motion(self):
        c = json.loads((ROOT / "config/arena.json").read_text())
        c.update(json.loads((ROOT / "config/navigation-profiles.json").read_text())["safe"])
        with tempfile.TemporaryDirectory(dir=ROOT / "cache") as folder:
            config = Path(folder) / "arena.json"
            config.write_text(json.dumps(c))
            rclpy.init(
                domain_id=79, args=["--ros-args", "-p", f"arena_config:={config}"]
            )
            node = TrajectoryMission()
            try:
                node.plan = RoutePlan.build(segments(c), c)
                node.tracker = OmniTracker(node.plan, c)
                node.tracker.command[:] = [0.4, 0.0]
                node.state = "RUNNING"
                node.tick()
                self.assertEqual(node.state, "FAULT")
                self.assertEqual(node.reason, "LOCALIZATION_OR_SENSOR_STALE")
                self.assertEqual(node.tracker.command.tolist(), [0.0, 0.0])
                # The fault cannot silently restart when a later tick receives data.
                node.tick()
                self.assertEqual(node.state, "FAULT")
            finally:
                node.destroy_node()
                rclpy.shutdown()

    def test_old_run_path_and_empty_sensor_heartbeat_cannot_enable_motion(self):
        c = json.loads((ROOT / "config/arena.json").read_text())
        c.update(json.loads((ROOT / "config/navigation-profiles.json").read_text())["safe"])
        c.update(navigation_engine="trajectory", raw_collision_follow_path=True)
        with tempfile.TemporaryDirectory(dir=ROOT / "cache") as folder:
            config = Path(folder) / "arena.json"
            config.write_text(json.dumps(c))
            rclpy.init(
                domain_id=78,
                args=[
                    "--ros-args",
                    "-p",
                    f"arena_config:={config}",
                    "-p",
                    "localization:=sensors",
                ],
            )
            a = Adapter()
            try:

                def transform(*args):
                    t = TransformStamped()
                    t.header.stamp = a.get_clock().now().to_msg()
                    t.transform.rotation.w = 1.0
                    return t

                a.path_tf = N(lookup_transform=transform)
                a.receive_mission_status(
                    String(
                        data=json.dumps(
                            dict(
                                phase="ENTER",
                                state="RUNNING",
                                run_id="new",
                                route_id="route",
                            )
                        )
                    )
                )
                path = dict(
                    run_id="old",
                    route_id="route",
                    stamp=a.get_clock().now().nanoseconds / 1e9,
                    points=[[0.0, 0.0], [1.0, 0.0]],
                )
                a.receive_trajectory(String(data=json.dumps(path)))
                self.assertIsNone(a.body_path())
                path["run_id"] = "new"
                a.receive_trajectory(String(data=json.dumps(path)))
                self.assertEqual(len(a.body_path()), 2)
                path["stamp"] -= 1.0
                a.receive_trajectory(String(data=json.dumps(path)))
                self.assertIsNone(a.body_path())
                cloud = PointCloud2()
                cloud.header.stamp = a.get_clock().now().to_msg()
                cloud.height = 1
                cloud.width = 1
                cloud.point_step = 12
                cloud.row_step = 12
                cloud.fields = [
                    PointField(name=name, offset=4 * i, datatype=7, count=1)
                    for i, name in enumerate("xyz")
                ]
                cloud.data = array("B", np.zeros((1, 3), dtype="<f4").tobytes())
                a.lidar(cloud)
                self.assertNotIn("lidar", a.last)
                cloud.data = array(
                    "B", np.array([[1.0, 0.0, 0.5]], dtype="<f4").tobytes()
                )
                a.lidar(cloud)
                self.assertIn("lidar", a.last)
                a.enabled = True
                a.tick()
                self.assertEqual(a.output, [0.0, 0.0])
            finally:
                a.destroy_node()
                rclpy.shutdown()


if __name__ == "__main__":
    unittest.main()
