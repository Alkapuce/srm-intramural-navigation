"""Explicit ideal-localization adapter and sole final velocity publisher."""

import copy, json, math, time
import numpy as np
from array import array
from pathlib import Path
import rclpy
from intramural_base.runtime import run
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry, Path as RosPath
from sensor_msgs.msg import PointCloud2, Imu
from std_msgs.msg import Bool, String
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster, Buffer, TransformListener, TransformException
from intramural_mission.proximity import obstacle_near, obstacle_on_path, lidar_to_body


class Adapter(Node):
    def __init__(self):
        super().__init__("motion_guard")
        self.declare_parameter("localization", "ideal")
        self.mode = self.get_parameter("localization").value
        self.declare_parameter("motion_permitted", True)
        self.motion_permitted = self.get_parameter("motion_permitted").value
        self.declare_parameter("arena_config", "/work/config/arena.json")
        self.c = json.loads(Path(self.get_parameter("arena_config").value).read_text())
        self.enabled = False
        self.bypassing = False
        self.cmd = Twist()
        self.last = {}
        self.output = [0.0, 0.0]
        self.last_tick = time.monotonic()
        self.raw_near_points = np.empty((0, 2), dtype=np.float32)
        self.last_motion_direction = (0.0, 0.0)
        self.hazard = False
        self.collision_path = None
        if self.c.get('raw_collision_follow_path', False):
            self.path_tf = Buffer(node=self)
            self.path_listener = TransformListener(self.path_tf, self)
            self.create_subscription(RosPath, 'mission/collision_path', self.receive_path,
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.hazard_pub = self.create_publisher(Bool, "motion_guard/hazard", 10)
        self.diagnostic = self.create_publisher(String, "motion_guard/status", 10)
        self.create_subscription(Twist, "cmd_vel_nav2_result", self.command, 10)
        self.create_subscription(
            Bool,
            "mission/enabled",
            self.enable,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.create_subscription(Bool, 'mission/bypass', self.bypass,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(
            Odometry, "odometry", lambda m: self.touch("odom"), qos_profile_sensor_data
        )
        self.last_truth = None
        self.create_subscription(
            PointCloud2, "livox/lidar", self.lidar, qos_profile_sensor_data
        )
        self.create_subscription(
            Imu, "livox/imu", lambda m: self.touch("imu"), qos_profile_sensor_data
        )
        if self.mode == "ideal":
            self.odom_pub = self.create_publisher(Odometry, "odometry", 10)
            self.lidar_odom_pub = self.create_publisher(Odometry, "lidar_odometry", 10)
            self.registered_pub = self.create_publisher(
                PointCloud2, "registered_scan", 5
            )
            self.tf = TransformBroadcaster(self)
            self.static = StaticTransformBroadcaster(self)
            t = TransformStamped()
            t.header.frame_id = "map"
            t.child_frame_id = "odom"
            t.transform.rotation.w = 1.0
            self.static.sendTransform(t)
            self.create_subscription(
                Odometry, "chassis_odometry_gt", self.truth, qos_profile_sensor_data
            )
        elif self.mode != "sensors":
            raise ValueError("localization must be ideal or sensors")
        self.create_timer(
            0.025, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME)
        )

    def touch(self, key):
        self.last[key] = time.monotonic()

    def receive_path(self, msg):
        self.collision_path = msg

    def body_path(self):
        if self.collision_path is None or len(self.collision_path.poses) < 2:
            return None
        try:
            t = self.path_tf.lookup_transform('base_footprint', self.collision_path.header.frame_id, rclpy.time.Time())
        except TransformException:
            return None
        stamp = t.header.stamp.sec + t.header.stamp.nanosec/1e9
        if abs(self.get_clock().now().nanoseconds/1e9-stamp) > .25:
            return None
        q = t.transform.rotation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        co, si = math.cos(yaw), math.sin(yaw)
        return [[co*p.pose.position.x-si*p.pose.position.y+t.transform.translation.x,
                 si*p.pose.position.x+co*p.pose.position.y+t.transform.translation.y]
                for p in self.collision_path.poses]

    def enable(self, msg):
        self.enabled = msg.data

    def bypass(self, msg):
        self.bypassing = msg.data

    def command(self, msg):
        self.cmd = msg
        self.touch("command")

    def truth(self, msg):
        self.last_truth = msg
        m = copy.deepcopy(msg)
        m.header.frame_id = "odom"
        m.child_frame_id = "base_footprint"
        m.pose.pose.position.z = 0.0
        self.odom_pub.publish(m)
        self.lidar_odom_pub.publish(m)
        t = TransformStamped()
        t.header = m.header
        t.child_frame_id = m.child_frame_id
        t.transform.translation.x = m.pose.pose.position.x
        t.transform.translation.y = m.pose.pose.position.y
        t.transform.rotation = m.pose.pose.orientation
        self.tf.sendTransform(t)

    def lidar(self, msg):
        self.touch("lidar")
        raw_fields = {f.name: f for f in msg.fields}
        if all(k in raw_fields and raw_fields[k].datatype == 7 for k in ("x", "y", "z")):
            raw_dtype = np.dtype({"names": ["x", "y", "z"], "formats": ["<f4"] * 3,
                "offsets": [raw_fields[k].offset for k in ("x", "y", "z")],
                "itemsize": msg.point_step})
            raw = np.ndarray((msg.height, msg.width), dtype=raw_dtype, buffer=msg.data,
                strides=(msg.row_step, msg.point_step))
            xyz = np.column_stack([raw[k].reshape(-1) for k in ("x", "y", "z")])
            ranges2 = np.sum(xyz * xyz, axis=1)
            valid = np.isfinite(xyz).all(axis=1) & (ranges2 > 1e-12)
            minimum = self.c.get("raw_zero_intensity_min_range_m", 0.)
            intensity = raw_fields.get("intensity")
            if minimum > 0 and intensity is not None and intensity.datatype == 7:
                strength = np.ndarray((msg.height,msg.width),dtype="<f4",buffer=msg.data,
                    offset=intensity.offset,strides=(msg.row_step,msg.point_step)).reshape(-1)
                # Observed 10–12 cm zero-strength near returns; retain all stronger or farther returns.
                valid &= (strength != 0) | (ranges2 >= minimum * minimum)
            xyz = xyz[valid]
            body = lidar_to_body(xyz, self.c["lidar_pose"])
            mask = ((body[:, 2] > .15) & (body[:, 2] < 1.4)
                    & (np.sum(body[:, :2] ** 2, axis=1) > self.c["robot_radius"] ** 2))
            self.raw_near_points = body[mask, :2]
        else:
            self.raw_near_points = np.empty((0, 2), dtype=np.float32)
        if self.mode != "ideal" or self.last_truth is None:
            return
        # Only ideal mode registers real simulated sensor points using truth.
        # Sensor mode's registered_scan is exclusively produced by Point-LIO.
        fields = {f.name: f for f in msg.fields}
        if any(k not in fields or fields[k].datatype != 7 for k in ("x", "y", "z")):
            return
        dtype = np.dtype(
            {
                "names": ["x", "y", "z"],
                "formats": ["<f4"] * 3,
                "offsets": [fields[k].offset for k in ("x", "y", "z")],
                "itemsize": msg.point_step,
            }
        )
        data = np.ndarray(
            (msg.height, msg.width),
            dtype=dtype,
            buffer=msg.data,
            strides=(msg.row_step, msg.point_step),
        )
        points = np.column_stack([data[k].reshape(-1) for k in ("x", "y", "z")])
        points = points[np.isfinite(points).all(axis=1)]
        points = lidar_to_body(points, self.c["lidar_pose"])
        q = self.last_truth.pose.pose.orientation
        theta = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        ct, st = math.cos(theta), math.sin(theta)
        points = points @ np.array([[ct, st, 0], [-st, ct, 0], [0, 0, 1]])
        p = self.last_truth.pose.pose.position
        points += np.array([p.x, p.y, 0])
        packed = np.column_stack([points, np.ones(len(points))]).astype("<f4")
        from sensor_msgs.msg import PointField

        cloud = PointCloud2()
        cloud.header = copy.deepcopy(msg.header)
        cloud.header.frame_id = "odom"
        cloud.height = 1
        cloud.width = len(points)
        cloud.fields = [
            PointField(name=k, offset=i * 4, datatype=7, count=1)
            for i, k in enumerate(["x", "y", "z", "intensity"])
        ]
        cloud.point_step = 16
        cloud.row_step = 16 * len(points)
        cloud.is_dense = True
        cloud.data = array("B", packed.tobytes())
        self.registered_pub.publish(cloud)

    def tick(self):
        now = time.monotonic()
        dt = min(now - self.last_tick, 0.1)
        self.last_tick = now
        stale = any(
            now - self.last.get(k, 0) > age
            for k, age in [
                ("command", 0.3),
                ("odom", 0.5),
                ("lidar", 0.7),
                ("imu", 0.2),
            ]
        )
        values = [self.cmd.linear.x, self.cmd.linear.y, self.cmd.angular.z]
        valid = all(math.isfinite(v) for v in values)
        target = values[:2] if self.motion_permitted and self.enabled and not stale and valid else [0.0, 0.0]
        speed_limit = self.c['bypass_speed'] if self.bypassing else self.c['max_speed']
        length = math.hypot(*target)
        if length > speed_limit:
            target = [v*speed_limit/length for v in target]
        if math.hypot(*target) > .05:
            self.last_motion_direction = tuple(target)
        speed = max(math.hypot(*target), math.hypot(*self.output))
        # shortcut: 0.2 s latency + configured braking, replace after vehicle stop-distance measurements.
        radial = self.c['robot_radius'] + self.c.get('minimum_clearance', .1)
        stop_radius = radial+.2*speed+speed*speed/(2*self.c['max_acceleration'])
        lateral = self.c['robot_radius'] if 'minimum_clearance' in self.c else radial
        min_points = self.c.get('raw_collision_min_points', 1)
        self.hazard = obstacle_near(self.raw_near_points, self.last_motion_direction,
            radial=radial, forward=stop_radius, lateral=lateral, immediate=lateral, min_points=min_points)
        self.guard_mode = 'velocity'
        if self.c.get('raw_collision_follow_path', False):
            path = self.body_path()
            if path is not None:
                self.hazard = obstacle_on_path(self.raw_near_points, path,
                    reach=stop_radius-radial+self.c.get('minimum_clearance', 0.), radius=self.c['robot_radius'], min_points=min_points)
                self.guard_mode = 'ordered_path'
        if self.hazard:
            target = [0.0, 0.0]
        self.hazard_pub.publish(Bool(data=self.hazard))
        length = math.hypot(*target)
        if length > self.c["max_speed"]:
            target = [v * self.c["max_speed"] / length for v in target]
        if not self.motion_permitted or not self.enabled or stale or not valid or self.hazard or not any(target):
            self.output = [0.0, 0.0]
        else:
            delta = [target[i] - self.output[i] for i in range(2)]
            d = math.hypot(*delta)
            limit = self.c["max_acceleration"] * dt
            self.output = (
                [self.output[i] + delta[i] * min(1.0, limit / d) for i in range(2)]
                if d
                else target
            )
        msg = Twist()
        msg.linear.x, msg.linear.y = self.output
        msg.angular.z = 0.0
        self.pub.publish(msg)
        self.diagnostic.publish(
            String(
                data=json.dumps(
                    {
                        "enabled": self.enabled,
                        "motion_permitted": self.motion_permitted,
                        "stale": stale,
                        "finite": valid,
                        "mode": self.mode,
                        "wz_disabled": True,
                        "raw_lidar_hazard": self.hazard,
                        "collision_check": self.guard_mode,
                    }
                )
            )
        )

    def stop(self):
        self.pub.publish(Twist())


def main():
    run(Adapter, lambda node: node.stop() if rclpy.ok() else None)
