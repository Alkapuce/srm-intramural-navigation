"""Explicit ideal-localization adapter and sole final velocity publisher."""

import copy, json, math, time
import numpy as np
from array import array
from collections import deque
from pathlib import Path
import rclpy
from intramural_base.runtime import run
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry, Path as RosPath
from sensor_msgs.msg import PointCloud2, Imu, PointField
from std_msgs.msg import Bool, String
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster, Buffer, TransformListener, TransformException
from intramural_mission.proximity import obstacle_near, obstacle_on_path, lidar_to_body
from intramural_mission.route import curve_speed_limit
from intramural_mission.safety import executable_path, response_horizon, collision_reach, compensate_motion, map_motion_pose


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
        self.body_velocity = np.zeros(2)
        self.previous_odometry = None
        self.odom_history = deque(maxlen=32)
        self.raw_point_times = None
        self.raw_motion_problem = ''
        self.velocity_stamp = 0.
        self.last_tick = time.monotonic()
        self.raw_near_points = np.empty((0, 2), dtype=np.float32)
        self.last_motion_direction = (0.0, 0.0)
        self.hazard = False
        self.atomic_command = self.c.get('navigation_engine') == 'trajectory'
        self.snapshot_stamp = -math.inf
        self.observed_stamp = -math.inf
        self.observation_pub = self.create_publisher(String, 'motion_guard/observations', 1)
        self.mission_phase = ''
        self.create_subscription(String,'mission/status',self.receive_mission_status,10)
        self.collision_path = None
        if self.c.get('raw_collision_follow_path', False):
            self.path_tf = Buffer(node=self)
            self.path_listener = TransformListener(self.path_tf, self)
            if self.c.get('navigation_engine')=='trajectory':
                self.create_subscription(String,'mission/trajectory',self.receive_trajectory,
                    QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
            else:
                self.create_subscription(RosPath, 'mission/collision_path', self.receive_path,
                    QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.hazard_pub = self.create_publisher(Bool, "motion_guard/hazard", 10)
        self.diagnostic = self.create_publisher(String, "motion_guard/status", 10)
        self.create_subscription(Twist, "cmd_vel_nav2_result", self.command, 10)
        self.create_subscription(Twist, "cmd_vel_controller", self.controller_command, 10)
        self.create_subscription(
            Bool,
            "mission/enabled",
            self.enable,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.create_subscription(Bool, 'mission/bypass', self.bypass,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(
            Odometry, "odometry", self.odometry, qos_profile_sensor_data
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
            .01 if self.atomic_command else .025, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME)
        )

    def touch(self, key):
        self.last[key] = time.monotonic()

    def odometry(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec/1e9
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        position = np.array([p.x, p.y])
        if (not np.isfinite([*position,q.x,q.y,q.z,q.w,stamp]).all()
                or abs(self.get_clock().now().nanoseconds/1e9-stamp) > .5):
            return
        previous = self.previous_odometry
        if previous is not None and stamp <= previous[0]:
            return
        if previous is not None:
            dt = stamp-previous[0]
            velocity = (position-previous[1])/dt
            if np.linalg.norm(velocity) > 2*self.c['max_speed']:
                return
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            co, si = math.cos(yaw), math.sin(yaw)
            self.body_velocity = np.array([co*velocity[0]+si*velocity[1],-si*velocity[0]+co*velocity[1]])
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        self.previous_odometry = (stamp, position)
        self.odom_history.append((stamp, *position, yaw))
        self.velocity_stamp = stamp
        self.touch('odom')

    def receive_mission_status(self,msg):
        status=json.loads(msg.data)
        self.mission_phase=status['phase']
        self.mission_state=status.get('state','')
        self.mission_context=(status.get('run_id',''),status.get('route_id',''))

    def receive_trajectory(self,msg):
        d=json.loads(msg.data)
        points=np.asarray(d['points'],dtype=float)
        pose=np.asarray(d.get('pose'),dtype=float)
        context=(d['run_id'],d['route_id'])
        command=np.asarray(d.get('command', [0., 0.]), dtype=float)
        if (points.ndim!=2 or points.shape[1]!=2 or not np.isfinite(points).all() or not math.isfinite(d['stamp'])
                or pose.shape!=(3,) or not np.isfinite(pose).all() or not math.isfinite(d['pose_stamp'])
                or command.shape!=(2,) or not np.isfinite(command).all()):
            self.collision_path=None
            self.cmd = Twist()
            return
        if self.atomic_command:
            now=self.get_clock().now().nanoseconds/1e9
            if abs(now-d['pose_stamp']) > .25:
                self.collision_path=None
                self.cmd=Twist()
                return
            if (getattr(self,'mission_state','') in ('RUNNING','BLOCKED','CANCELLING')
                    and context[0] != getattr(self,'mission_context',('',''))[0]):
                return
            if abs(now-d['stamp']) > .3 or d['stamp'] <= self.snapshot_stamp:
                return
            self.snapshot_stamp=d['stamp']
            self.mission_context=context
            self.cmd=Twist()
            self.cmd.linear.x,self.cmd.linear.y=map(float,command)
            self.touch('command')
            if not np.any(command):self.output=[0.,0.]
        self.collision_context=context
        self.collision_pose, self.collision_pose_stamp = pose, d['pose_stamp']
        from geometry_msgs.msg import PoseStamped
        path=RosPath();path.header.frame_id='map';path.header.stamp.sec=int(d['stamp'])
        path.header.stamp.nanosec=int((d['stamp']-int(d['stamp']))*1e9)
        for x,y in points:
            p=PoseStamped();p.pose.position.x=float(x);p.pose.position.y=float(y);path.poses.append(p)
        self.collision_path=path

    def receive_path(self, msg):
        self.collision_path = msg

    def body_path(self):
        self.path_problem = ''
        if self.collision_path is None or len(self.collision_path.poses) < 2:
            self.path_problem = 'NO_PATH'
            return None
        if self.c.get('navigation_engine')=='trajectory':
            expected=getattr(self,'mission_context',None)
            actual=getattr(self,'collision_context',None)
            if actual is None or expected is None or (actual[0]!=expected[0] if self.atomic_command else actual!=expected):
                self.path_problem = 'CONTEXT_MISMATCH'
                return None
            now=self.get_clock().now().nanoseconds/1e9
            stamp=self.collision_path.header.stamp.sec+self.collision_path.header.stamp.nanosec/1e9
            if abs(now-stamp)>.3 or abs(now-self.collision_pose_stamp)>.25:
                self.path_problem = 'PATH_OR_POSE_STALE'
                return None
            # Match the controller's observed pose instead of a second, independently received TF.
            x,y,yaw=self.collision_pose
            co,si=math.cos(yaw),math.sin(yaw)
            return [[co*(p.pose.position.x-x)+si*(p.pose.position.y-y),
                     -si*(p.pose.position.x-x)+co*(p.pose.position.y-y)] for p in self.collision_path.poses]
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
        if self.atomic_command:
            return
        self.cmd = msg
        self.touch("command")

    def controller_command(self, msg):
        if self.atomic_command:
            return
        # A segment-end zero must survive the upstream smoother and timer ordering.
        if msg.linear.x == 0.0 and msg.linear.y == 0.0:
            self.cmd = Twist()
            self.output = [0.0, 0.0]
            self.pub.publish(Twist())

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
            stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
            body = lidar_to_body(xyz, self.c["lidar_pose"])
            if np.any(np.sum(body[:,:2]**2,axis=1)>self.c["robot_radius"]**2) and abs(self.get_clock().now().nanoseconds/1e9-stamp)<.7:
                self.touch("lidar")
            mask = ((body[:, 2] > .15) & (body[:, 2] < 1.4)
                    & (np.sum(body[:, :2] ** 2, axis=1) > self.c["robot_radius"] ** 2))
            tag = raw_fields.get("tag")
            offset = raw_fields.get('offset_time')
            self.raw_point_times = None
            self.raw_trusted_times = None
            if offset is not None and offset.datatype == PointField.UINT32:
                offsets = np.ndarray((msg.height,msg.width),dtype='<u4',buffer=msg.data,
                    offset=offset.offset,strides=(msg.row_step,msg.point_step)).reshape(-1)[valid]
                times = stamp + offsets*1e-9
                self.raw_point_times = times[mask]
            self.raw_trusted_points = None
            if tag is not None and tag.datatype == PointField.UINT8:
                confidence = np.ndarray((msg.height,msg.width),dtype="u1",buffer=msg.data,
                    offset=tag.offset,strides=(msg.row_step,msg.point_step)).reshape(-1)[valid]
                # Preserve contact protection regardless of MID-360 confidence.
                contact = self.c["robot_radius"] + self.c.get("minimum_clearance", .02)
                supported = ((confidence & 0x3f) == 0) | (np.sum(body[:,:2]**2,axis=1) < contact**2)
                self.raw_trusted_points = body[mask & supported, :2]
                if self.raw_point_times is not None:
                    self.raw_trusted_times = times[mask & supported]
            self.raw_stamp = stamp
            self.raw_near_points = body[mask, :2]
        else:
            self.raw_near_points = np.empty((0, 2), dtype=np.float32)
            self.raw_point_times = None
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
        speed_limit = self.c['bypass_speed'] if self.bypassing and not self.atomic_command else self.c['max_speed']
        path = self.body_path() if self.c.get('raw_collision_follow_path', False) else None
        raw_points = self.raw_near_points
        trusted_points = getattr(self, 'raw_trusted_points', None)
        sensor_age = 0.
        self.raw_motion_problem = ''
        if self.c.get('navigation_engine') == 'trajectory' and getattr(self, 'raw_point_times', None) is not None:
            now_ros = self.get_clock().now().nanoseconds/1e9
            sensor_age = max(0., now_ros-self.raw_stamp)
            try:
                raw_points = compensate_motion(raw_points, self.raw_point_times, self.odom_history, now_ros)
                if trusted_points is not None:
                    trusted_points = compensate_motion(trusted_points, self.raw_trusted_times, self.odom_history, now_ros)
                if path is not None:
                    path = compensate_motion(path, self.collision_pose_stamp, self.odom_history, now_ros).tolist()
            except ValueError as error:
                self.raw_motion_problem = str(error)
                stale = True
        if path is not None and self.c.get('continuous_corners', False) and self.c.get('navigation_engine')!='trajectory':
            speed_limit = min(speed_limit, curve_speed_limit(path, speed_limit,
                self.c['lateral_acceleration'], self.c['max_acceleration']))
            if getattr(self,'mission_phase','')=='RETURN':
                remaining=math.hypot(*path[0])+sum(math.dist(a,b)for a,b in zip(path,path[1:]))
                speed_limit=min(speed_limit,max(.05,math.sqrt(2*self.c['max_acceleration']*max(0.,remaining-self.c['parking_tolerance']))))
        if (self.c.get('allow_bypass', False) and self.atomic_command and not self.raw_motion_problem
                and getattr(self,'raw_stamp',-math.inf)>self.observed_stamp
                and getattr(self,'collision_pose',None) is not None):
            self.observed_stamp=self.raw_stamp
            x,y,yaw=self.collision_pose
            if self.raw_point_times is not None:
                (x,y),yaw=map_motion_pose(self.collision_pose,self.collision_pose_stamp,
                    self.odom_history,self.get_clock().now().nanoseconds/1e9)
            co,si=math.cos(yaw),math.sin(yaw)
            world=raw_points@np.array([[co,si],[-si,co]])+np.array([x,y])
            counts=np.ones(len(world),dtype=int)
            # Same compensated body observations as the emergency guard; no second scan filter.
            if len(world):
                _,indices,counts=np.unique(np.floor(world/.04).astype(int),axis=0,return_index=True,return_counts=True)
                world=world[indices]
                within=np.linalg.norm(world-[x,y],axis=1)<4.
                world,counts=world[within],counts[within]
            self.observation_pub.publish(String(data=json.dumps(dict(stamp=self.raw_stamp,
                reference_stamp=self.collision_pose_stamp,points=world.tolist(),weights=counts.tolist()))))
        length = math.hypot(*target)
        if length > speed_limit:
            target = [v*speed_limit/length for v in target]
        delta = [target[i]-self.output[i] for i in range(2)]
        d=math.hypot(*delta)
        scheduled=([self.output[i]+delta[i]*min(1.,self.c['max_acceleration']*dt/d)for i in range(2)] if d else target)
        if not any(target):scheduled=[0.,0.]
        if math.hypot(*scheduled) > .05:
            self.last_motion_direction = tuple(scheduled)
        observed_velocity = getattr(self, 'body_velocity', self.output)
        if math.hypot(*observed_velocity) > .05:
            self.last_motion_direction = tuple(observed_velocity)
        speed = max(math.hypot(*scheduled), math.hypot(*self.output), math.hypot(*observed_velocity))
        if self.c.get('navigation_engine')=='trajectory':
            if getattr(self,'mission_state','')=='BLOCKED':speed=max(speed,getattr(self,'held_speed',0.))
            else:self.held_speed=speed
        radial = self.c['robot_radius'] + self.c.get('minimum_clearance', .1)
        if self.c.get('navigation_engine') == 'trajectory':
            stop_radius = radial + collision_reach(self.c, speed, sensor_age) - self.c.get('minimum_clearance', 0.)
        else:
            # Legacy control retains its original constant-deceleration model.
            braking = min(self.c['max_acceleration'], self.c.get('braking_acceleration', self.c['max_acceleration']))
            stop_radius = radial+.2*speed+speed*speed/(2*braking)
        lateral = self.c['robot_radius'] if 'minimum_clearance' in self.c else radial
        min_points = self.c.get('raw_collision_min_points', 1)
        self.hazard = obstacle_near(raw_points, self.last_motion_direction,
            radial=radial, forward=stop_radius, lateral=lateral, immediate=lateral, min_points=min_points)
        self.guard_mode = 'velocity'
        if self.c.get('raw_collision_follow_path', False):
            if path is not None:
                if self.c.get('navigation_engine')=='trajectory':
                    path=executable_path(path,observed_velocity,response_horizon(self.c))
                self.hazard = obstacle_on_path(raw_points, path,
                    reach=stop_radius-radial+self.c.get('minimum_clearance', 0.), radius=self.c['robot_radius'], min_points=min_points)
                self.guard_mode = 'ordered_path'
        if self.hazard and path is not None and getattr(self, 'raw_trusted_points', None) is not None:
            trusted = obstacle_on_path(trusted_points, path,
                reach=stop_radius-radial+self.c.get('minimum_clearance', 0.), radius=self.c['robot_radius'], min_points=min_points)
            if trusted or np.count_nonzero(np.linalg.norm(raw_points,axis=1) < radial) >= min_points:
                self.pending_raw_stamp = None
            else:
                # A single uncertain scan must not stop the car; persistent weak obstacles still stop it.
                if getattr(self, 'pending_raw_stamp', None) is None:self.pending_raw_stamp = self.raw_stamp
                self.hazard = self.raw_stamp > self.pending_raw_stamp
        else:
            self.pending_raw_stamp = None
        if self.hazard:
            target = [0.0, 0.0]
        if self.c.get('navigation_engine')=='trajectory' and self.enabled:
            if self.c.get('raw_collision_follow_path') and path is None:stale=True
        self.hazard_pub.publish(Bool(data=self.hazard))
        length = math.hypot(*target)
        if length > self.c["max_speed"]:
            target = [v * self.c["max_speed"] / length for v in target]
        if not self.motion_permitted or not self.enabled or stale or not valid or self.hazard or not any(target):
            self.output = [0.0, 0.0]
        else:
            self.output = scheduled
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
                        "sensors_fresh": not self.raw_motion_problem and all(now-self.last.get(k,0)<age for k,age in (("odom",.5),("lidar",.7),("imu",.2))),
                        "finite": valid,
                        "mode": self.mode,
                        "wz_disabled": True,
                        "raw_lidar_hazard": self.hazard,
                        "raw_hazard_unconfirmed": getattr(self, "pending_raw_stamp", None) is not None and not self.hazard,
                        "collision_check": self.guard_mode,
                        "collision_reach_m": stop_radius-radial+self.c.get('minimum_clearance', 0.),
                        "raw_source_age_seconds": sensor_age,
                        "raw_motion_compensated": self.c.get('navigation_engine') == 'trajectory' and getattr(self, 'raw_point_times', None) is not None and not self.raw_motion_problem,
                        "raw_motion_problem": self.raw_motion_problem,
                        "path_problem": getattr(self, "path_problem", ''),
                        "path_speed_limit": speed_limit,
                        "route_id": getattr(self,'collision_context',('',''))[1],
                        "guard_tick_ms": (time.monotonic()-now)*1000,
                        "command": values[:2], "output": self.output,
                        "velocity_observed_body": list(observed_velocity),
                        "input_ages": {k: now-self.last.get(k, 0.) for k in ('command','odom','lidar','imu')},
                        "nearest_raw_points": (raw_points[np.argsort(np.sum(raw_points**2,axis=1))[:6]].tolist()
                                               if self.hazard else []),
                        "collision_prefix": path[:8] if self.hazard and path is not None else [],
                    }
                )
            )
        )

    def stop(self):
        self.pub.publish(Twist())


def main():
    run(Adapter, lambda node: node.stop() if rclpy.ok() else None)
