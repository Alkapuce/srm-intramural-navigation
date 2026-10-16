"""Ordered FollowPath mission; crossing revisits cannot select later segments."""

import json, math, time
from pathlib import Path
import rclpy
from intramural_base.runtime import run
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav2_msgs.action import FollowPath
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Path as RosPath, OccupancyGrid
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from intramural_mission.route import segments, clearance, primitives, projected_progress, line
from intramural_mission.hazards import first_blocked
from intramural_mission.bypass import plan, entry_path


class Mission(Node):
    def __init__(self):
        super().__init__("mission")
        self.declare_parameter("arena_config", "/work/config/arena.json")
        self.c = json.loads(Path(self.get_parameter("arena_config").value).read_text())
        self.declare_parameter("start_permitted", True)
        self.start_permitted = self.get_parameter("start_permitted").value
        self.declare_parameter("variant", "figure8")
        self.variant = self.get_parameter("variant").value
        if self.variant not in ("figure8", "primitives"):
            raise ValueError(self.variant)
        self.route = (
            primitives(self.c) if self.variant == "primitives" else segments(self.c)
        )
        self.state = "IDLE"
        self.phase = "IDLE"
        self.index = -1
        self.progress = 0
        self.progress_fraction = 0.0
        self.segment_result = None
        self.goal_handle = None
        self.goal_pending = False
        self.pose = None
        self.last_pose = 0
        self.tf_problem = "No transform yet"
        self.tf_age = None
        self.reason = ""
        self.run = 0
        self.start_time = 0
        self.history = []
        self.navigator_active = False
        self.state_future = None
        self.last_check = 0
        self.finished = None
        self.costmap = None
        self.costmap_at = 0.0
        self.blocked_index = None
        self.blocked_count = 0
        self.bypass_count = 0
        self.bypassing = False
        self.resume_at = 0.0
        self.blocked_at = 0.0
        self.plan_at = 0.0
        self.grid_transform = None
        self.clear_since = None
        self.abort_at = None
        self.raw_hazard = False
        self.raw_hazard_at = 0.0
        self.ready_since = None
        self.static_map = None
        self.start_from_here = False
        self.tf = Buffer(node=self)
        self.listener = TransformListener(self.tf, self)
        self.client = ActionClient(self, FollowPath, "follow_path")
        self.lifecycle = self.create_client(GetState, "controller_server/get_state")
        self.create_subscription(OccupancyGrid, "local_costmap/costmap", self.receive_costmap,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(OccupancyGrid, "map", self.receive_map,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Bool, "motion_guard/hazard", self.receive_raw_hazard, 10)
        q = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(String, "mission/status", q)
        self.enable_pub = self.create_publisher(Bool, "mission/enabled", q)
        self.bypass_pub = self.create_publisher(Bool, "mission/bypass", q)
        self.path_pub = self.create_publisher(RosPath, "mission/path", q)
        self.collision_path_pub = self.create_publisher(RosPath, "mission/collision_path", q)
        for name, callback in [
            ("start", self.start),
            ("start_here", self.start_here),
            ("cancel", self.cancel),
            ("reset", self.reset),
        ]:
            self.create_service(Trigger, "mission/" + name, callback)
        self.enable_pub.publish(Bool(data=False))
        self.create_timer(
            0.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME)
        )

    def enabled(self, value):
        self.enable_pub.publish(Bool(data=value))

    def receive_costmap(self, msg):
        self.costmap = msg
        self.costmap_at = time.monotonic()

    def receive_map(self, msg):
        self.static_map = msg

    def receive_raw_hazard(self, msg):
        self.raw_hazard = msg.data
        self.raw_hazard_at = time.monotonic()

    def obstacle_index(self):
        if self.costmap is None or time.monotonic() - self.costmap_at > 0.7:
            return None
        grid = self.costmap
        try:
            t = self.tf.lookup_transform(grid.header.frame_id, "map", rclpy.time.Time())
        except Exception:
            return None
        q = t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        q = grid.info.origin.orientation
        origin_yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                                1 - 2 * (q.y * q.y + q.z * q.z))
        self.grid_transform = (t.transform.translation.x, t.transform.translation.y, yaw)
        arguments = (self.route[self.index][1], self.progress, self.pose,
            grid.data, grid.info.width, grid.info.height, grid.info.resolution,
            (grid.info.origin.position.x, grid.info.origin.position.y, origin_yaw),
            self.grid_transform)
        stopped = self.state == 'BLOCKED' or self.abort_at is not None
        blocked = first_blocked(*arguments, radius=self.c['robot_radius']+self.c.get('minimum_clearance', .12),
            threshold=100, lookahead=5.0 if stopped else 1.2)
        if blocked is None and stopped:
            # The controller also rejects inscribed cells while a removed object's inflation clears.
            blocked = first_blocked(*arguments, radius=self.c.get('minimum_clearance', .05), threshold=99, lookahead=5.0)
        return blocked

    def raw_obstacle(self):
        return self.raw_hazard and time.monotonic() - self.raw_hazard_at < .3

    def pause_for_obstacle(self, index):
        self.state = "BLOCKED"
        self.blocked_index = index
        self.blocked_count += 1
        self.clear_since = None
        self.abort_at = None
        self.blocked_at = time.monotonic()
        self.enabled(False)
        if self.goal_handle:
            self.goal_handle.cancel_goal_async()

    def resume_segment(self):
        # Same segment and ordered path; no progress reset or phase advancement.
        self.state = "RUNNING"
        self.blocked_index = None
        self.clear_since = None
        self.abort_at = None
        self.resume_at = time.monotonic()
        self.enabled(True)
        goal = FollowPath.Goal()
        goal.path = self.make_path(self.route[self.index][1][self.progress:])
        goal.controller_id = "FollowPath"
        goal.goal_checker_id = "general_goal_checker"
        self.path_pub.publish(goal.path)
        self.goal_pending = True
        self.client.send_goal_async(goal).add_done_callback(self.accepted)

    def try_bypass(self):
        if not self.c.get("allow_bypass", True):
            return
        now = time.monotonic()
        wait = .5 if self.bypassing else self.c['bypass_wait_seconds']
        if (now-self.blocked_at < wait or now-self.plan_at < 1.0
                or self.goal_pending or self.goal_handle is not None
                or now-self.costmap_at >= .7 or self.grid_transform is None):
            return
        self.plan_at = now
        grid = self.costmap
        tx, ty, yaw = self.grid_transform
        q = grid.info.origin.orientation
        angle = math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        co, so, cy, sy = math.cos(angle), math.sin(angle), math.cos(yaw), math.sin(yaw)
        obstacles = []
        for i, value in enumerate(grid.data):
            if value != 100:
                continue
            x,y = (i%grid.info.width+.5)*grid.info.resolution, (i//grid.info.width+.5)*grid.info.resolution
            gx = co*x-so*y+grid.info.origin.position.x-tx
            gy = so*x+co*y+grid.info.origin.position.y-ty
            obstacles.append((cy*gx+sy*gy,-sy*gx+cy*gy))
        replacement = plan(self.c,self.route,self.index,self.progress,self.pose,obstacles)
        if replacement is None:
            return  # A blocked corridor stays stopped; retry only with a fresh grid.
        phase, path = replacement[0]
        prefix = self.route[self.index][1][:self.progress]
        self.route[self.index:self.index+len(replacement)] = [(phase,prefix+path)]+replacement[1:]
        self.bypass_count += 1
        self.bypassing = True
        self.bypass_pub.publish(Bool(data=True))
        self.resume_segment()

    def start_here(self, request, response):
        return self.start(request, response, from_here=True)

    def start(self, request, response, from_here=False):
        if not self.start_permitted:
            response.message = "Vehicle standby: field geometry and physical safety not accepted"
            return response
        if self.state != "IDLE":
            response.message = "Reset required or mission already running"
            return response
        if (
            not self.navigator_active
            or not self.pose
            or time.monotonic() - self.last_pose > 0.5
            or self.ready_since is None
            or time.monotonic() - self.ready_since < 2.0
            or self.tf_age is None
            or abs(self.tf_age) >= .25
            or not self.client.server_is_ready()
        ):
            response.message = "Localization or controller not ready"
            return response
        if not from_here and math.dist(self.pose, self.c["parking"]) > self.c["parking_tolerance"]:
            response.message = "Robot is outside parking tolerance"
            return response
        route = primitives(self.c) if self.variant == 'primitives' else segments(self.c)
        if from_here:
            if self.variant != 'figure8':
                response.message = "Start here requires the figure8 mission"
                return response
            path = entry_path(self.c, self.pose, route[0][1][-1], self.static_map)
            if path is None:
                response.message = "No known free path from current pose to entry; check map and localization"
                return response
            route[0] = ('ENTER', path)
        self.run += 1
        self.history = []
        self.reason = ""
        self.finished = None
        self.blocked_index = None
        self.blocked_count = 0
        self.bypass_count = 0
        self.bypassing = False
        self.bypass_pub.publish(Bool(data=False))
        self.route = route
        self.start_from_here = from_here
        self.clear_since = None
        self.abort_at = None
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        self.state = "RUNNING"
        self.enabled(True)
        self.index = -1
        self.next_segment()
        response.success = True
        response.message = "Mission started from current pose; returning to configured parking" if from_here else "Mission started"
        return response

    def reset(self, request, response):
        if (
            self.state in ("RUNNING", "BLOCKED", "CANCELLING")
            or self.goal_handle is not None
            or self.goal_pending
        ):
            response.message = "Cancel and wait before reset"
            return response
        self.state = "IDLE"
        self.phase = "IDLE"
        self.index = -1
        self.progress = 0
        self.progress_fraction = 0.0
        self.segment_result = None
        self.reason = ""
        self.history = []
        self.enabled(False)
        response.success = True
        response.message = "Reset; use start from parking or start_here from current localized pose"
        return response

    def cancel(self, request, response):
        if self.state not in ("RUNNING", "BLOCKED"):
            response.message = "No active mission"
            return response
        self.state = "CANCELLING"
        self.enabled(False)
        if self.goal_handle:
            self.goal_handle.cancel_goal_async()
        elif self.segment_result is not None or not self.goal_pending:
            self.state = "CANCELLED"
            self.finished = self.get_clock().now().nanoseconds / 1e9
            self.segment_result = None
        response.success = True
        response.message = "Cancellation requested; wait for CANCELLED"
        return response

    def fault(self, reason):
        self.reason = reason
        self.state = "FAULT"
        self.finished = self.get_clock().now().nanoseconds / 1e9
        self.enabled(False)
        if self.goal_handle:
            self.goal_handle.cancel_goal_async()

    def make_path(self, points):
        path = RosPath()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()
        for x, y in points:
            p = PoseStamped()
            p.header = path.header
            p.pose.position.x = float(x)
            p.pose.position.y = float(y)
            p.pose.orientation.w = 1.0
            path.poses.append(p)
        return path

    def next_segment(self):
        if self.index == 0 and self.variant == 'figure8' and self.bypassing:
            entered = self.route[0][1]
            phase, exiting = self.route[-2]
            self.route[-2] = (phase, exiting[:-1] + line(exiting[-1], entered[-1], self.c['path_spacing']))
            returning = (entry_path(self.c, entered[-1], self.c['parking'], self.static_map)
                         if self.start_from_here else list(reversed(entered)))
            if returning is None:
                self.fault("No known free return path to parking")
                return
            self.route[-1] = ('RETURN', returning)
        self.index += 1
        self.progress = 0
        self.progress_fraction = 0.0
        self.segment_result = None
        self.abort_at = None
        if self.index == len(self.route):
            self.state = "DONE"
            self.phase = "DONE"
            self.finished = self.get_clock().now().nanoseconds / 1e9
            self.enabled(False)
            return
        self.phase, points = self.route[self.index]
        self.history.append(
            {
                "phase": self.phase,
                "segment": self.index,
                "sim_time": self.get_clock().now().nanoseconds / 1e9,
            }
        )
        goal = FollowPath.Goal()
        goal.path = self.make_path(points)
        goal.controller_id = "FollowPath"
        goal.goal_checker_id = "general_goal_checker"
        self.path_pub.publish(goal.path)
        self.goal_pending = True
        self.client.send_goal_async(goal).add_done_callback(self.accepted)

    def accepted(self, future):
        self.goal_pending = False
        try:
            self.goal_handle = future.result()
        except Exception as exc:
            self.fault(str(exc))
            return
        if not self.goal_handle.accepted:
            self.goal_handle = None
            self.fault("Controller rejected segment")
            return
        self.goal_handle.get_result_async().add_done_callback(self.result)
        if self.state != "RUNNING" and self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()

    def result(self, future):
        self.goal_handle = None
        try:
            status = future.result().status
        except Exception as exc:
            self.fault(str(exc))
            return
        if self.state == "CANCELLING":
            self.state = "CANCELLED" if status in (4, 5) else "FAULT"
            self.finished = self.get_clock().now().nanoseconds / 1e9
            self.reason = "" if status in (4, 5) else f"Cancel result {status}"
            return
        if self.state != "RUNNING":
            return
        if status == 6:
            # Controller can see an occupied costmap before the 5 Hz published
            # grid reaches this node. Allow its next grid to prove blockage.
            self.abort_at = time.monotonic()
            return
        if status != 4:
            self.fault(f"Segment {self.index}: action status={status}")
            return
        # Action result and TF callbacks can arrive in either order. Do not
        # decide from the previous timer's pose or a rounded sample index.
        self.segment_result = status
        self.result_at = time.monotonic()

    def finish_segment(self):
        if self.segment_result is None:
            return
        points = self.route[self.index][1]
        tolerance = self.c["parking_tolerance"] if self.route[self.index][0] == "RETURN" else self.c["checkpoint_tolerance"]
        # Metric path remainder preserves ordered progress without a conflicting percentage gate on short segments.
        remaining = sum(math.dist(a, b) * min(1.0, max(0.0, i + 1 - self.progress_fraction))
                        for i, (a, b) in enumerate(zip(points, points[1:])))
        if (
            self.pose is not None
            and math.dist(self.pose, points[-1]) <= tolerance
            and remaining <= tolerance
        ):
            self.segment_result = None
            self.next_segment()
        elif time.monotonic() - self.result_at > 0.5:
            self.fault(
                f"Segment {self.index}: completed action but remaining={remaining:.3f}m, tolerance={tolerance:.3f}m, pose={self.pose}"
            )

    def tick(self):
        now = time.monotonic()
        try:
            t = self.tf.lookup_transform("map", "base_footprint", rclpy.time.Time())
            stamp = t.header.stamp.sec + t.header.stamp.nanosec / 1e9
            self.tf_age = self.get_clock().now().nanoseconds / 1e9 - stamp
            if abs(self.tf_age) < 0.5:
                self.pose = [t.transform.translation.x, t.transform.translation.y]
                self.last_pose = now
                self.tf_problem = ""
            else:
                self.tf_problem = f"TF stamp age {self.tf_age:.3f} simulation seconds"
        except Exception as exc:
            self.tf_problem = str(exc)
        if (
            now - self.last_check > 1
            and self.lifecycle.service_is_ready()
            and (self.state_future is None or self.state_future.done())
        ):
            self.last_check = now
            self.state_future = self.lifecycle.call_async(GetState.Request())
            self.state_future.add_done_callback(self.lifecycle_state)
        if self.navigator_active:
            self.ready_since = self.ready_since or now
        else:
            self.ready_since = None
        if self.state in ("RUNNING", "BLOCKED"):
            if self.state == "RUNNING" and now-self.resume_at > .4 and self.raw_obstacle():
                self.pause_for_obstacle(self.progress)
                self.publish_status()
                return
            if now - self.last_pose > 0.5:
                self.fault("Localization stale: " + self.tf_problem)
            elif self.costmap_at and now - self.costmap_at > 1.0:
                self.fault("Local costmap stale")
            else:
                blocked = self.obstacle_index()
                if self.state == "RUNNING" and blocked is not None:
                    self.pause_for_obstacle(blocked)
                elif self.state == "BLOCKED":
                    if blocked is None and not self.raw_obstacle() and now - self.costmap_at < 0.7:
                        self.clear_since = self.clear_since or now
                        if now - self.clear_since > 0.5 and not self.goal_pending and self.goal_handle is None:
                            self.resume_segment()
                    else:
                        self.clear_since = None
                        self.try_bypass()
                if self.state != "RUNNING":
                    self.publish_status()
                    return
                if self.abort_at is not None:
                    if now - self.abort_at > 0.8:
                        self.fault(f"Segment {self.index}: controller aborted without confirmed obstacle")
                    self.publish_status()
                    return
                points = self.route[self.index][1]
                end = min(len(points), self.progress + 20)
                candidate = min(
                    range(self.progress, end),
                    key=lambda i: math.dist(points[i], self.pose),
                )
                error = math.dist(points[candidate], self.pose)
                if error > self.c["max_tracking_error"]:
                    self.fault(f"Path tracking error {error:.3f}")
                elif min(clearance(self.c, self.pose)) < self.c.get("minimum_clearance", 0.05):
                    self.fault("Pole or boundary clearance insufficient")
                else:
                    self.progress = max(self.progress, candidate)
                    self.progress_fraction = max(
                        self.progress_fraction,
                        projected_progress(points, candidate, self.pose),
                    )
                    self.finish_segment()
        self.publish_status()

    def publish_status(self):
        now = time.monotonic()
        if self.state in ('RUNNING', 'BLOCKED') and 0 <= self.index < len(self.route):
            future, travelled = [], 0.
            horizon = max(1.2, .2*self.c['max_speed']+self.c['max_speed']**2/(2*self.c['max_acceleration'])+.02)
            for index in range(self.index, len(self.route)):
                for p in self.route[index][1][self.progress if index == self.index else 0:]:
                    if future:
                        travelled += math.dist(future[-1], p)
                    future.append(p)
                    if travelled >= horizon:
                        break
                if travelled >= horizon:
                    break
            self.collision_path_pub.publish(self.make_path(future))
        clock_time = (
            self.get_clock().now().nanoseconds / 1e9
            if self.finished is None
            else self.finished
        )
        report = {
            "state": self.state,
            "phase": self.phase,
            "segment": self.index,
            "progress": self.progress,
            "continuous_progress": self.progress_fraction,
            "run": self.run,
            "pose": self.pose,
            "tf_age_sim_seconds": self.tf_age,
            "tf_problem": self.tf_problem,
            "start_permitted": self.start_permitted,
            "ready": self.start_permitted and self.ready_since is not None and now - self.ready_since >= 2.0
            and now - self.last_pose < .25 and self.tf_age is not None
            and abs(self.tf_age) < .25,
            "reason": self.reason,
            "blocked_index": self.blocked_index,
            "blocked_count": self.blocked_count,
            "bypass_count": self.bypass_count,
            "bypassing": self.bypassing,
            "raw_lidar_hazard": self.raw_obstacle(),
            "history": self.history,
            "elapsed_sim_seconds": clock_time - self.start_time if self.run else 0,
        }
        self.pub.publish(String(data=json.dumps(report)))

    def lifecycle_state(self, future):
        try:
            self.navigator_active = future.result().current_state.id == 3
        except Exception:
            self.navigator_active = False


def main():
    run(Mission, lambda node: node.enabled(False) if rclpy.ok() else None)
