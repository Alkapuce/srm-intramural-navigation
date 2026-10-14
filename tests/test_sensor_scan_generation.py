"""Live ROS regression: navigation odometry must not wait for a point-cloud pair."""
import os
import signal
import subprocess
import tempfile
import time
import unittest
from pathlib import Path

import rclpy
from rclpy.context import Context
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from rclpy.executors import SingleThreadedExecutor
from ament_index_python.packages import get_package_prefix
from rclpy.qos import qos_profile_sensor_data


class SensorOdometryTests(unittest.TestCase):
    def test_body_odometry_flows_without_cloud_and_keeps_source_stamp(self):
        context = Context()
        rclpy.init(context=context, domain_id=72)
        node = rclpy.create_node('odometry_sync_regression', context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        env = dict(os.environ, ROS_DOMAIN_ID='72')
        with tempfile.TemporaryFile(mode='w+') as log:
            child = subprocess.Popen([str(Path(get_package_prefix('sensor_scan_generation')) / 'lib/sensor_scan_generation/sensor_scan_generation_node'), '--ros-args', '-r', '__ns:=/sync_regression',
                '-r', '/tf:=tf', '-r', '/tf_static:=tf_static',
                '-p', 'lidar_frame:=test_lidar', '-p', 'base_frame:=test_base',
                '-p', 'robot_base_frame:=test_base'], env=env, stdout=log,
                stderr=subprocess.STDOUT)
            try:
                transform = TransformStamped()
                transform.header.frame_id = 'test_base'
                transform.child_frame_id = 'test_lidar'
                transform.transform.translation.x = .15
                transform.transform.translation.y = -.15
                transform.transform.translation.z = .22
                transform.transform.rotation.w = 1.
                # Publish on the same namespaced static-TF topic as the child.
                from tf2_msgs.msg import TFMessage
                from rclpy.qos import QoSProfile, DurabilityPolicy
                static = node.create_publisher(TFMessage,'/sync_regression/tf_static',
                    QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
                static.publish(TFMessage(transforms=[transform]))
                publisher = node.create_publisher(Odometry, '/sync_regression/lidar_odometry', qos_profile_sensor_data)
                received = []
                node.create_subscription(Odometry, '/sync_regression/odometry', received.append, qos_profile_sensor_data)
                msg = Odometry()
                msg.header.frame_id = 'test_odom'
                msg.child_frame_id = 'test_lidar'
                msg.header.stamp.sec = 10
                msg.header.stamp.nanosec = 123
                msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = 1.15, 1.85, .22
                msg.pose.pose.orientation.w = 1.
                deadline = time.monotonic()+5
                while time.monotonic() < deadline and len(received) < 10:
                    self.assertIsNone(child.poll(),'Odometry node exited early')
                    publisher.publish(msg)
                    executor.spin_once(timeout_sec=.02)
                self.assertGreaterEqual(len(received),10,'Odometry is gated by absent point clouds')
                for output in received:
                    self.assertEqual(output.header.stamp,msg.header.stamp)
                    self.assertEqual(output.child_frame_id,'test_base')
                    self.assertAlmostEqual(output.pose.pose.position.x,1.,places=6)
                    self.assertAlmostEqual(output.pose.pose.position.y,2.,places=6)
                    self.assertAlmostEqual(output.pose.pose.position.z,0.,places=6)
            finally:
                child.send_signal(signal.SIGINT)
                try:
                    child.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    child.kill();child.wait()
                executor.shutdown()
                node.destroy_node()
                context.shutdown()
                self.assertEqual(child.returncode,0,'Odometry node did not shut down cleanly')
