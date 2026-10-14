"""Live regression for independent cloud/odometry arrival during LOAM startup."""
import os
import signal
import struct
import subprocess
import tempfile
import time
import unittest
from pathlib import Path

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy
from ament_index_python.packages import get_package_prefix
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage


class LoamStartupTests(unittest.TestCase):
    def test_cloud_waits_for_mount_initialization_then_uses_known_transform(self):
        context=Context();rclpy.init(context=context,domain_id=75)
        node=rclpy.create_node('loam_startup_regression',context=context)
        executor=SingleThreadedExecutor(context=context);executor.add_node(node)
        env=dict(os.environ,ROS_DOMAIN_ID='75')
        binary=Path(get_package_prefix('loam_interface'))/'lib/loam_interface/loam_interface_node'
        with tempfile.TemporaryFile(mode='w+') as log:
            child=subprocess.Popen([str(binary),'--ros-args','-r','__ns:=/loam_regression',
                '-r','/tf:=tf','-r','/tf_static:=tf_static',
                '-p','state_estimation_topic:=raw_odom','-p','registered_scan_topic:=raw_scan',
                '-p','base_frame:=test_base','-p','lidar_frame:=test_lidar','-p','odom_frame:=test_odom'],
                env=env,stdout=log,stderr=subprocess.STDOUT)
            try:
                pub=node.create_publisher(PointCloud2,'/loam_regression/raw_scan',10)
                odom_pub=node.create_publisher(Odometry,'/loam_regression/raw_odom',10)
                received=[]
                node.create_subscription(PointCloud2,'/loam_regression/registered_scan',received.append,10)
                cloud=PointCloud2(height=1,width=1,point_step=16,row_step=16,is_dense=True)
                cloud.header.frame_id='lidar_odom';cloud.header.stamp.sec=10
                cloud.fields=[PointField(name=k,offset=i*4,datatype=7,count=1) for i,k in enumerate(('x','y','z','intensity'))]
                cloud.data=struct.pack('<ffff',1,0,0,1)
                deadline=time.monotonic()+5
                while pub.get_subscription_count()==0 and time.monotonic()<deadline:
                    self.assertIsNone(child.poll())
                    executor.spin_once(timeout_sec=.02)
                self.assertGreater(pub.get_subscription_count(),0)
                deadline=time.monotonic()+.5
                while time.monotonic()<deadline:
                    pub.publish(cloud);executor.spin_once(timeout_sec=.02)
                self.assertEqual(len(received),0,'Cloud was transformed before the mount was initialized')
                static=node.create_publisher(TFMessage,'/loam_regression/tf_static',
                    QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
                t=TransformStamped();t.header.frame_id='test_base';t.child_frame_id='test_lidar'
                t.transform.translation.x=.15;t.transform.translation.y=-.15;t.transform.translation.z=.22
                t.transform.rotation.w=1.
                static.publish(TFMessage(transforms=[t]))
                odom=Odometry();odom.header.stamp=cloud.header.stamp;odom.pose.pose.orientation.w=1.
                deadline=time.monotonic()+5
                while not received and time.monotonic()<deadline:
                    odom_pub.publish(odom);pub.publish(cloud);executor.spin_once(timeout_sec=.02)
                self.assertTrue(received,'Initialized cloud was not forwarded')
                self.assertEqual(received[-1].header.frame_id,'test_odom')
                for actual,expected in zip(struct.unpack_from('<fff',received[-1].data),[1.15,-.15,.22]):
                    self.assertAlmostEqual(actual,expected,places=5)
            finally:
                if child.poll() is None:child.send_signal(signal.SIGINT)
                try:child.wait(timeout=8)
                except subprocess.TimeoutExpired:child.kill();child.wait()
                executor.shutdown();node.destroy_node();context.shutdown()
                self.assertEqual(child.returncode,0)
