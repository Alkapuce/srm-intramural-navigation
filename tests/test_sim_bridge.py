"""Exit the production bridge while real 1 MiB transport callbacks are in flight."""
import os
from pathlib import Path
import signal
import struct
import subprocess
import tempfile
import time
import unittest
import uuid

from ament_index_python.packages import get_package_prefix
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import PointCloud2, PointField


class SimBridgeTests(unittest.TestCase):
    def test_signals_drain_live_pointcloud_transport_before_exit(self):
        context = Context()
        rclpy.init(context=context, domain_id=96)
        node = rclpy.create_node('bridge_shutdown_driver', context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        topic = '/bridge_shutdown_' + uuid.uuid4().hex
        publisher = node.create_publisher(PointCloud2, topic, 10)
        cloud = PointCloud2(height=1, width=65536, point_step=16, row_step=1048576,
                            is_dense=True)
        cloud.header.frame_id = 'lidar'
        cloud.fields = [PointField(name=k, offset=4*i, datatype=7, count=1)
                        for i, k in enumerate(('x', 'y', 'z', 'intensity'))]
        cloud.data = struct.pack('<ffff', 1., 0., .3, 1.) * cloud.width
        env = dict(os.environ, ROS_DOMAIN_ID='96', IGN_PARTITION=topic[1:])
        producer = None
        try:
            with tempfile.TemporaryDirectory() as directory:
                folder = Path(directory)
                with (folder/'producer.log').open('w+') as source_log:
                    producer = subprocess.Popen([
                        get_package_prefix('ros_gz_bridge')+'/lib/ros_gz_bridge/parameter_bridge',
                        topic+'@sensor_msgs/msg/PointCloud2]ignition.msgs.PointCloudPacked'],
                        env=env, stdout=source_log, stderr=subprocess.STDOUT)
                    for i in range(6):
                        received = []
                        output = topic + '/output_' + str(i)
                        subscriber = node.create_subscription(
                            PointCloud2, output, lambda msg: received.append(len(msg.data)), 10)
                        config = folder/'bridge.yaml'
                        config.write_text(f'''- ros_topic_name: {output}
  gz_topic_name: {topic}
  ros_type_name: sensor_msgs/msg/PointCloud2
  gz_type_name: ignition.msgs.PointCloudPacked
  direction: GZ_TO_ROS
''')
                        with (folder/f'bridge-{i}.log').open('w+') as log:
                            process = subprocess.Popen([
                                get_package_prefix('intramural_sim')+'/lib/intramural_sim/sensor_bridge',
                                '--ros-args', '-p', 'config_file:='+str(config)],
                                env=env, stdout=log, stderr=subprocess.STDOUT)
                            try:
                                deadline = time.monotonic() + 10.
                                while len(received) < 3 and time.monotonic() < deadline:
                                    if process.poll() is not None:
                                        break
                                    publisher.publish(cloud)
                                    executor.spin_once(timeout_sec=.01)
                                self.assertGreaterEqual(len(received), 3)
                                self.assertTrue(all(size == cloud.row_step for size in received))
                                for _ in range(8):
                                    publisher.publish(cloud)
                                time.sleep((0., .001, .01)[i % 3])
                                process.send_signal(signal.SIGTERM if i % 2 else signal.SIGINT)
                                process.wait(timeout=12)
                                log.seek(0)
                                self.assertEqual(process.returncode, 0, log.read())
                            finally:
                                if process.poll() is None:
                                    process.kill()
                                    process.wait()
                                node.destroy_subscription(subscriber)
                    producer.send_signal(signal.SIGINT)
                    producer.wait(timeout=8)
                    source_log.seek(0)
                    self.assertEqual(producer.returncode, 0, source_log.read())
        finally:
            if producer is not None and producer.poll() is None:
                producer.kill()
                producer.wait()
            executor.shutdown()
            node.destroy_node()
            context.shutdown()
