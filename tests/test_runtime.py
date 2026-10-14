"""Signals must finish native message conversion and keep ROS alive for cleanup."""

import json
import os
import signal
import subprocess
import sys
import time
import unittest

import rclpy
from rclpy.context import Context
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as RosPath

WORKER = '''
import json, sys, rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from intramural_base.runtime import run
class Probe(Node):
    def __init__(self):
        super().__init__('runtime_signal_worker')
        self.poses = 0
        self.create_subscription(Path,sys.argv[1],self.receive,10)
    def receive(self,msg):
        self.poses = len(msg.poses)
run(Probe,lambda node:print(json.dumps({'alive':rclpy.ok(),'poses':node.poses}),flush=True))
'''


class RuntimeTests(unittest.TestCase):
    def test_shutdown_finishes_message_before_releasing_context(self):
        context = Context()
        rclpy.init(context=context, domain_id=77)
        node = rclpy.create_node('runtime_signal_driver', context=context)
        try:
            for sig in (signal.SIGINT, signal.SIGTERM):
                with self.subTest(signal=sig):
                    topic = '/runtime_signal_' + str(int(sig))
                    pub = node.create_publisher(RosPath, topic, 10)
                    process = subprocess.Popen([sys.executable,'-c',WORKER,topic],
                        env=dict(os.environ,ROS_DOMAIN_ID='77'),stdout=subprocess.PIPE,
                        stderr=subprocess.STDOUT,text=True)
                    try:
                        end = time.monotonic()+10
                        while not pub.get_subscription_count() and time.monotonic()<end:
                            time.sleep(.01)
                        self.assertTrue(pub.get_subscription_count())
                        msg = RosPath()
                        msg.poses = [PoseStamped()]*20000
                        pub.publish(msg)
                        time.sleep(.004)
                        process.send_signal(sig)
                        output,_ = process.communicate(timeout=8)
                        self.assertEqual(process.returncode,0,output)
                        self.assertEqual(json.loads(output),{'alive':True,'poses':20000})
                    finally:
                        if process.poll() is None:
                            process.kill()
                            process.wait()
                        node.destroy_publisher(pub)
        finally:
            node.destroy_node()
            context.shutdown()
