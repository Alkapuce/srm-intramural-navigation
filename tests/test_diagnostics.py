"""Native ROS diagnostics remain independent even when their publisher process stalls."""
import os
import signal
import time
import unittest
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from nav_msgs.msg import Path
from intramural_mission.diagnostics import Diagnostics


class DiagnosticsTests(unittest.TestCase):
    def setUp(self):
        rclpy.init(args=[], domain_id=89)
        self.node=rclpy.create_node('diagnostics_acceptance',namespace='/diagnostics_test')

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def test_stalled_native_publisher_does_not_block_control_and_delivers_latest_snapshots(self):
        states,paths,ticks=[],[],[]
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(String,'mission/status',lambda m:states.append(m.data),qos)
        self.node.create_subscription(Path,'mission/path',paths.append,qos)
        self.node.create_timer(.005,lambda:ticks.append(time.monotonic()))
        diagnostics=Diagnostics(self.node.get_namespace(),89)
        try:
            self.assertTrue(diagnostics.ready.wait(5.))
            os.kill(diagnostics.process.pid,signal.SIGSTOP)
            os.waitid(os.P_PID,diagnostics.process.pid,os.WSTOPPED|os.WNOWAIT)
            begin=time.monotonic()
            for index in range(60):
                diagnostics.publish('mission/status',String(data=str(index)+'x'*200000))
                diagnostics.publish('mission/path',Path())
            diagnostics.publish('mission/status',String(data='final'))
            self.assertLess(time.monotonic()-begin,.2)
            before=len(ticks);deadline=time.monotonic()+.1
            while time.monotonic()<deadline:rclpy.spin_once(self.node,timeout_sec=.005)
            self.assertGreater(len(ticks)-before,5)
            os.kill(diagnostics.process.pid,signal.SIGCONT)
            deadline=time.monotonic()+5.
            while ('final'not in states or not paths)and time.monotonic()<deadline:
                rclpy.spin_once(self.node,timeout_sec=.02)
            self.assertIn('final',states)
            self.assertTrue(paths)
        finally:
            if diagnostics.process.is_alive():os.kill(diagnostics.process.pid,signal.SIGCONT)
            diagnostics.close()
        self.assertFalse(diagnostics.thread.is_alive())
        self.assertEqual(diagnostics.process.exitcode,0)
        with self.assertRaises(RuntimeError):
            diagnostics.publish('mission/status',String(data='after-close'))

    def test_native_publisher_failure_is_reported_to_control_and_shutdown(self):
        diagnostics=Diagnostics(self.node.get_namespace(),89)
        self.assertTrue(diagnostics.ready.wait(5.))
        diagnostics.process.terminate();diagnostics.process.join(2.)
        try:
            with self.assertRaisesRegex(RuntimeError,'Diagnostic publisher exited'):
                diagnostics.check()
        finally:
            with self.assertRaises(RuntimeError):diagnostics.close()
        self.assertFalse(diagnostics.thread.is_alive())
        self.assertFalse(diagnostics.process.is_alive())
