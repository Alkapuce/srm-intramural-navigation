"""Exercise the real offline boundary and position-domain identification without hardware."""

import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import os
import threading
import time
from types import SimpleNamespace
import unittest
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts/internal"))
from response_fit import fit_response, stopping_samples


def load(name, file):
    spec = importlib.util.spec_from_file_location(name, ROOT / "scripts/validation" / file)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class ResponseTests(unittest.TestCase):
    def test_current_candidate_response_plan_does_not_bypass_the_vehicle_limit(self):
        import subprocess
        result=subprocess.run([sys.executable,str(ROOT/'scripts/validation/measure-response.py'),
            'plan','--axis','both','--levels','2','--repeats','1'],capture_output=True,text=True,timeout=5)
        self.assertEqual(result.returncode,0,result.stderr)
        schedule=json.loads(result.stdout)['phases']
        self.assertEqual({tuple(row[1])for row in schedule},
            {(0.,0.),(2.,0.),(-2.,0.),(0.,2.),(0.,-2.)})
        measurement=load('measurement_limit','measure-response.py')
        with tempfile.TemporaryDirectory(dir=ROOT/'cache') as folder:
            folder=Path(folder)
            (folder/'base-vehicle.json').write_text(json.dumps(dict(firmware_protocol_verified=True,
                link_loss_stop_verified=True,calibrated_for_vehicle=True,body_to_protocol=[[1,0],[0,1]],
                max_body_speed_mps=1.,max_body_acceleration_mps2=3.5,max_protocol_axis=2.)))
            args=SimpleNamespace(execute=True,stop_available=True,vehicle_config=folder,levels=[2.],
                device=str(folder/'no-serial-device'))
            with self.assertRaises(ValueError):measurement.record(args,folder/'output',schedule)
            self.assertFalse((folder/'output').exists())

    def test_identification_recovers_analytical_response_and_validates_unseen_tail(self):
        delay, tau = .12, .16
        commands = np.column_stack((np.arange(0, 30, .01), np.zeros((3000, 2))))
        observations = np.column_stack((np.arange(0, 29.9, .1), np.zeros((299, 2))))
        matrix = np.array([[1.1, -.1], [.2, .9]])
        events = [(1, 2.4, 0, .3), (4, 5.1, 1, -.2), (7, 8.3, 0, -.3),
                  (10, 11.7, 1, .25), (14, 15.2, 0, .2), (18, 19.6, 1, -.3),
                  (22, 23.4, 0, -.25), (26, 27.3, 1, .2)]
        def step_position(t):
            t = np.maximum(t-delay, 0)
            return t-tau*(1-np.exp(-t/tau))
        for start, end, axis, value in events:
            commands[(commands[:, 0] >= start) & (commands[:, 0] < end), axis+1] = value
            displacement = value * (step_position(observations[:, 0]-start)-step_position(observations[:, 0]-end))
            observations[:, 1:] += displacement[:, None] * matrix[:, axis]
        result, _ = fit_response(commands.tolist(), observations.tolist())
        self.assertTrue(result["eligible_for_model"], result)
        self.assertAlmostEqual(result["response_delay"], delay, delta=.025)
        self.assertAlmostEqual(result["response_tau"], tau, delta=.025)
        self.assertLess(result["validation_rmse_m"], .005)
        np.testing.assert_allclose(result["gain_world_from_input"], matrix, atol=.03)

    def test_plant_queue_and_tail_are_not_instantaneous(self):
        bench = load("benchmark", "compare-controllers.py")
        plant = bench.Plant([0, 0], delay=.1, tau=.2, dt=.05)
        for _ in range(2):
            plant.step([1, 0])
            self.assertEqual(plant.pose[0], 0)
        plant.step([1, 0])
        self.assertAlmostEqual(plant.velocity[0], 1-np.exp(-.25))
        plant.step([0, 0])
        self.assertGreater(plant.velocity[0], 1-np.exp(-.25))
        with self.assertRaises(ValueError):
            plant.step([float("nan"), 0])

    def test_historical_source_clock_reconstruction_deduplicates_commands(self):
        measurement = load("measurement", "measure-response.py")
        with tempfile.TemporaryDirectory() as folder:
            file = Path(folder) / "trace.jsonl"
            row = dict(time=10., commands=[dict(kind="final", time=9.8, xy=[.1, 0])],
                       mission=dict(pose=[1, 2], history=[dict(sim_time=8.)], elapsed_sim_seconds=1.9,
                                    tf_age_sim_seconds=.1))
            file.write_text(json.dumps(row)+"\n"+json.dumps(row)+"\n")
            commands, poses, _ = measurement.extract(file)
            self.assertEqual(len(commands), 1)
            self.assertEqual(len(poses), 1)
            self.assertAlmostEqual(poses[0][0], 9.8)
            row["mission"]["state"] = "FAULT"
            row["time"] = 12.
            file.write_text(json.dumps(row)+"\n")
            _, terminal, _ = measurement.extract(file)
            self.assertAlmostEqual(terminal[0][0], 11.9)
        plan = measurement.phases("both", [.15, .3], 2)
        np.testing.assert_allclose(np.sum([seconds*np.array(u) for seconds, u, _ in plan], axis=0), [0, 0])

    def test_invalid_or_unexcited_data_is_rejected(self):
        samples = [[i*.1, 0., 0.] for i in range(100)]
        with self.assertRaises(ValueError):
            fit_response(samples, samples)

    def test_stop_summary_uses_complete_zero_tail(self):
        bench = load("benchmark_stopping", "compare-controllers.py")
        plant = bench.Plant([0, 0], delay=.1, tau=.15)
        commands, poses = [], []
        for i in range(200):
            t = i*.025
            u = [.3, 0] if t < 2 else [0, 0]
            commands.append([t, *u])
            plant.step(u)
            if i % 4 == 0:
                poses.append([t+.025, *plant.pose])
        samples = stopping_samples(commands, poses)
        self.assertEqual(len(samples), 1)
        self.assertGreater(samples[0]["max_displacement_after_zero_m"], .03)
        self.assertLess(samples[0]["max_displacement_after_zero_m"], .12)
        self.assertEqual(stopping_samples(commands[:85], poses[:22]), [])

    def test_ground_recorder_uses_19_byte_pty_and_zeros_on_stale_odometry(self):
        import rclpy
        from rclpy.context import Context
        from rclpy.qos import qos_profile_sensor_data
        from nav_msgs.msg import Odometry
        from intramural_base.protocol import decode_command
        measurement = load("measurement_pty", "measure-response.py")
        master, slave = os.openpty()
        device = os.ttyname(slave)
        os.close(slave)
        os.set_blocking(master, False)
        context = Context()
        rclpy.init(context=context)
        publisher_node = rclpy.create_node("response_fixture", context=context)
        publisher = publisher_node.create_publisher(Odometry, "/response_test/odom", qos_profile_sensor_data)
        finished, frames, buffer = threading.Event(), [], bytearray()
        def fixture():
            start = time.monotonic()
            while not finished.is_set():
                try:
                    buffer.extend(os.read(master, 4096))
                except (BlockingIOError, OSError):
                    pass
                while len(buffer) >= 19:
                    _, values = decode_command(bytes(buffer[:19]))
                    frames.append(values)
                    del buffer[:19]
                if time.monotonic()-start < 1.6:
                    message = Odometry()
                    message.header.stamp = publisher_node.get_clock().now().to_msg()
                    message.header.frame_id = "camera_init"
                    message.pose.pose.orientation.w = 1.
                    publisher.publish(message)
                time.sleep(.01)
        thread = threading.Thread(target=fixture)
        thread.start()
        try:
            with tempfile.TemporaryDirectory() as folder:
                folder = Path(folder)
                (folder/"base-vehicle.json").write_text(json.dumps(dict(
                    calibrated_for_vehicle=True, firmware_protocol_verified=True, link_loss_stop_verified=True,
                    body_to_protocol=[[1, 0], [0, 1]], max_body_speed_mps=.35,
                    max_body_acceleration_mps2=1., max_protocol_axis=.35)))
                args = SimpleNamespace(execute=True, stop_available=True, vehicle_config=folder,
                                       levels=[.15], device=device, odom_topic="/response_test/odom")
                with self.assertRaisesRegex(RuntimeError, "odometry stopped"):
                    measurement.record(args, folder/"record", [(4., [.15, 0.], "test")])
                summary = json.loads((folder/"record/recording.json").read_text())
                self.assertIn("odometry stopped", summary["error"])
                time.sleep(.03)
                self.assertTrue(any(v[0] > 0 for v in frames))
                self.assertTrue(all(v == (0., 0., 0.) for v in frames[-20:]))
        finally:
            finished.set()
            thread.join()
            publisher_node.destroy_node()
            context.shutdown()
            os.close(master)


class PluginBoundaryTests(unittest.TestCase):
    @unittest.skipUnless((ROOT / "cache/controller-benchmark/controller-worker").exists(), "Build with compare-controllers.sh first")
    def test_actual_plugins_generate_finite_nonrotating_commands_and_exit(self):
        bench = load("benchmark_plugins", "compare-controllers.py")
        grid = dict(width=120, height=120, resolution=.05, origin=[-3., -3.], data=[0]*(120*120))
        with tempfile.TemporaryDirectory() as folder:
            for backend in ("pb", "mppi", "mppi-response"):
                with self.subTest(backend=backend):
                    worker = bench.Worker(dict(backend=backend, dt=.025, max_speed=.65, max_acceleration=1.2,
                                              model_delay=.2, model_tau=.15, map=grid), Path(folder) / (backend+".log"))
                    try:
                        commands = []
                        for _ in range(10):
                            result = worker.call(dict(pose=[0, 0], yaw=0., velocity=[0, 0],
                                                      path=[[i*.05, 0] for i in range(31)], speed_cap=.65,
                                                      previous_command=[0, 0], pending_commands=[]))
                            self.assertTrue(np.isfinite(result["command"]).all())
                            commands.append(result["command"][0])
                        self.assertGreater(max(commands), 0)
                    finally:
                        worker.close()


if __name__ == "__main__":
    unittest.main()
