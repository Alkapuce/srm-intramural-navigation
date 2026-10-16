"""Exercise the commissioning CLI against a PTY MCU, never a real serial device."""
import json
import os
from pathlib import Path
import pty
import select
import signal
import struct
import subprocess
import sys
import tempfile
import time
import unittest

from intramural_base.protocol import crc8, crc16, feedback_frame


class VehiclePulseTests(unittest.TestCase):
    def exercise(self, legacy=False, interrupt=False, feedback=True, cli=False, value=-.02):
        root = Path(__file__).resolve().parents[1]
        evidence = root / "artifacts" / "tests" / "vehicle-pulse"
        evidence.mkdir(parents=True, exist_ok=True)
        output = Path(tempfile.mkdtemp(dir=evidence)) / "result"
        master, slave = pty.openpty()
        command = [sys.executable, str(root / "scripts/validation/vehicle-pulse.py"),
                   "--device", os.ttyname(slave), "--output", str(output),
                   "--axis", "y", "--value", str(value), "--seconds", "0.3",
                   "--wheels-lifted", "--stop-available"]
        if legacy:
            command.append("--legacy20")
        if cli:
            command = [str(root / "scripts/srm-pulse"), "y", *command[2:]]
            value_index = command.index("--axis")
            del command[value_index:value_index + 2]
        process = subprocess.Popen(command, cwd=output.parent, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        captured = bytearray()
        frames = []
        buffer = bytearray()
        interrupted = False
        try:
            deadline = time.monotonic() + 15
            while True:
                readable, _, _ = select.select([master], [], [], .02)
                if readable:
                    data = os.read(master, 4096)
                    captured.extend(data)
                    buffer.extend(data)
                    while len(buffer) >= 5:
                        size = int.from_bytes(buffer[1:3], "little") + 7
                        self.assertIn(size, (19, 20))
                        if len(buffer) < size:
                            break
                        frame = bytes(buffer[:size])
                        del buffer[:size]
                        self.assertEqual(frame[0], 0xA5)
                        self.assertEqual(frame[4], crc8(frame[:4]))
                        self.assertEqual(int.from_bytes(frame[-2:], "little"), crc16(frame[:-2]))
                        frames.append(frame)
                        if feedback:
                            os.write(master, feedback_frame(1, bytes(11), frame[3]))
                        if interrupt and not interrupted and any(struct.unpack_from("<fff", frame, 5)):
                            process.send_signal(signal.SIGTERM)
                            interrupted = True
                if process.poll() is not None and not readable:
                    break
                if time.monotonic() > deadline:
                    self.fail("Pulse CLI did not finish")
            log = process.communicate(timeout=2)[0]
            (output.parent / "process.log").write_bytes(log)
            (output.parent / "wire.bin").write_bytes(captured)
            report = json.loads((output / "result.json").read_text())
            self.assertEqual(process.returncode, 1 if interrupt or not feedback else 0, log.decode())
            self.assertFalse(buffer)
            self.assertEqual(report["command_bytes"], 20 if legacy else 19)
            self.assertEqual(report["protocol"], "legacy20" if legacy else "competition19")
            self.assertEqual(report["transmitted_frames"], len(frames))
            self.assertEqual([f.hex() for f in frames], [t["hex"] for t in report["tx"]])
            self.assertEqual([f[3] for f in frames], [(i + int(legacy)) % 256 for i in range(len(frames))])
            self.assertTrue(all(len(f) == (20 if legacy else 19) for f in frames))
            if legacy:
                self.assertTrue(all(f[17] == 0 for f in frames))
            values = [struct.unpack_from("<fff", f, 5) for f in frames]
            nonzero = [v for v in values if any(v)]
            if feedback:
                self.assertGreater(len(nonzero), 0)
                self.assertLessEqual(len(nonzero), 61 if legacy else 31)
                if legacy and not interrupt:
                    self.assertGreater(len(nonzero), 40)
                    times = [t["time"] for t in report["tx"] if any(t["values"])]
                    self.assertTrue(all(b-a >= .0049 for a,b in zip(times,times[1:])))
                for vx, vy, wz in nonzero:
                    self.assertEqual((vx, wz), (0, 0))
                    self.assertAlmostEqual(vy, value, places=6)
            else:
                self.assertFalse(nonzero)
            self.assertTrue(all(not any(v) for v in values[:100]))
            self.assertTrue(all(not any(v) for v in values[-10:]))
            self.assertTrue(report["port_closed"])
        finally:
            if process.poll() is None:
                process.kill()
            process.communicate()
            os.close(master)
            os.close(slave)

    def test_competition_default(self):
        self.exercise(value=.1)

    def test_legacy20_pulse_and_zero_cleanup(self):
        self.exercise(legacy=True, cli=True)

    def test_legacy20_sigterm_returns_to_zero(self):
        self.exercise(legacy=True, interrupt=True)

    def test_legacy20_without_feedback_never_pulses(self):
        self.exercise(legacy=True, feedback=False)


if __name__ == "__main__":
    unittest.main()
