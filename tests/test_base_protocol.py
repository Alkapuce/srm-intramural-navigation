"""Document vectors and defensive serial stream handling."""

import math
import struct
import unittest

from intramural_base.protocol import (
    crc8, crc16, command_frame, decode_command, feedback_frame, FeedbackParser,
)
from intramural_base.transport import CommandPolicy, validate_vehicle_config


class ProtocolTests(unittest.TestCase):
    def test_document_crc_and_three_command_vectors(self):
        self.assertEqual(crc8(b"123456789"), 0x0B)
        self.assertEqual(crc16(b"123456789"), 0x6F91)
        cases = [
            ((1, 0, 0), "A5 0C 00 01 26 00 00 80 3F 00 00 00 00 00 00 00 00 6B A3"),
            ((0, 0, 0), "A5 0C 00 01 26 00 00 00 00 00 00 00 00 00 00 00 00 18 9F"),
            ((1, -1, 3), "A5 0C 00 01 26 00 00 80 3F 00 00 80 BF 00 00 40 40 D3 96"),
        ]
        for values, hex_frame in cases:
            with self.subTest(values=values):
                frame = command_frame(*values, 1)
                self.assertEqual(frame, bytes.fromhex(hex_frame))
                self.assertEqual(decode_command(frame), (1, tuple(float(v) for v in values)))

    def test_nonfinite_and_corrupt_command_rejected(self):
        with self.assertRaises(ValueError):
            command_frame(math.nan, 0, 0, 0)
        frame = bytearray(command_frame(0, 0, 0, 0))
        frame[8] ^= 1
        with self.assertRaises(ValueError):
            decode_command(frame)
        with self.assertRaises(ValueError):
            decode_command(bytes(frame) + b"0")

    def test_document_feedback_half_noise_crc_and_resync(self):
        raw = bytes.fromhex("A5 0B 00 01 5C 01 00 AA 2C 01 88 77 66 55 44 33 22 11 EF 74")
        bad = bytearray(raw)
        bad[-1] ^= 0x80
        invalid = feedback_frame(1, b"bad", 2)
        parser = FeedbackParser()
        self.assertEqual(parser.feed(b"noise" + raw[:8]), [])
        packets = parser.feed(raw[8:] + bad + invalid + raw)
        self.assertEqual(len(packets), 2)
        self.assertEqual(packets[0].values["stage_remain_time"], 300)
        self.assertEqual(packets[0].values["sync_time_stamp"], 0x1122334455667788)
        self.assertEqual(packets[1].cmd_id, 1)
        self.assertGreater(parser.discarded, 0)

    def test_robot_feedback_and_multiple_frames(self):
        payload = struct.pack("<BBHHHHHB", 7, 2, 100, 200, 3, 4, 5, 0b101)
        frame = feedback_frame(11, payload, 255)
        parser = FeedbackParser()
        packets = parser.feed(frame + frame)
        self.assertEqual([p.seq for p in packets], [255, 255])
        self.assertEqual(packets[0].values["robot_id"], 7)
        self.assertEqual(packets[0].values["power_outputs"], 5)

    def test_parser_bounds_untrusted_length(self):
        p = FeedbackParser()
        self.assertEqual(p.feed(b"\xa5\xff\xff" + b"x" * 10000), [])
        self.assertLessEqual(len(p.buffer), 4096)


class PolicyTests(unittest.TestCase):
    def test_real_port_requires_independent_hardware_verifications(self):
        keys = ("calibrated_for_vehicle", "firmware_protocol_verified", "link_loss_stop_verified")
        confirmed = dict.fromkeys(keys, True)
        validate_vehicle_config(confirmed)
        for key in keys:
            for value in (False, None, "true"):
                with self.subTest(key=key, value=value), self.assertRaisesRegex(ValueError, key):
                    validate_vehicle_config({**confirmed, key: value})
        with self.assertRaises(ValueError):
            validate_vehicle_config({})
        validate_vehicle_config({}, simulation=True)

    def policy(self):
        return CommandPolicy([[0, 2], [-2, 0]], .35, .6, .2, .3)

    def test_axis_mapping_and_protocol_cap(self):
        p = self.policy()
        p.enabled = True
        p.receive(.3, .4, 1, 0)
        p.last_tick = 0
        values, reason = p.tick(.1)
        self.assertEqual(reason, "active")
        self.assertEqual(values[2], 0)
        self.assertLessEqual(max(map(abs, values[:2])), .2)
        self.assertLessEqual(math.hypot(*p.previous), .06 + 1e-9)

    def test_stale_disable_and_invalid_zero_without_ramp(self):
        p = self.policy()
        p.enabled = True
        p.receive(.1, 0, 0, 0)
        p.last_tick = 0
        self.assertNotEqual(p.tick(.1)[0], (0, 0, 0))
        self.assertEqual(p.tick(.41)[0], (0, 0, 0))
        p.receive(float("inf"), 0, 0, .42)
        self.assertEqual(p.tick(.43)[0], (0, 0, 0))
        p.receive(.1, 0, 0, .44)
        p.enabled = False
        self.assertEqual(p.tick(.45)[0], (0, 0, 0))

    def test_invalid_calibration_is_rejected(self):
        for matrix in ([[0, 0], [0, 0]], [[math.nan, 0], [0, 1]], [[1]]):
            with self.subTest(matrix=matrix), self.assertRaises(ValueError):
                CommandPolicy(matrix, .1, .1, .1)


if __name__ == "__main__":
    unittest.main()
