"""Check actual serialized schema and instantaneous-time semantics."""

import struct
import unittest
from types import SimpleNamespace
from unittest.mock import Mock
from sensor_msgs.msg import Imu, PointCloud2, PointField
from intramural_mission.lidar_adapter import convert, LidarAdapter, MAX_INSTANT_OFFSET_SECONDS


class LidarAdapterTests(unittest.TestCase):
    def test_free_fall_and_unstable_imu_withhold_initial_scans(self):
        adapter = SimpleNamespace(gravity_since=None, gravity_ready=False, pub=Mock())
        msg = Imu()
        for stamp, acceleration in [(0, 0.), (.2, 9.81), (.6, float('nan')),
                                     (.7, 9.81), (1.1, 9.81), (1.3, 9.81)]:
            msg.header.stamp.sec = int(stamp)
            msg.header.stamp.nanosec = int(round((stamp-int(stamp))*1e9))
            msg.linear_acceleration.z = acceleration
            LidarAdapter.receive_imu(adapter, msg)
            LidarAdapter.receive(adapter, self.cloud())
            self.assertEqual(adapter.pub.publish.call_count, int(stamp == 1.3))

    def cloud(self):
        m = PointCloud2()
        m.header.frame_id = "front_mid360"
        m.header.stamp.sec = 42
        m.header.stamp.nanosec = 123
        m.height = 1
        m.width = 3
        m.point_step = 24
        m.row_step = 72
        m.fields = [
            PointField(name=k, offset=off, datatype=typ, count=1)
            for k, off, typ in [
                ("x", 0, 7),
                ("y", 4, 7),
                ("z", 8, 7),
                ("intensity", 12, 7),
                ("ring", 16, 4),
            ]
        ]
        m.data = b"".join(
            struct.pack("<ffffHHf", x, y, z, i, ring, 0, 0)
            for x, y, z, i, ring in [
                (1, 2, 3, 4, 31),
                (float("nan"), 1, 1, 1, 0),
                (1, 1, 1, 1, 32),
            ]
        )
        return m

    def test_valid_point_preserves_axes_ring_intensity_and_stamp(self):
        c = convert(self.cloud())
        self.assertEqual(c.width, 1)
        self.assertEqual(c.header, self.cloud().header)
        x, y, z, i, ring, pad, t = struct.unpack("<ffffHHf", c.data)
        self.assertEqual((x, y, z, i, ring), (1, 2, 3, 4, 31))
        self.assertAlmostEqual(t, MAX_INSTANT_OFFSET_SECONDS / 3, places=14)
        self.assertTrue(c.is_dense)
        self.assertEqual(c.row_step, len(c.data))
        self.assertEqual(
            [(f.name, f.datatype) for f in c.fields],
            [("x", 7), ("y", 7), ("z", 7), ("intensity", 7), ("ring", 4), ("time", 7)],
        )

    def test_bad_schema_or_frame_is_rejected(self):
        for mutation in [
            lambda m: setattr(m.header, "frame_id", "wrong"),
            lambda m: setattr(m, "is_bigendian", True),
            lambda m: setattr(m.fields[4], "datatype", 7),
        ]:
            m = self.cloud()
            mutation(m)
            with self.assertRaises(ValueError):
                convert(m)
