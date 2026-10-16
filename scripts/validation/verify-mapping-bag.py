#!/usr/bin/env python3
"""Check a completed site-mapping bag before using it to build a map."""
import argparse
import json
import math
from pathlib import Path
import sqlite3

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    bag, output = Path(args.bag).resolve(), Path(args.output).resolve()
    if output.exists():
        parser.error("Choose a new report file; existing results are not overwritten")
    if not (bag / "metadata.yaml").is_file():
        parser.error("Bag has no metadata.yaml; stop recording normally first")
    required = {
        "/livox/lidar": (8, .3),
        "/livox/imu": (180, .1),
        "/intramural_robot/cloud_registered": (8, .3),
        "/intramural_robot/aft_mapped_to_init": (8, .3),
    }
    for db in bag.glob("*.db3"):
        with sqlite3.connect(db.as_uri() + "?mode=ro", uri=True) as connection:
            if connection.execute("PRAGMA quick_check").fetchall() != [("ok",)]:
                raise ValueError("SQLite integrity check failed: " + str(db))
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3"), rosbag2_py.ConverterOptions("", ""))
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types() if t.name in required}
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(required)))
    stats = {}
    while reader.has_next():
        topic, raw, received = reader.read_next()
        msg = deserialize_message(raw, types[topic])
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        s = stats.setdefault(topic, {"count": 0, "first": stamp, "last": stamp, "max_gap_s": 0,
                                     "backwards": 0, "invalid": 0, "frames": set()})
        s["count"] += 1
        s["backwards"] += int(stamp < s["last"])
        s["max_gap_s"] = max(s["max_gap_s"], stamp - s["last"])
        s["last"] = stamp
        s["frames"].add(msg.header.frame_id)
        if topic.endswith("cloud_registered"):
            if msg.header.frame_id != "camera_init" or msg.is_bigendian:
                raise ValueError("Unexpected registered cloud frame/byte order")
            fields = {f.name: f for f in msg.fields}
            for axis in "xyz":
                if fields[axis].datatype != 7:
                    raise ValueError("Expected float32 XYZ")
                values = np.ndarray((msg.height, msg.width), dtype="<f4", buffer=msg.data,
                                    offset=fields[axis].offset, strides=(msg.row_step, msg.point_step))
                s["invalid"] += int(np.count_nonzero(~np.isfinite(values)))
        elif topic.endswith("aft_mapped_to_init"):
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            s["invalid"] += int(not all(math.isfinite(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w)))
            s["invalid"] += int(not .9 < q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w < 1.1)
        elif topic == "/livox/imu":
            a, w = msg.linear_acceleration, msg.angular_velocity
            s["invalid"] += int(not all(math.isfinite(v) for v in (a.x, a.y, a.z, w.x, w.y, w.z)))
        else:
            s["invalid"] += int(msg.point_num == 0 or msg.point_num != len(msg.points))
    errors = []
    for topic, (min_hz, max_gap) in required.items():
        if topic not in stats:
            errors.append("Missing " + topic)
            continue
        s = stats[topic]
        duration = s["last"] - s["first"]
        s.update(duration_s=duration, hz=(s["count"]-1)/duration if duration > 0 else 0,
                 frames=sorted(s["frames"]))
        if duration < 2 or s["hz"] < min_hz or s["max_gap_s"] > max_gap or s["backwards"] or s["invalid"]:
            errors.append("Stream continuity/data check failed: " + topic)
    if len(stats) == len(required):
        if max(s["first"] for s in stats.values()) - min(s["first"] for s in stats.values()) > .5:
            errors.append("Sensor stream start times differ by more than 0.5 s")
        if max(s["last"] for s in stats.values()) - min(s["last"] for s in stats.values()) > .5:
            errors.append("Sensor stream end times differ by more than 0.5 s")
    report = {"bag": str(bag), "passed": not errors, "streams": stats, "errors": errors,
              "independent_precision_test": False}
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    if errors:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
