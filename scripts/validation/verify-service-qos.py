#!/usr/bin/env python3
"""Compile a tiny native probe and check effective Fast DDS service reply QoS."""

import subprocess
from pathlib import Path

source = Path("/work/scripts/validation/service-qos.cpp")
binary = Path("/work/cache/stage34/service-qos")
binary.parent.mkdir(parents=True, exist_ok=True)
includes = [
    "-I" + str(p) for p in Path("/opt/ros/humble/include").iterdir() if p.is_dir()
]
if not binary.exists() or binary.stat().st_mtime < source.stat().st_mtime:
    subprocess.run(
        [
            "g++",
            "-std=c++17",
            str(source),
            *includes,
            "-L/opt/ros/humble/lib",
            "-Wl,-rpath,/opt/ros/humble/lib",
            "-lrclcpp",
            "-ltracetools",
            "-lrosidl_runtime_c",
            "-llibstatistics_collector",
            "-lstatistics_msgs__rosidl_typesupport_cpp",
            "-lrcutils",
            "-lrmw",
            "-lrcl",
            "-lstd_srvs__rosidl_typesupport_cpp",
            "-lrmw_fastrtps_cpp",
            "-lfastrtps",
            "-o",
            str(binary),
        ],
        check=True,
    )
subprocess.run([str(binary)], check=True, timeout=10)
