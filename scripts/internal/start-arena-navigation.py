#!/usr/bin/env python3
"""Confirm startup inputs and service replies before requesting Nav2 activation."""

import json
import time
from pathlib import Path
import rclpy
from lifecycle_msgs.srv import GetState
from nav2_msgs.srv import ManageLifecycleNodes
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from rclpy.qos import qos_profile_sensor_data

root = Path(__file__).resolve().parents[2]
c = json.loads((root / "config/arena.json").read_text())
ns = "/" + c["namespace"]
rclpy.init()
n = rclpy.create_node("arena_startup")
seen = {}
created = time.monotonic()
n.create_subscription(
    Clock,
    "/clock",
    lambda msg: seen.update(clock=msg.clock.sec + msg.clock.nanosec / 1e9),
    qos_profile_sensor_data,
)
n.create_subscription(
    Odometry,
    ns + "/odometry",
    lambda msg: seen.update(odom=time.monotonic()),
    qos_profile_sensor_data,
)


def until(condition, timeout, label):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        rclpy.spin_once(n, timeout_sec=0.05)
        if condition():
            return
    raise RuntimeError("Arena startup timeout: " + label)


def request(client, req, label, timeout=5):
    if not client.wait_for_service(timeout_sec=10):
        raise RuntimeError("Missing service " + label)
    f = client.call_async(req)
    until(f.done, timeout, label)
    return f.result()


try:
    # LifecycleManager creates change_state clients at launch. Allow endpoint
    # discovery to settle; an early Fast DDS response was lost in a recorded run.
    until(
        lambda: time.monotonic() - created >= 3
        and seen.get("clock", 0) > 0
        and time.monotonic() - seen.get("odom", 0) < 0.5,
        40,
        "clock and odometry",
    )
    clients = {
        name: n.create_client(GetState, ns + "/" + name + "/get_state")
        for name in ["map_server", "controller_server", "velocity_smoother"]
    }
    for name, client in clients.items():
        if request(client, GetState.Request(), name).current_state.id != 1:
            raise RuntimeError("Unexpected pre-start lifecycle state " + name)
    manager = n.create_client(
        ManageLifecycleNodes, ns + "/lifecycle_manager_arena/manage_nodes"
    )
    req = ManageLifecycleNodes.Request()
    req.command = 0
    if not request(manager, req, "Nav2 startup", 40).success:
        raise RuntimeError("Nav2 startup not acknowledged")
    for name, client in clients.items():
        if request(client, GetState.Request(), name).current_state.id != 3:
            raise RuntimeError("Nav2 node not active " + name)
    print(
        "PASS arena startup: live clock/odometry, service replies and active Nav2 nodes",
        flush=True,
    )
finally:
    n.destroy_node()
    rclpy.shutdown()
