#!/usr/bin/env python3
"""Gracefully transition Nav2 managed nodes while simulation time is still available."""
import rclpy
from nav2_msgs.srv import ManageLifecycleNodes
rclpy.init()
node=rclpy.create_node('stage2_navigation_shutdown')
try:
    for name in ('lifecycle_manager_navigation','lifecycle_manager_localization'):
        client=node.create_client(ManageLifecycleNodes,
            '/red_standard_robot1/'+name+'/manage_nodes')
        if not client.wait_for_service(timeout_sec=3):
            raise RuntimeError(f'{name}: lifecycle shutdown service unavailable')
        request=ManageLifecycleNodes.Request();request.command=ManageLifecycleNodes.Request.SHUTDOWN
        future=client.call_async(request)
        rclpy.spin_until_future_complete(node,future,timeout_sec=15)
        if not future.done() or not future.result() or not future.result().success:
            raise RuntimeError(f'{name}: lifecycle shutdown failed')
        print(f'{name}: lifecycle shutdown acknowledged',flush=True)
        node.destroy_client(client)
finally:
    node.destroy_node();rclpy.shutdown()
