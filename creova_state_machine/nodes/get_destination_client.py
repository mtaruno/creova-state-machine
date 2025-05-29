#!/usr/bin/env python3
"""
get_destination_client.py
─────────────────────────
Queries the /get_destination service exposed by physical_ai_node.py
and prints the most-recently stored destination string.
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class DestinationClient(Node):
    def __init__(self):
        super().__init__("destination_client")
        self.cli = self.create_client(Trigger, "get_destination")
        self.get_logger().info("⏳ Waiting for /get_destination …")
        self.cli.wait_for_service()
        self.get_logger().info("🟢 Service available")

    def request_destination(self):
        req = Trigger.Request()
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        resp = future.result()

        if resp is None:
            self.get_logger().error("Service call failed")
        elif resp.success:
            self.get_logger().info(f"Latest destination = {resp.message!r}")
        else:
            self.get_logger().warn("No destination stored yet")


def main(args=None):
    rclpy.init(args=args)
    node = DestinationClient()
    node.request_destination()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

