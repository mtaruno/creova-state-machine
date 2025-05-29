#!/usr/bin/env python3
"""
destination_server_node.py
• Subscribes to /latest_destination (std_msgs/String)
• Offers  /get_destination (std_srvs/Trigger)
"""

from typing import Optional
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger


class DestinationServer(Node):
    def __init__(self) -> None:
        super().__init__("destination_server_node")

        self._latest_dest: Optional[str] = None

        self.create_subscription(String, "/latest_destination", self._dest_cb, 10)
        self.create_service(Trigger, "get_destination", self._srv_cb)

        self.get_logger().info("Service ready: /get_destination")

    # ------------------------------------------------------------------
    def _dest_cb(self, msg: String) -> None:
        self._latest_dest = msg.data or None
        self.get_logger().info(f"Cached destination → {self._latest_dest!r}")

    # ------------------------------------------------------------------
    def _srv_cb(self, _req: Trigger.Request, res: Trigger.Response):
        if self._latest_dest is None:
            res.success, res.message = False, "no destination stored"
        else:
            res.success, res.message = True, self._latest_dest
        return res


def main(args=None):
    rclpy.init(args=args)
    node = DestinationServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

