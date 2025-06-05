#!/usr/bin/env python3
"""
destination_server_node.py
• Subscribes to /latest_destination (std_msgs/String)
• Offers     /get_destination    (std_srvs/Trigger)
• Waits DELAY_SECS after every new /latest_destination before releasing it
"""

from typing import Optional
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from std_srvs.srv import Trigger


DELAY_SECS = 90     # hold time before the destination is handed out


class DestinationServer(Node):
    def __init__(self) -> None:
        super().__init__("destination_server_node")

        self._latest_dest: Optional[str] = None
        self._dest_stamp: Optional[Time] = None  # ROS time when message arrived

        self.create_subscription(String, "/latest_destination", self._dest_cb, 10)
        self.create_service(Trigger, "get_destination", self._srv_cb)

        self.get_logger().info(
            f"Service ready: /get_destination (releases after {DELAY_SECS}s)"
        )

    # ------------------------------------------------------------------ #
    # callback: /latest_destination                                      #
    # ------------------------------------------------------------------ #
    def _dest_cb(self, msg: String) -> None:
        self._latest_dest = msg.data or None
        self._dest_stamp = self.get_clock().now()  # ROS time (respects /clock)
        self.get_logger().info(f"Cached destination → {self._latest_dest!r}")

    # ------------------------------------------------------------------ #
    # service: /get_destination                                          #
    # ------------------------------------------------------------------ #
    def _srv_cb(self, _req: Trigger.Request, res: Trigger.Response):
        if self._latest_dest is None:
            res.success, res.message = False, "no destination stored"
            return res

        # how long ago was the destination received?
        elapsed = (self.get_clock().now() - self._dest_stamp).nanoseconds / 1e9
        if elapsed < DELAY_SECS:
            remaining = int(DELAY_SECS - elapsed)
            res.success = False
            res.message = f"destination not ready ({remaining}s remaining)"
        else:
            res.success = True
            res.message = self._latest_dest
            # Optionally clear it so it can only be fetched once:
            # self._latest_dest = None
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
