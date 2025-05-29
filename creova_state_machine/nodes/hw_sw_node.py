#!/usr/bin/env python3
"""
hw_sw_node.py
─────────────
Bridges the hardware switch that detects whether the robot's lid is closed
to a clean ROS topic `/lid_closed` (std_msgs/Bool).

Replace the placeholder subscriber with your real sensor topic/type.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool                # for /lid_closed

# TODO: change this to the real message type you receive from the MCU
from std_msgs.msg import Int8                # e.g. 0=open, 1=closed


class HWSWNode(Node):
    def __init__(self):
        super().__init__("hw_sw_node")

        # publisher: clean boolean state
        self.pub = self.create_publisher(Bool, "/lid_closed", 10)

        # subscriber: raw sensor (edit topic + type!)
        self.create_subscription(
            Int8,                            # <— change if needed
            "/lid_switch_raw",               # <— change if needed
            self._raw_cb,
            10,
        )
        self.get_logger().info("🟢 hw_sw_node up — publishing /lid_closed")

    # ------------------------------------------------------------------
    def _raw_cb(self, msg: Int8):
        closed = bool(msg.data)              # 1 => True; 0 => False
        self.pub.publish(Bool(data=closed))
        # Optional debug
        state = "CLOSED" if closed else "OPEN"
        self.get_logger().info(f"Lid state: {state}")


def main(args=None):
    rclpy.init(args=args)
    node = HWSWNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
