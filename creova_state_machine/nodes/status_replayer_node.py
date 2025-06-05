#!/usr/bin/env python3
"""
status_replayer_node.py
  • Reads latest_status.txt (default path = $HOME/latest_status.txt)
  • Publishes its contents on /robot_status at `rate_hz` (default 2 Hz)
Use when you need to broadcast the most recent status after a restart.
Antonio: This node is to read the local file and publish it in Domain 30
"""

import pathlib, time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class StatusReplayer(Node):
    def __init__(self):
        super().__init__("status_replayer")

        self.declare_parameter("file_path", str(pathlib.Path.home() / "latest_status.txt"))
        self.declare_parameter("rate_hz",   2.0)
        self.file_path = pathlib.Path(
            self.get_parameter("file_path").get_parameter_value().string_value
        )
        self.period = 1.0 / self.get_parameter("rate_hz").value

        self.pub = self.create_publisher(String, "/robot_status", 10)
        self.create_timer(self.period, self.tick)

        self.get_logger().info(f"✓ Replaying {self.file_path} every {self.period:.2f}s")

    # ----------------------------------------------------------
    def tick(self):
        try:
            data = self.file_path.read_text().strip()
        except FileNotFoundError:
            self.get_logger().warn_once(f"File {self.file_path} not found")
            return

        msg = String(); msg.data = data
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = StatusReplayer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
