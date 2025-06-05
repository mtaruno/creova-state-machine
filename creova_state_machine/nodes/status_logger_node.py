#!/usr/bin/env python3
"""
status_logger_node.py
  • Subscribes to /robot_status   (std_msgs/String)
  • Prints each change with a timestamp
  • Over-writes a local file      (latest_status.txt) with the newest value
  • Republishes it on /latest_status (std_msgs/String)
  Anotonio: This node is used to record the current kinova arm status to a local file.
"""

import datetime, pathlib, signal, sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

LOGFILE = pathlib.Path.home() / "latest_status.txt"

class StatusLogger(Node):
    def __init__(self):
        super().__init__("status_logger")
        self.last_status: str | None = None

        self.declare_parameter("file_path", str(LOGFILE))
        self.file_path = pathlib.Path(
            self.get_parameter("file_path").get_parameter_value().string_value
        )

        self.create_subscription(String, "/robot_status",
                                 self.cb_status, 10)
        self.pub_latest = self.create_publisher(String, "/latest_status", 10)

        self.get_logger().info(f"✓ Logging to {self.file_path}")

    # --------------------------------------------------------------
    def cb_status(self, msg: String):
        if msg.data == self.last_status:          # ignore repeats
            return

        ts = datetime.datetime.now().strftime("%H:%M:%S")
        print(f"[{ts}] status = {msg.data}")
        self.last_status = msg.data

        # over-write the file with only the latest value
        try:
            self.file_path.write_text(msg.data + "\n")
        except Exception as e:
            self.get_logger().warn(f"Could not write file: {e}")

        # republish on /latest_status
        self.pub_latest.publish(msg)

def main():
    rclpy.init()
    node = StatusLogger()

    signal.signal(signal.SIGINT, lambda *_: (rclpy.shutdown(), sys.exit(0)))
    rclpy.spin(node)

if __name__ == "__main__":
    main()
