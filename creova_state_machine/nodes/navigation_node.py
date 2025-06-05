#!/usr/bin/env python3
"""
nav_status_recorder.py
----------------------
Listens to /nav2_status (c8nav/msg/Nav2Status) and stores the latest
status in nav2_status.json.
"""

import json
import rclpy
from rclpy.node import Node
from c8nav.msg import Nav2Status

FILE_PATH = "nav2_status.json"

class NavStatusRecorder(Node):
    def __init__(self):
        super().__init__("nav_status_recorder")

        self.create_subscription(
            Nav2Status,
            "/nav2_status",
            self.status_cb,
            10
        )
        self.get_logger().info("Recording /nav2_status → nav2_status.json")

    def status_cb(self, msg: Nav2Status):
        data = {
            "ready": msg.ready,
            "x": msg.position.x,
            "y": msg.position.y,
            "distance_to_goal": msg.distance_to_goal,
            "eta": msg.estimated_time_to_goal
        }
        try:
            with open(FILE_PATH, "w") as f:
                json.dump(data, f)
        except Exception as e:
            self.get_logger().error(f"Failed to write {FILE_PATH}: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = NavStatusRecorder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

