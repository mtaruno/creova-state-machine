#!/usr/bin/env python3
"""
nav_ready_trigger.py
--------------------
Reads nav2_status.json every 0.5 s.  When `ready` flips False→True,
calls /trigger_task (std_srvs/srv/Trigger).
"""

import json
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from pathlib import Path

FILE_PATH = Path("nav2_status.json")

class NavReadyTrigger(Node):
    def __init__(self):
        super().__init__("nav_ready_trigger")

        self.trigger_cli = self.create_client(Trigger, "/trigger_task")
        self._prev_ready = False

        # Poll file twice a second
        self.create_timer(0.5, self.check_status_file)
        self.get_logger().info("NavReadyTrigger polling nav2_status.json …")

    # ----------------------------------------------------------
    #  Timer callback
    # ----------------------------------------------------------
    def check_status_file(self):
        if not FILE_PATH.exists():
            return

        try:
            with open(FILE_PATH, "r") as f:
                data = json.load(f)
        except Exception:
            return  # ignore malformed file until next poll

        ready_now = bool(data.get("ready", False))

        # Rising edge detection
        if ready_now and not self._prev_ready:
            self.call_trigger_service()

        self._prev_ready = ready_now

    # ----------------------------------------------------------
    #  Service helpers
    # ----------------------------------------------------------
    def call_trigger_service(self):
        if not self.trigger_cli.wait_for_service(timeout_sec=0.0):
            self.get_logger().warning("/trigger_task service not available")
            return

        self.get_logger().info("Ready=True → calling /trigger_task …")
        fut = self.trigger_cli.call_async(Trigger.Request())
        fut.add_done_callback(self.handle_response)

    def handle_response(self, future):
        try:
            resp = future.result()
            if resp.success:
                self.get_logger().info(f"Task triggered: {resp.message}")
            else:
                self.get_logger().warning(f"Trigger failed: {resp.message}")
        except Exception as e:
            self.get_logger().error(f"Service call error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = NavReadyTrigger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
