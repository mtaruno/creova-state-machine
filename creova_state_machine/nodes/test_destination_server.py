#!/usr/bin/env python3
"""
physical_ai_node.py
────────────────────
• Subscribes to JSON robot-command messages on a configurable topic
  (default: /robot_command, std_msgs/String).

• Stores the most recently received "destination" field.

• Provides a service /get_destination (std_srvs/Trigger) that
  returns that destination to any client.

---------------------------------------------------------------
JSON schema expected on the command topic:

{
    "id":          <int | str>,
    "object":      "<object_name>",
    "destination": "<pose_or_location>"
}
---------------------------------------------------------------
"""

import json
from typing import Any, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger


class PhysicalAINode(Node):
    """Listens for robot commands and serves the latest destination."""

    def __init__(self) -> None:
        super().__init__("physical_ai_node")

        # ── parameters ────────────────────────────────────────────────
        topic: str = self.declare_parameter("topic", "robot_command").value
        depth: int = self.declare_parameter("queue_depth", 10).value

        # ── internal state ────────────────────────────────────────────
        self._last_destination: Optional[str] = None

        # ── subscriber ───────────────────────────────────────────────
        self.create_subscription(String, topic, self._command_cb, depth)
        self.get_logger().info(f"🟢 Subscribed to '{topic}'")

        # ── service ──────────────────────────────────────────────────
        self.create_service(Trigger, "get_destination", self._srv_cb)
        self.get_logger().info("🟢 Service ready: /get_destination")

    # ------------------------------------------------------------------
    # callback: handle each incoming command
    # ------------------------------------------------------------------
    def _command_cb(self, msg: String) -> None:
        try:
            data: dict[str, Any] = json.loads(msg.data)
            dest = data.get("destination")
            self._last_destination = dest
            self.get_logger().info(f"🔔 Stored destination = {dest!r}")
        except Exception as exc:
            self.get_logger().error(
                f"⚠️  Invalid JSON: {msg.data!r} (error: {exc})"
            )

    # ------------------------------------------------------------------
    # callback: handle /get_destination service requests
    # ------------------------------------------------------------------
    def _srv_cb(self, _: Trigger.Request, resp: Trigger.Response):
        if self._last_destination is not None:
            resp.success = True
            resp.message = str(self._last_destination)
        else:
            resp.success = False
            resp.message = ""
        return resp


# ─────────────────────────────────────────────────────────────────────
def main(args=None) -> None:
    rclpy.init(args=args)
    node = PhysicalAINode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down physical_ai_node")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
