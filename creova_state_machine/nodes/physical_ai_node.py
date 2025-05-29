#!/usr/bin/env python3
"""
physical_ai_node.py  (listener / broadcaster)

• Subscribes: /robot_command  (std_msgs/String, JSON payload)
• Publishes : /latest_destination (std_msgs/String)
              /latest_object      (std_msgs/String)
              /latest_task_id     (std_msgs/String)

JSON schema expected on /robot_command:
{
    "id":          <int | str>,
    "object":      "<object_name>",
    "destination": "<pose_or_location>"
}
"""

import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class PhysicalAINode(Node):
    def __init__(self) -> None:
        super().__init__("physical_ai_node")

        # ── parameters ────────────────────────────────────────────────
        topic  = self.declare_parameter("topic", "robot_command").value
        depth  = self.declare_parameter("queue_depth", 10).value

        # ── publishers ────────────────────────────────────────────────
        self.dest_pub = self.create_publisher(String, "/latest_destination", 10)
        self.obj_pub  = self.create_publisher(String, "/latest_object",      10)
        self.id_pub   = self.create_publisher(String, "/latest_task_id",     10)

        # ── subscriber ────────────────────────────────────────────────
        self.create_subscription(String, topic, self._cmd_cb, depth)

        self.get_logger().info(
            f"🟢 physical_ai_node listening on '{topic}' and publishing helper topics"
        )

    # ------------------------------------------------------------------
    def _cmd_cb(self, msg: String) -> None:
        """Parse JSON and broadcast individual fields."""
        try:
            data = json.loads(msg.data)

            dest = str(data.get("destination", ""))
            obj  = str(data.get("object",      ""))
            task = str(data.get("id",          ""))

            # Publish each field
            self.dest_pub.publish(String(data=dest))
            self.obj_pub.publish(String(data=obj))
            self.id_pub.publish(String(data=task))

            self.get_logger().info(
                f"Updated → dest={dest!r}, object={obj!r}, id={task!r}"
            )

        except Exception as exc:
            self.get_logger().error(f"⚠️  Bad JSON: {exc} – raw={msg.data!r}")


# ─────────────────────────────────────────────────────────────────────
def main(args=None) -> None:
    rclpy.init(args=args)
    node = PhysicalAINode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
