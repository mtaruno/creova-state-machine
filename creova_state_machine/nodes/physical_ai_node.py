#!/usr/bin/env python3
"""
robot_command_subscriber.py
---------------------------
• Subscribes: /robot_command   (std_msgs/String with JSON payload)
• Publishes : /object_name     (std_msgs/String, just the object name)
"""

import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def main() -> None:
    rclpy.init()
    node = Node("robot_command_subscriber")

    # Publisher to send the extracted object name
    obj_pub = node.create_publisher(String, "object_name", 10)

    # ----------------------------------------------------------
    #  Callback for /robot_command
    # ----------------------------------------------------------
    def callback(msg: String) -> None:
        try:
            data = json.loads(msg.data)
            obj = data.get("object", "")

            print(
                "[robot_command_subscriber] Received command: "
                f"id={data.get('id')}, object={obj}, "
                f"destination={data.get('destination')}, raw={msg.data}"
            )

            # Publish the object name (if present) on /object_name
            if obj:
                out_msg = String(data=obj)
                obj_pub.publish(out_msg)
                print(f"[robot_command_subscriber] → published object_name: {obj}")

        except Exception as e:  # noqa: B902
            print(
                "[robot_command_subscriber] Invalid message received: "
                f"{msg.data}, error: {e}"
            )

    # Subscription
    node.create_subscription(String, "robot_command", callback, 10)
    print("[robot_command_subscriber] Listening on topic: /robot_command")

    # Spin until Ctrl-C
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down robot_command_subscriber")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

