#!/usr/bin/env python3
"""
physical_ai_node.py  (listener / broadcaster)
 
• Subscribes: /robot_command  (std_msgs/String, JSON payload)
• Publishes : /latest_destination (std_msgs/String)
              /latest_object      (std_msgs/String)
              /latest_task_id     (std_msgs/String)
 
Expected JSON payload:
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
 
        # Subscription to /robot_command
        self.create_subscription(String, 'robot_command', self._cmd_cb, 10)
 
        # Publishers for individual fields
        self.dest_pub = self.create_publisher(String, "/latest_destination", 10)
        self.obj_pub  = self.create_publisher(String, "/latest_object", 10)
        self.id_pub   = self.create_publisher(String, "/latest_task_id", 10)
 
        print('[physical_ai_node] Listening on topic: robot_command')
 
    def _cmd_cb(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
 
            dest = str(data.get("destination", ""))
            obj  = str(data.get("object", ""))
            task = str(data.get("id", ""))
 
            print(f'[physical_ai_node] Received command:\n'
                  f'id={task}, object={obj}, destination={dest}, raw={msg.data}')
 
            # Publish fields
            self.dest_pub.publish(String(data=dest))
            self.obj_pub.publish(String(data=obj))
            self.id_pub.publish(String(data=task))
 
        except Exception as e:
            print(f'[physical_ai_node] Received invalid message: {msg.data}, error: {e}')
 
 
def main(args=None) -> None:
    rclpy.init(args=args)
    node = PhysicalAINode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('[physical_ai_node] Shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()
 
