#!/usr/bin/env python3
"""
object_selector.py
------------------
• Sub  : /object_name     (std_msgs/String)   ← comes from robot_command_subscriber
• Svc  : /trigger_task    (std_srvs/Trigger)  ← external nodes call this to send a task
• Pub  : /task_goal       (std_msgs/String)   ← JSON: {task_id, x, y}
"""

import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

class ObjectSelectorPublisher(Node):
    def __init__(self):
        super().__init__('object_selector')

        # Publisher for task_goal
        self.publisher_ = self.create_publisher(String, 'task_goal', 10)

        # Subscriber for the latest object name
        self.latest_obj = None
        self.create_subscription(String, 'object_name', self.obj_callback, 10)

        # Trigger service
        self.create_service(Trigger, 'trigger_task', self.handle_trigger)

        self.file_path = 'detections.json'
        self.task_id   = 0
        self.get_logger().info(
            'ObjectSelector ready. Waiting for /object_name, '
            'call /trigger_task to send task.'
        )

    # ----------------------------------------------------------
    #  Subscription callback
    # ----------------------------------------------------------
    def obj_callback(self, msg: String):
        self.latest_obj = msg.data.strip()
        self.get_logger().info(f'Latest object set to: {self.latest_obj}')

    # ----------------------------------------------------------
    #  Service handler
    # ----------------------------------------------------------
    def handle_trigger(self, request, response):
        # Ensure we have heard an object name first
        if not self.latest_obj:
            response.success = False
            response.message = 'No object name received on /object_name yet.'
            return response

        # Load detections and look up that object
        try:
            with open(self.file_path, 'r') as f:
                data = json.load(f)
        except (FileNotFoundError, json.JSONDecodeError):
            response.success = False
            response.message = 'detections.json not found or invalid.'
            return response

        if self.latest_obj not in data:
            response.success = False
            response.message = f'Object "{self.latest_obj}" not in detections.'
            return response

        pos = data[self.latest_obj]

        # Build and publish the task message
        self.task_id += 1
        out_msg = String()
        out_msg.data = json.dumps({
            "task_id": self.task_id,
            "x": pos['x'],
            "y": pos['y']
        })
        self.publisher_.publish(out_msg)
        self.get_logger().info(f'Published task: {out_msg.data}')

        response.success = True
        response.message = f'Task {self.task_id} for "{self.latest_obj}" published.'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ObjectSelectorPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down object_selector …')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
