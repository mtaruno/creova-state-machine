#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
import json
import random

class ObjectSelectorPublisher(Node):
    def __init__(self):
        super().__init__('object_selector')
        self.publisher_ = self.create_publisher(String, 'task_goal', 10)
        self.file_path = 'detections.json'
        self.task_id = 0

        self.service = self.create_service(Trigger, 'trigger_task', self.handle_trigger)
        self.get_logger().info('ObjectSelector ready. Call /trigger_task to send task.')

    def handle_trigger(self, request, response):
        try:
            with open(self.file_path, 'r') as f:
                data = json.load(f)
        except (FileNotFoundError, json.JSONDecodeError):
            response.success = False
            response.message = 'detections.json not found or invalid'
            return response

        if not data:
            response.success = False
            response.message = 'No objects available'
            return response

        object_name = random.choice(list(data.keys()))
        pos = data[object_name]

        msg = String()
        self.task_id += 1
        msg.data = json.dumps({
            "task_id": self.task_id,
            "x": pos['x'],
            "y": pos['y']
        })

        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')

        response.success = True
        response.message = f'Task {self.task_id} published.'
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ObjectSelectorPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down selector...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
