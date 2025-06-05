#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trans_msgs.msg import FinalDetection
import json
import os

class PerceptionRecorder(Node):
    def __init__(self):
        super().__init__('perception_recorder')
        self.subscription = self.create_subscription(
            FinalDetection,
            '/final/xyz',
            self.listener_callback,
            10
        )
        self.detections = {}
        self.file_path = 'detections.json'
        self.get_logger().info('Recording detections to local file.')

    def listener_callback(self, msg):
        self.detections[msg.object_name] = {
            'x': msg.x,
            'y': msg.y,
            'z': msg.z
        }

        with open(self.file_path, 'w') as f:
            json.dump(self.detections, f, indent=2)

        self.get_logger().info(f'Updated {msg.object_name} → ({msg.x:.2f}, {msg.y:.2f}, {msg.z:.2f})')

def main(args=None):
    rclpy.init(args=args)
    node = PerceptionRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Recorder node interrupted.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':

    main()

