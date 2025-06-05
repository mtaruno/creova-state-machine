#!/usr/bin/env python3
"""
Mock Object List Publisher - Simulates perception team's object list
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time

class MockObjectListPublisher(Node):
    def __init__(self):
        super().__init__('mock_object_list_publisher')
        
        # Publisher for object list
        self.object_list_pub = self.create_publisher(String, '/perception/object_list', 10)
        
        # Mock object database
        self.objects = [
            {"name": "apple", "x": 0.5, "y": 0.2, "z": 0.1},
            {"name": "banana", "x": 0.3, "y": -0.1, "z": 0.1},
            {"name": "orange", "x": 0.7, "y": 0.0, "z": 0.1},
            {"name": "cup", "x": 0.2, "y": 0.3, "z": 0.05},
            {"name": "bottle", "x": 0.6, "y": -0.2, "z": 0.15}
        ]
        
        # Timer to publish object list every 2 seconds
        self.timer = self.create_timer(2.0, self.publish_object_list)
        
        self.get_logger().info('Mock Object List Publisher initialized - publishing every 2 seconds')
        
    def publish_object_list(self):
        """Publish the current object list."""
        msg = String()
        msg.data = json.dumps(self.objects)
        self.object_list_pub.publish(msg)
        self.get_logger().info(f'Published object list with {len(self.objects)} objects')

def main(args=None):
    rclpy.init(args=args)
    node = MockObjectListPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
