#!/usr/bin/env python3
"""
Mock Perception Node - Simulates perception functionality for testing
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time

class MockPerceptionNode(Node):
    def __init__(self):
        super().__init__('mock_perception_node')
        
        # Publishers
        self.object_details_pub = self.create_publisher(String, '/perception/object_details', 10)
        
        # Subscribers
        self.pick_request_sub = self.create_subscription(
            String, '/perception/pick_request', self.handle_pick_request, 10)
        
        # Mock object database
        self.object_database = {
            'apple': {
                'name': 'apple',
                'pose': {
                    'position': {'x': 0.5, 'y': 0.2, 'z': 0.1},
                    'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
                },
                'confidence': 0.95
            },
            'banana': {
                'name': 'banana',
                'pose': {
                    'position': {'x': 0.3, 'y': -0.1, 'z': 0.1},
                    'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
                },
                'confidence': 0.92
            },
            'orange': {
                'name': 'orange',
                'pose': {
                    'position': {'x': 0.7, 'y': 0.0, 'z': 0.1},
                    'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
                },
                'confidence': 0.88
            }
        }
        
        self.get_logger().info('Mock Perception Node initialized')
        
    def handle_pick_request(self, msg: String):
        """Handle pick request from orchestration node."""
        object_name = msg.data
        self.get_logger().info(f"Received pick request for: {object_name}")
        
        # Simulate processing time
        def respond_with_delay():
            time.sleep(1.0)  # Simulate perception processing time
            
            if object_name in self.object_database:
                # Object found, send details
                object_data = self.object_database[object_name]
                response_msg = String()
                response_msg.data = json.dumps(object_data)
                self.object_details_pub.publish(response_msg)
                self.get_logger().info(f"Sent object details for: {object_name}")
            else:
                # Object not found
                self.get_logger().warn(f"Object not found in database: {object_name}")
                # Could send a "not found" response here if needed
        
        # Run in separate thread to avoid blocking
        import threading
        threading.Thread(target=respond_with_delay).start()

def main(args=None):
    rclpy.init(args=args)
    node = MockPerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
