#!/usr/bin/env python3
'''
Test script to demonstrate the state machine system
'''

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped
from custom_msgs.msg import Intent, ObjectRequest, Status
import time

class TestNode(Node):
    def __init__(self):
        super().__init__('test_node')
        
        # Publishers to simulate user input
        self.intent_pub = self.create_publisher(
            Intent,
            '/pai/intent_out',
            10
        )
        
        # Subscribers to monitor system state
        self.perception_sub = self.create_subscription(
            Status,
            '/perception/feedback',
            self.handle_perception_feedback,
            10
        )
        self.manipulation_sub = self.create_subscription(
            Status,
            '/manipulation/feedback',
            self.handle_manipulation_feedback,
            10
        )
        self.navigation_sub = self.create_subscription(
            Status,
            '/navigation/feedback',
            self.handle_navigation_feedback,
            10
        )
        
        self.get_logger().info('Test Node initialized')
        
        # Start the test after a short delay
        self.timer = self.create_timer(2.0, self.run_test)
        
    def run_test(self):
        # Cancel the timer so this only runs once
        self.timer.cancel()
        
        # Simulate a user asking for an apple
        self.get_logger().info('Simulating user request: "Bring me an apple"')
        
        intent_msg = Intent()
        intent_msg.action = "fetch"
        intent_msg.object = "apple"
        intent_msg.user_location = Point(x=2.0, y=1.5, z=0.0)  # Simulated user location
        
        self.intent_pub.publish(intent_msg)
        self.get_logger().info('Intent published')
        
    def handle_perception_feedback(self, msg: Status):
        self.get_logger().info(f'Perception feedback: {msg.message}')
        
    def handle_manipulation_feedback(self, msg: Status):
        self.get_logger().info(f'Manipulation feedback: {msg.message}')
        
    def handle_navigation_feedback(self, msg: Status):
        self.get_logger().info(f'Navigation feedback: {msg.message}')

def main(args=None):
    rclpy.init(args=args)
    node = TestNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
