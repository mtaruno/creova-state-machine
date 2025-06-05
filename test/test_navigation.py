#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time

class NavigationTester(Node):
    def __init__(self):
        super().__init__('navigation_tester')
        
        # Create publishers
        self.goal_pub = self.create_publisher(String, 'requested_location', 10)
        self.status_pub = self.create_publisher(String, 'nav2_status', 10)
        
        # Create subscribers
        self.goal_sub = self.create_subscription(
            String,
            'go_to_location',
            self.goal_callback,
            10)
        self.status_sub = self.create_subscription(
            String,
            'pai_details',
            self.status_callback,
            10)
        
        self.get_logger().info('Navigation tester node started')
        
    def goal_callback(self, msg):
        """Callback for received set_goal messages"""
        try:
            data = json.loads(msg.data)
            self.get_logger().info(f'Received goal: {data}')
        except json.JSONDecodeError:
            self.get_logger().error(f'Invalid JSON in goal message: {msg.data}')
            
    def status_callback(self, msg):
        """Callback for received status_summary messages"""
        try:
            data = json.loads(msg.data)
            self.get_logger().info(f'Received status: {data}')
        except json.JSONDecodeError:
            self.get_logger().error(f'Invalid JSON in status message: {msg.data}')
            
    def send_test_goal(self):
        """Send a test goal message"""
        goal_msg = {
            'name': 'office',
            'x': 5.2,
            'y': 3.1
        }
        msg = String()
        msg.data = json.dumps(goal_msg)
        self.goal_pub.publish(msg)
        self.get_logger().info(f'Sent goal: {goal_msg}')
        
    def send_test_status(self):
        """Send a test status message"""
        status_msg = {
            'status': 'arrived',
            'location': 'office',
            'distance': 2.4,
            'time': 1.8
        }
        msg = String()
        msg.data = json.dumps(status_msg)
        self.status_pub.publish(msg)
        self.get_logger().info(f'Sent status: {status_msg}')

def main(args=None):
    rclpy.init(args=args)
    tester = NavigationTester()
    
    try:
        # Send test messages
        tester.send_test_goal()
        time.sleep(1)  # Wait for goal to be processed
        tester.send_test_status()
        
        # Keep node running
        rclpy.spin(tester)
    except KeyboardInterrupt:
        pass
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 