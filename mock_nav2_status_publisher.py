#!/usr/bin/env python3
"""
Mock Nav2 Status Publisher - Simulates navigation team's nav2 status
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
import time
import threading

class MockNav2StatusPublisher(Node):
    def __init__(self):
        super().__init__('mock_nav2_status_publisher')
        
        # Publisher for nav2 status
        self.nav2_status_pub = self.create_publisher(String, '/nav2_status', 10)
        
        # Service server for get_destination
        self.destination_service = self.create_service(
            Trigger, 'get_destination', self.handle_get_destination)
        
        # Navigation state
        self.navigating = False
        self.distance_to_goal = 0.0
        self.navigation_start_time = None
        
        # Timer to publish nav2 status every second
        self.timer = self.create_timer(1.0, self.publish_nav2_status)
        
        self.get_logger().info('Mock Nav2 Status Publisher initialized')
        
    def handle_get_destination(self, request, response):
        """Handle get_destination service calls."""
        self.get_logger().info('Received get_destination service call')
        
        # Start navigation simulation
        self.navigating = True
        self.distance_to_goal = 5.0  # Start with 5 meters to goal
        self.navigation_start_time = time.time()
        
        response.success = True
        response.message = 'Navigation started to destination'
        
        # Simulate navigation progress in separate thread
        def simulate_navigation():
            while self.navigating and self.distance_to_goal > 0:
                time.sleep(1.0)
                self.distance_to_goal = max(0.0, self.distance_to_goal - 1.0)
                
                if self.distance_to_goal <= 0:
                    self.navigating = False
                    self.get_logger().info('Navigation completed - reached destination')
        
        threading.Thread(target=simulate_navigation).start()
        
        return response
        
    def publish_nav2_status(self):
        """Publish nav2 status every second."""
        # Calculate estimated time to goal
        estimated_time = self.distance_to_goal if self.navigating else 0.0
        ready = not self.navigating
        
        status_msg = String()
        status_msg.data = f"""position:
  x: 0.0
  y: 0.0
  z: 0.0
distance_to_goal: {self.distance_to_goal}
estimated_time_to_goal: {estimated_time}
ready: {str(ready).lower()}"""
        
        self.nav2_status_pub.publish(status_msg)
        
        if self.navigating:
            self.get_logger().debug(f'Nav2 status: distance={self.distance_to_goal}, ready={ready}')

def main(args=None):
    rclpy.init(args=args)
    node = MockNav2StatusPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
