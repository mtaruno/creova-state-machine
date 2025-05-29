#!/usr/bin/env python3

# Standard library imports
import json
from typing import Optional, List, Dict

# ROS 2 imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class NavigationNode(Node):
    """
    Navigation Node for handling location requests and navigation status.
    
    This node manages location requests and navigation status by:
    1. Receiving and processing location requests
    2. Receiving and processing navigation status updates
    3. Publishing location requests and navigation status

    TODO: Need to add publishers from here to physical_ai_node: pai_details topic
    """
    
    def __init__(self):
        """Initialize the navigation node with subscribers and publishers."""
        super().__init__('navigation_node')
        
        # Create subscribers
        self.location_request_sub = self.create_subscription(
            String,
            'requested_location',
            self.location_request_callback,
            10)
            
        self.nav_status_sub = self.create_subscription(
            String,
            'nav2_status',
            self.nav_status_callback,
            10)
            
        # Create publishers
        self.go_to_location_pub = self.create_publisher(
            String,
            'go_to_location',
            10)
            
        self.pai_details_pub = self.create_publisher(
            String,
            'pai_details',
            10)
            
        # Initialize system status
        self.system_status = {
            'location': '',
            'status': '',
            'distance': 0.0,
            'time': 0.0
        }
        
        self.get_logger().info('Navigation node initialized')

    def location_request_callback(self, msg):
        try:
            # Parse the JSON string
            location_data = json.loads(msg.data)
            self.get_logger().info(f'Received location request: {location_data}')
            
            # Publish to go_to_location
            go_to_msg = String()
            go_to_msg.data = msg.data  # Reuse the same JSON string
            self.go_to_location_pub.publish(go_to_msg)
            self.get_logger().info(f'Published to go_to_location: {go_to_msg.data}')
            
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse location request JSON: {e}')
        except Exception as e:
            self.get_logger().error(f'Error processing location request: {e}')

    def nav_status_callback(self, msg):
        try:
            # Parse the JSON string
            status_data = json.loads(msg.data)
            self.get_logger().info(f'Received navigation status: {status_data}')
            
            # Update system status
            self.system_status = {
                'location': status_data.get('location', ''),
                'status': status_data.get('status', ''),
                'distance': status_data.get('distance', 0.0),
                'time': status_data.get('time', 0.0)
            }
            
            # Publish to pai_details
            pai_msg = String()
            pai_msg.data = json.dumps(self.system_status)
            self.pai_details_pub.publish(pai_msg)
            self.get_logger().info(f'Published to pai_details: {pai_msg.data}')
            
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse navigation status JSON: {e}')
        except Exception as e:
            self.get_logger().error(f'Error processing navigation status: {e}')

def main(args=None):
    """
    Main entry point for the navigation node.
    
    Initializes ROS 2, creates and spins the navigation node.
    Handles graceful shutdown on keyboard interrupt.
    """
    rclpy.init(args=args)
    node = NavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 
