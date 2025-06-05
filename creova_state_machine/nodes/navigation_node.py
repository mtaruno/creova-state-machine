#!/usr/bin/env python3

# Standard library imports
import json
import time
import threading

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
        self.goal_sub = self.create_subscription(
            String,
            '/navigation/goal',
            self.handle_navigation_goal,
            10)

        self.nav_status_sub = self.create_subscription(
            String,
            'nav2_status',
            self.nav_status_callback,
            10)

        # Create publishers
        self.status_pub = self.create_publisher(
            String,
            '/navigation/status',
            10)

        self.go_to_location_pub = self.create_publisher(
            String,
            'go_to_location',
            10)

        # Initialize system status
        self.system_status = {
            'location': '',
            'status': '',
            'distance': 0.0,
            'time': 0.0
        }

        # Navigation state
        self.busy = False
        self.current_task_id = None

        self.get_logger().info('Navigation node initialized')

    def handle_navigation_goal(self, msg):
        """Handle navigation goal from orchestration node."""
        try:
            goal_data = json.loads(msg.data)
            destination = goal_data.get('destination', '')
            task_id = goal_data.get('task_id', 0)

            self.get_logger().info(f'Received navigation goal: {destination} (task {task_id})')

            if self.busy:
                self.send_status_feedback(False, f"Navigation node is busy")
                return

            self.busy = True
            self.current_task_id = task_id

            # Simulate navigation (in real system, this would interface with nav2)
            self.simulate_navigation(destination)

        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse navigation goal JSON: {e}')
        except Exception as e:
            self.get_logger().error(f'Error processing navigation goal: {e}')

    def simulate_navigation(self, destination):
        """Simulate navigation to destination."""
        def navigate():
            self.get_logger().info(f'Starting navigation to {destination}...')

            # Simulate navigation time
            time.sleep(3.0)

            # Simulate successful arrival
            self.system_status['location'] = destination
            self.system_status['status'] = 'arrived'

            self.busy = False
            self.send_status_feedback(True, f"Successfully delivered to {destination}")
            self.get_logger().info(f'Navigation to {destination} completed')

        # Run navigation in separate thread to avoid blocking
        nav_thread = threading.Thread(target=navigate)
        nav_thread.start()

    def send_status_feedback(self, success, message):
        """Send status feedback to orchestration node."""
        feedback_msg = String()
        feedback_msg.data = json.dumps({
            'success': success,
            'message': message,
            'task_id': self.current_task_id,
            'timestamp': time.time()
        })
        self.status_pub.publish(feedback_msg)

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
            self.get_logger().error(f"Failed to write {FILE_PATH}: {e}")

def main(args=None):
    """
    Main entry point for the navigation node.

    Initializes ROS 2, creates and spins the navigation node.
    Handles graceful shutdown on keyboard interrupt.
    """
    rclpy.init(args=args)
    node = NavigationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
