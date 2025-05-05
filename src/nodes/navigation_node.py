#!/usr/bin/env python3
'''
Navigation Node for controlling the mobile base
'''

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from geometry_msgs.msg import PoseStamped
from custom_msgs.msg import Status

class NavigationNode(Node):
    """
    Node responsible for controlling the mobile base for navigation
    """
    def __init__(self):
        super().__init__('navigation_node')

        # Publishers
        self.feedback_pub = self.create_publisher(
            Status,
            '/navigation/feedback',
            10
        )
        self.at_location_pub = self.create_publisher(
            Status,
            '/navigation/at_location',
            10
        )
        self.delivery_complete_pub = self.create_publisher(
            Status,
            '/navigation/delivery_complete',
            10
        )
        self.at_base_pub = self.create_publisher(
            Status,
            '/navigation/at_base',
            10
        )

        # Subscribers
        self.go_to_arm_sub = self.create_subscription(
            PoseStamped,
            '/navigation/go_to_arm',
            self.handle_go_to_arm,
            10
        )
        self.go_to_user_sub = self.create_subscription(
            PoseStamped,
            '/navigation/go_to_user',
            self.handle_go_to_user,
            10
        )
        self.return_to_base_sub = self.create_subscription(
            Empty,
            '/navigation/return_to_base',
            self.handle_return_to_base,
            10
        )

        # State variables
        self.current_location = "base"  # Start at base

        self.get_logger().info('Navigation Node initialized')

    def handle_go_to_arm(self, msg: PoseStamped):
        """
        Handle command to navigate to the arm for handoff
        """
        self.get_logger().info(f'Received command to navigate to arm position: {msg.pose.position}')

        # Simulate navigation to arm (in a real system, this would control the mobile base)
        # For demo purposes, we'll just simulate success
        self.current_location = "arm"

        # Send feedback that navigation was successful
        feedback = Status()
        feedback.success = True
        feedback.status_code = 0
        feedback.message = "Successfully navigated to arm position"

        self.feedback_pub.publish(feedback)

        # Also publish to at_location topic
        at_location_msg = Status()
        at_location_msg.success = True
        at_location_msg.status_code = 0
        at_location_msg.message = "At arm location for handoff"

        self.at_location_pub.publish(at_location_msg)
        self.get_logger().info('Navigation to arm completed')

    def handle_go_to_user(self, msg: PoseStamped):
        """
        Handle command to navigate to the user for delivery
        """
        self.get_logger().info(f'Received command to navigate to user position: {msg.pose.position}')

        # Simulate navigation to user
        self.current_location = "user"

        # Send feedback that navigation was successful
        feedback = Status()
        feedback.success = True
        feedback.status_code = 0
        feedback.message = "Successfully navigated to user position"

        self.feedback_pub.publish(feedback)

        # Also publish to delivery_complete topic
        delivery_msg = Status()
        delivery_msg.success = True
        delivery_msg.status_code = 0
        delivery_msg.message = "Delivery to user complete"

        self.delivery_complete_pub.publish(delivery_msg)
        self.get_logger().info('Navigation to user completed')

    def handle_return_to_base(self, _):
        """
        Handle command to return to base
        """
        self.get_logger().info('Received command to return to base')

        # Simulate navigation to base
        self.current_location = "base"

        # Send feedback that navigation was successful
        feedback = Status()
        feedback.success = True
        feedback.status_code = 0
        feedback.message = "Successfully returned to base"

        self.feedback_pub.publish(feedback)

        # Also publish to at_base topic
        at_base_msg = Status()
        at_base_msg.success = True
        at_base_msg.status_code = 0
        at_base_msg.message = "Robot has returned to base"

        self.at_base_pub.publish(at_base_msg)
        self.get_logger().info('Return to base completed')

def main(args=None):
    """
    Main function to initialize and run the node
    """
    rclpy.init(args=args)
    node = NavigationNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()