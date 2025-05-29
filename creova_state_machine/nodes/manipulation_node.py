#!/usr/bin/env python3
'''
Manipulation Node for controlling the robot arm
'''

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from geometry_msgs.msg import Pose
from custom_msgs.msg import PickCommand, Status

class ManipulationNode(Node):
    """
    Node responsible for controlling the robot arm for picking and handoff operations
    """
    def __init__(self):
        super().__init__('manipulation_node')

        # Publishers
        self.feedback_pub = self.create_publisher(
            Status,
            '/manipulation/feedback',
            10
        )
        self.object_acquired_pub = self.create_publisher(
            Status,
            '/manipulation/object_acquired',
            10
        )
        self.handoff_complete_pub = self.create_publisher(
            Status,
            '/manipulation/handoff_complete',
            10
        )

        # Subscribers
        self.pick_sub = self.create_subscription(
            PickCommand,
            '/manipulation/pick',
            self.handle_pick_command,
            10
        )
        self.handoff_sub = self.create_subscription(
            Empty,
            '/manipulation/handoff',
            self.handle_handoff_command,
            10
        )

        # State variables
        self.holding_object = False
        self.current_object = None

        self.get_logger().info('Manipulation Node initialized')

    def handle_pick_command(self, msg: PickCommand):
        """
        Handle pick command from orchestration
        """
        self.get_logger().info(f'Received pick command for: {msg.label} at position: {msg.pose.position}')

        # Simulate picking operation (in a real system, this would control the robot arm)
        # For demo purposes, we'll just simulate success
        self.holding_object = True
        self.current_object = msg.label

        # Send feedback that pick was successful
        feedback = Status()
        feedback.success = True
        feedback.status_code = 0
        feedback.message = f"Successfully picked {msg.label}"

        self.feedback_pub.publish(feedback)

        # Also publish to object_acquired topic
        acquired_msg = Status()
        acquired_msg.success = True
        acquired_msg.status_code = 0
        acquired_msg.message = f"Object {msg.label} acquired"

        self.object_acquired_pub.publish(acquired_msg)
        self.get_logger().info(f'Pick operation completed for {msg.label}')

    def handle_handoff_command(self, _):
        """
        Handle handoff command from orchestration
        """
        if not self.holding_object:
            self.get_logger().error('Received handoff command but not holding any object')
            feedback = Status()
            feedback.success = False
            feedback.status_code = 1
            feedback.message = "No object to hand off"
            self.feedback_pub.publish(feedback)
            return

        self.get_logger().info(f'Performing handoff for: {self.current_object}')

        # Simulate handoff operation
        self.holding_object = False
        object_handed = self.current_object
        self.current_object = None

        # Send feedback that handoff was successful
        feedback = Status()
        feedback.success = True
        feedback.status_code = 0
        feedback.message = f"Successfully handed off {object_handed}"

        self.feedback_pub.publish(feedback)

        # Also publish to handoff_complete topic
        handoff_msg = Status()
        handoff_msg.success = True
        handoff_msg.status_code = 0
        handoff_msg.message = f"Handoff of {object_handed} complete"

        self.handoff_complete_pub.publish(handoff_msg)
        self.get_logger().info(f'Handoff operation completed for {object_handed}')

def main(args=None):
    """
    Main function to initialize and run the node
    """
    rclpy.init(args=args)
    node = ManipulationNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()