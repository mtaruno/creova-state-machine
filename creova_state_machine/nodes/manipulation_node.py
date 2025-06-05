#!/usr/bin/env python3
'''
Manipulation Node for controlling the robot arm
'''

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time

class ManipulationNode(Node):
    """
    Node responsible for controlling the robot arm for picking and handoff operations
    """
    def __init__(self):
        super().__init__('manipulation_node')

        # Publishers
        self.feedback_pub = self.create_publisher(
            String,
            '/manipulation/feedback',
            10
        )

        # Subscribers
        self.command_sub = self.create_subscription(
            String,
            '/manipulation/command',
            self.handle_command,
            10
        )

        # State variables
        self.holding_object = False
        self.current_object = None
        self.busy = False

        self.get_logger().info('Manipulation Node initialized')

    def handle_command(self, msg: String):
        """
        Handle command from orchestration node
        """
        try:
            command_data = json.loads(msg.data)
            command = command_data.get('command', '')

            if command == 'pick':
                self.handle_pick_operation(command_data)
            elif command == 'handoff':
                self.handle_handoff_operation(command_data)
            else:
                self.get_logger().warn(f'Unknown command: {command}')

        except json.JSONDecodeError:
            self.get_logger().error(f'Invalid JSON in command: {msg.data}')
        except Exception as e:
            self.get_logger().error(f'Error processing command: {e}')

    def handle_pick_operation(self, command_data):
        """
        Handle pick operation
        """
        object_name = command_data.get('object', '')
        task_id = command_data.get('task_id', 0)

        self.get_logger().info(f'Received pick command for: {object_name} (task {task_id})')

        if self.busy:
            self.send_feedback(False, 1, f"Manipulation node is busy")
            return

        self.busy = True

        # Simulate picking operation (in a real system, this would control the robot arm)
        # For demo purposes, we'll simulate a delay and then success
        self.get_logger().info(f'Starting pick operation for {object_name}...')

        # Simulate some processing time
        time.sleep(2.0)

        # Simulate success
        self.holding_object = True
        self.current_object = object_name
        self.busy = False

        # Send success feedback
        self.send_feedback(True, 0, f"Successfully picked {object_name}")
        self.get_logger().info(f'Pick operation completed for {object_name}')

    def handle_handoff_operation(self, command_data):
        """
        Handle handoff operation
        """
        task_id = command_data.get('task_id', 0)

        if not self.holding_object:
            self.get_logger().error('Received handoff command but not holding any object')
            self.send_feedback(False, 1, "No object to hand off")
            return

        if self.busy:
            self.send_feedback(False, 1, "Manipulation node is busy")
            return

        self.busy = True
        object_name = self.current_object

        self.get_logger().info(f'Performing handoff for: {object_name} (task {task_id})')

        # Simulate handoff operation
        time.sleep(1.0)

        self.holding_object = False
        self.current_object = None
        self.busy = False

        # Send success feedback
        self.send_feedback(True, 0, f"Successfully handed off {object_name}")
        self.get_logger().info(f'Handoff operation completed for {object_name}')

    def send_feedback(self, success, status_code, message):
        """
        Send feedback to orchestration node
        """
        feedback_msg = String()
        feedback_msg.data = json.dumps({
            'success': success,
            'status_code': status_code,
            'message': message,
            'timestamp': time.time()
        })
        self.feedback_pub.publish(feedback_msg)

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