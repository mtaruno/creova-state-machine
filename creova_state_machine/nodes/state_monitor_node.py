#!/usr/bin/env python3
"""
State Monitor Node - Publishes current system state for physical AI team
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class StateMonitorNode(Node):
    def __init__(self):
        super().__init__('state_monitor')

        # Publisher for system status
        self.system_status_pub = self.create_publisher(String, '/system_status', 10)

        # Subscriber for state changes from orchestration
        self.state_change_sub = self.create_subscription(
            String, '/state_changes', self.handle_state_change, 10)

        # Subscriber for manipulation commands to track robot activity
        self.manipulation_command_sub = self.create_subscription(
            String, '/manipulation/command', self.handle_manipulation_command, 10)

        # Current system state - just the status
        self.current_status = 'IDLE'

        # Timer to publish status periodically
        self.status_timer = self.create_timer(2.0, self.publish_status)

        self.get_logger().info('State Monitor Node initialized')

    def handle_state_change(self, msg):
        """Handle state changes from orchestration."""
        try:
            state_data = json.loads(msg.data)
            self.get_logger().info(f"Received state change: {state_data}")

            # Update current state based on the change
            self.update_system_state(state_data)

            # Just log the change
            self.get_logger().debug(f"Updated status to: {self.current_status}")

        except json.JSONDecodeError:
            self.get_logger().error(f"Invalid JSON in state change: {msg.data}")
        except Exception as e:
            self.get_logger().error(f"Error processing state change: {e}")

    def handle_manipulation_command(self, msg):
        """Handle manipulation commands to track robot activity."""
        # Not needed for simple status - manipulation state is tracked via state changes
        _ = msg  # Suppress unused parameter warning

    def update_system_state(self, state_data):
        """Update the current system state based on state changes."""
        entity = state_data.get('entity')
        to_state = state_data.get('to_state')
        old_status = self.current_status

        if entity == 'task':
            # Update system status based on task state
            if to_state == 'PENDING':
                self.current_status = 'TASK_QUEUED'
            elif to_state == 'ACTIVE':
                self.current_status = 'SEARCHING_OBJECT'
            elif to_state == 'PICKING':
                self.current_status = 'PICKING_OBJECT'
            elif to_state == 'WAITING_FOR_DELIVERY':
                self.current_status = 'READY_FOR_DELIVERY'
            elif to_state == 'DELIVERING':
                self.current_status = 'DELIVERING'
            elif to_state == 'COMPLETE':
                self.current_status = 'IDLE'
            elif to_state == 'FAILED':
                self.current_status = 'ERROR'

            # Log status change if it actually changed
            if old_status != self.current_status:
                self.get_logger().info(f"Status updated: {old_status} -> {self.current_status}")

    def publish_status(self):
        """Publish current system status."""
        msg = String()
        msg.data = self.current_status
        self.system_status_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = StateMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
