#!/usr/bin/env python3
"""
Test script to verify the integration between orchestration node and other nodes
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time

class IntegrationTester(Node):
    def __init__(self):
        super().__init__('integration_tester')

        # Publishers to simulate physical_ai_node
        self.object_pub = self.create_publisher(String, '/latest_object', 10)
        self.destination_pub = self.create_publisher(String, '/latest_destination', 10)

        # Subscriber to monitor state changes
        self.state_sub = self.create_subscription(
            String, '/state_changes', self.state_change_callback, 10)

        self.get_logger().info('Integration tester initialized')

        # Wait a bit for connections to establish
        time.sleep(2.0)

        # Send test commands
        self.send_test_task()

    def state_change_callback(self, msg):
        """Monitor state changes from orchestration node."""
        try:
            state_data = json.loads(msg.data)
            entity = state_data.get('entity', '')
            from_state = state_data.get('from_state', '')
            to_state = state_data.get('to_state', '')
            reason = state_data.get('reason', '')

            self.get_logger().info(f"State change - {entity}: {from_state} -> {to_state} ({reason})")
        except Exception as e:
            self.get_logger().error(f"Error parsing state change: {e}")

    def send_test_task(self):
        """Send a test task to the system."""
        self.get_logger().info("Sending test task...")

        # Send object request
        object_msg = String()
        object_msg.data = "apple"
        self.object_pub.publish(object_msg)
        self.get_logger().info("Sent object request: apple")

        # Send destination request
        destination_msg = String()
        destination_msg.data = "kitchen"
        self.destination_pub.publish(destination_msg)
        self.get_logger().info("Sent destination request: kitchen")

        # Wait a bit and send a second task
        def send_second_task():
            time.sleep(10.0)
            self.get_logger().info("Sending second test task...")

            # Send second object request
            object_msg2 = String()
            object_msg2.data = "banana"
            self.object_pub.publish(object_msg2)
            self.get_logger().info("Sent object request: banana")

            # Send second destination request
            destination_msg2 = String()
            destination_msg2.data = "living_room"
            self.destination_pub.publish(destination_msg2)
            self.get_logger().info("Sent destination request: living_room")

        # Schedule second task
        import threading
        threading.Thread(target=send_second_task).start()

def main(args=None):
    rclpy.init(args=args)
    tester = IntegrationTester()

    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        pass
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
