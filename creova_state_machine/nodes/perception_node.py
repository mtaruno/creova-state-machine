#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trans_msgs.msg import FinalDetection
import json
import os

class PerceptionRecorder(Node):
    def __init__(self):
        super().__init__('perception_node')

        # State variables
        self.latest_object_list: Optional[List[Dict]] = None
        self.target_object: Optional[str] = None

        # Subscribers
        self.object_list_sub = self.create_subscription(
            String,
            '/perception/object_list',
            self.object_list_callback,
            10)

        self.pick_request_sub = self.create_subscription(
            String,
            '/perception/pick_request',
            self.pick_request_callback,
            10)

        # Publishers
        self.object_details_pub = self.create_publisher(
            String,
            '/perception/object_details',
            10)

        self.validation_request_pub = self.create_publisher(
            String,
            '/perception/validation_request',
            10)

        self.get_logger().info('Perception node started')

    def listener_callback(self, msg):
        self.detections[msg.object_name] = {
            'x': msg.x,
            'y': msg.y,
            'z': msg.z
        }

    def pick_request_callback(self, msg: String):
        """Handle incoming pick object requests."""
        self.target_object = msg.data
        self.get_logger().info(f'Received pick request for object: {self.target_object}')
        self.process_objects()

    def process_objects(self):
        """Process objects when both list and target are available."""
        if self.latest_object_list is None or self.target_object is None:
            return

        # Search for the target object in the list
        for obj in self.latest_object_list:
            if obj.get('name') == self.target_object:
                # Found the object, publish its details
                object_details_msg = String()
                object_details_msg.data = json.dumps(obj)
                self.object_details_pub.publish(object_details_msg)
                self.get_logger().info(f'Found object {self.target_object}, published object details')

                # Reset state
                self.latest_object_list = None
                self.target_object = None
                return

        # Object not found, request validation
        validation_msg = String()
        validation_msg.data = self.target_object
        self.validation_request_pub.publish(validation_msg)
        self.get_logger().info(f'Object {self.target_object} not found, requesting validation')

        # Reset state
        self.latest_object_list = None
        self.target_object = None

def main(args=None):
    rclpy.init(args=args)
    node = PerceptionRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Recorder node interrupted.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
