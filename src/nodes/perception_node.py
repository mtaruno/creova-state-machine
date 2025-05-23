#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
from typing import Optional, List, Dict

class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')
        
        # State variables
        self.latest_object_list: Optional[List[Dict]] = None
        self.target_object: Optional[str] = None
        
        # Subscribers
        self.object_list_sub = self.create_subscription(
            String,
            '/object_list',
            self.object_list_callback,
            10)
        
        self.pick_object_sub = self.create_subscription(
            String,
            '/pick_object',
            self.pick_object_callback,
            10)
        
        # Publishers
        self.manipulation_pub = self.create_publisher(
            String,
            '/manipulation_object',
            10)
        
        self.validation_pub = self.create_publisher(
            String,
            '/validation_object',
            10)
        
        self.get_logger().info('Perception node started')

    def object_list_callback(self, msg: String):
        """Handle incoming object list messages."""
        try:
            self.latest_object_list = json.loads(msg.data)
            self.get_logger().info(f'Received object list with {len(self.latest_object_list)} objects')
            self.process_objects()
        except json.JSONDecodeError:
            self.get_logger().error('Failed to parse object list JSON')
        except Exception as e:
            self.get_logger().error(f'Error processing object list: {str(e)}')

    def pick_object_callback(self, msg: String):
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
                manipulation_msg = String()
                manipulation_msg.data = json.dumps(obj)
                self.manipulation_pub.publish(manipulation_msg)
                self.get_logger().info(f'Found object {self.target_object}, published to manipulation')
                
                # Reset state
                self.latest_object_list = None
                self.target_object = None
                return

        # Object not found, request validation
        validation_msg = String()
        validation_msg.data = self.target_object
        self.validation_pub.publish(validation_msg)
        self.get_logger().info(f'Object {self.target_object} not found, requesting validation')
        
        # Reset state
        self.latest_object_list = None
        self.target_object = None

def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 
