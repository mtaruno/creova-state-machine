#!/usr/bin/env python3

# Standard library imports
import json
from typing import Optional, List, Dict

# ROS 2 imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PerceptionNode(Node):
    """
    Perception Node for handling object detection and manipulation requests.
    
    This node manages object perception tasks by:
    1. Receiving and storing lists of detected objects
    2. Processing pick requests for specific objects
    3. Matching requested objects against detected objects
    4. Publishing results for manipulation or validation
    """
    
    def __init__(self):
        """Initialize the perception node with subscribers and publishers."""
        super().__init__('perception_node')
        
        # State variables to store latest data
        self.latest_object_list: Optional[List[Dict]] = None  # List of detected objects
        self.target_object: Optional[str] = None  # Currently requested object
        
        # Subscribers
        # 1. Object list subscriber - receives detected objects
        self.object_list_sub = self.create_subscription(
            String,
            '/object_list',
            self.object_list_callback,
            10)  # Queue size of 10 messages
        
        # 2. Pick request subscriber - receives object pick requests
        self.pick_object_sub = self.create_subscription(
            String,
            '/pick_object',
            self.pick_object_callback,
            10)
        
        # Publishers
        # 1. Manipulation publisher - sends object details for manipulation
        self.manipulation_pub = self.create_publisher(
            String,
            '/manipulation_object',
            10)
        
        # 2. Validation publisher - sends object name for validation
        self.validation_pub = self.create_publisher(
            String,
            '/validation_object',
            10)
        
        self.get_logger().info('Perception node started')

    def object_list_callback(self, msg: String):
        """
        Handle incoming object list messages.
        
        Args:
            msg (String): JSON string containing list of detected objects:
                [
                    {
                        "name": str,  # Object name
                        "x": float,   # X coordinate
                        "y": float,   # Y coordinate
                        "z": float    # Z coordinate
                    },
                    ...
                ]
        """
        try:
            # Parse and store the object list
            self.latest_object_list = json.loads(msg.data)
            self.get_logger().info(f'Received object list with {len(self.latest_object_list)} objects')
            self.process_objects()  # Process if we have a target object
        except json.JSONDecodeError:
            self.get_logger().error('Failed to parse object list JSON')
        except Exception as e:
            self.get_logger().error(f'Error processing object list: {str(e)}')

    def pick_object_callback(self, msg: String):
        """
        Handle incoming pick object requests.
        
        Args:
            msg (String): Name of the object to pick
        """
        self.target_object = msg.data
        self.get_logger().info(f'Received pick request for object: {self.target_object}')
        self.process_objects()  # Process if we have an object list

    def process_objects(self):
        """
        Process objects when both list and target are available.
        
        This method:
        1. Checks if both object list and target are available
        2. Searches for the target object in the list
        3. If found, publishes full object details for manipulation
        4. If not found, publishes object name for validation
        5. Resets state after processing
        """
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
    """
    Main entry point for the perception node.
    
    Initializes ROS 2, creates and spins the perception node.
    Handles graceful shutdown on keyboard interrupt.
    """
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
