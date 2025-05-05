#!/usr/bin/env python3
'''
Perception Node for object detection
'''

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from geometry_msgs.msg import Pose, Point, Quaternion
from custom_msgs.msg import ObjectRequest, ObjectPose, Status

class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')

        # Publishers
        self.object_pose_pub = self.create_publisher(
            ObjectPose,
            '/perception/object_pose',
            10
        )

        # Subscribers
        self.detect_object_sub = self.create_subscription(
            ObjectRequest,
            '/perception/detect_object',
            self.handle_detect_object_request,
            10
        )

        self.get_logger().info('Perception Node initialized')

    def handle_detect_object_request(self, msg: ObjectRequest):
        """
        Handle object detection requests
        """
        self.get_logger().info(f'Received request to detect: {msg.class_label}')

        # Simulate object detection (in a real system, this would call computer vision)
        # For demo purposes, we'll just return a fixed pose
        detected_pose = Pose()
        detected_pose.position = Point(x=0.5, y=0.2, z=0.1)  # Simulated object position
        detected_pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        # Publish the detected object pose
        object_pose_msg = ObjectPose()
        object_pose_msg.label = msg.class_label
        object_pose_msg.confidence = 0.95  # High confidence for simulation
        object_pose_msg.pose = detected_pose

        self.object_pose_pub.publish(object_pose_msg)
        self.get_logger().info(f'Published pose for {msg.class_label}')

def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()