# Example: Dummy get_object_pose service server

import rclpy
from rclpy.node import Node
from orchestration_node.srv import GetObjectPose
from geometry_msgs.msg import Pose

class DummyPerceptionServer(Node):
    def __init__(self):
        super().__init__('dummy_perception_server')
        self.srv = self.create_service(GetObjectPose, '/get_object_pose', self.get_object_pose_callback)

    def get_object_pose_callback(self, request, response):
        self.get_logger().info(f"Received request for object: {request.object_name}")
        response.pose = Pose()
        response.success = True
        response.message = "Object found."
        return response

def main(args=None):
    rclpy.init(args=args)
    node = DummyPerceptionServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()