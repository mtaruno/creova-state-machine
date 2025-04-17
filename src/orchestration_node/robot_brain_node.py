'''
Main ROS node with state machine

'''
# src/orchestration_node/robot_brain_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from orchestration_node.fsm import OrchestrationFSM, OrchestrationState

class RobotBrainNode(Node):
    def __init__(self):
        super().__init__('robot_brain')
        self.fsm = OrchestrationFSM(self)

        self.subscription = self.create_subscription(
            String,
            '/object_request',
            self.object_request_callback,
            10
        )

        # Initialize service clients here
        # self.get_object_pose_client = self.create_client(...)

        self.timer = self.create_timer(0.5, self.run_fsm)

    def object_request_callback(self, msg):
        self.get_logger().info(f"Received object request: {msg.data}")
        # Handle the object request, possibly triggering a state transition

    def run_fsm(self):
        # Periodically evaluate and act based on the current state
        pass

def main(args=None):
    rclpy.init(args=args)
    node = RobotBrainNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()