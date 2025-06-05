#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trans_msgs.msg import FinalDetection
import random

class PerceptionSimulator(Node):
    def __init__(self):
        super().__init__('perception_simulator')
        self.publisher_ = self.create_publisher(FinalDetection, 'perception', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)  # publish every 1 second
        self.objects = ['cup', 'book', 'bottle', 'phone', 'remote']
        self.index = 0
        self.get_logger().info('Perception simulator node started.')

    def timer_callback(self):
        msg = FinalDetection()
        msg.object_name = self.objects[self.index]
        msg.x = random.uniform(0.0, 1.0)
        msg.y = random.uniform(0.0, 1.0)
        msg.z = random.uniform(0.0, 1.0)

        self.publisher_.publish(msg)
        self.get_logger().info(
            f'Published: {msg.object_name} at (x={msg.x:.2f}, y={msg.y:.2f}, z={msg.z:.2f})'
        )

        # Move to the next object (looping back to 0 after the last one)
        self.index = (self.index + 1) % len(self.objects)

def main(args=None):
    rclpy.init(args=args)
    node = PerceptionSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Simulator interrupted by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
