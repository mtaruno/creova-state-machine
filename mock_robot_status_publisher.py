#!/usr/bin/env python3
"""
Mock Robot Status Publisher - Simulates manipulation team's robot status
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
import threading

class MockRobotStatusPublisher(Node):
    def __init__(self):
        super().__init__('mock_robot_status_publisher')
        
        # Publisher for robot status
        self.robot_status_pub = self.create_publisher(String, '/robot_status', 10)
        
        # Subscriber for manipulation commands
        self.command_sub = self.create_subscription(
            String, '/manipulation/command', self.handle_command, 10)
        
        self.get_logger().info('Mock Robot Status Publisher initialized')
        
    def handle_command(self, msg: String):
        """Handle manipulation commands and simulate robot response."""
        try:
            command_data = json.loads(msg.data)
            command = command_data.get('command', '')
            object_name = command_data.get('object', '')
            task_id = command_data.get('task_id', 0)
            
            self.get_logger().info(f'Received command: {command} for {object_name} (task {task_id})')
            
            if command == 'pick':
                # Simulate pick operation in separate thread
                def simulate_pick():
                    time.sleep(3.0)  # Simulate pick time
                    
                    # Send success status
                    status_msg = String()
                    status_msg.data = json.dumps({
                        'success': True,
                        'message': f'Successfully picked {object_name}',
                        'task_id': task_id,
                        'timestamp': time.time()
                    })
                    self.robot_status_pub.publish(status_msg)
                    self.get_logger().info(f'Pick operation completed for {object_name}')
                
                threading.Thread(target=simulate_pick).start()
                
        except json.JSONDecodeError:
            self.get_logger().error(f'Invalid JSON in command: {msg.data}')
        except Exception as e:
            self.get_logger().error(f'Error processing command: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = MockRobotStatusPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
