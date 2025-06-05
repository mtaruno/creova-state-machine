#!/usr/bin/env python3
"""
Simple test script for orchestration state machine
Run this after launching the system
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
import json
import time
import threading

class SimpleOrchestrationTester(Node):
    def __init__(self):
        super().__init__('simple_tester')
        
        # Publishers to simulate external systems
        self.object_list_pub = self.create_publisher(String, '/perception/object_list', 10)
        self.robot_status_pub = self.create_publisher(String, '/robot_status', 10)
        self.nav2_status_pub = self.create_publisher(String, '/nav2_status', 10)
        self.object_request_pub = self.create_publisher(String, '/latest_object', 10)
        self.destination_request_pub = self.create_publisher(String, '/latest_destination', 10)
        
        # Subscribers to monitor system
        self.state_change_sub = self.create_subscription(
            String, '/state_changes', self.handle_state_change, 10)
        self.manipulation_command_sub = self.create_subscription(
            String, '/manipulation/command', self.handle_manipulation_command, 10)
        self.system_status_sub = self.create_subscription(
            String, '/system_status', self.handle_system_status, 10)
        
        # Service server for destination
        self.destination_service = self.create_service(
            Trigger, 'get_destination', self.handle_destination_request)
        
        # Sample objects
        self.test_objects = [
            {"name": "apple", "x": 1.0, "y": 2.0, "z": 0.5},
            {"name": "bottle", "x": 1.5, "y": 1.0, "z": 0.3},
            {"name": "book", "x": 0.5, "y": 1.5, "z": 0.1}
        ]
        
        self.get_logger().info('Simple Tester initialized')
        
        # Start publishing object list
        self.object_timer = self.create_timer(2.0, self.publish_object_list)
        
        # Start test after delay
        self.test_timer = self.create_timer(5.0, self.run_simple_test)
        self.test_timer.cancel()  # Will start manually
        
    def publish_object_list(self):
        """Publish object list periodically."""
        msg = String()
        msg.data = json.dumps(self.test_objects)
        self.object_list_pub.publish(msg)
    
    def handle_state_change(self, msg):
        """Monitor state changes."""
        try:
            state_data = json.loads(msg.data)
            print(f"🔄 STATE CHANGE: {state_data['entity']} {state_data['from_state']} -> {state_data['to_state']}")
            if state_data.get('reason'):
                print(f"   Reason: {state_data['reason']}")
        except:
            pass
    
    def handle_manipulation_command(self, msg):
        """Monitor and respond to manipulation commands."""
        try:
            command_data = json.loads(msg.data)
            print(f"🤖 MANIPULATION COMMAND: {command_data}")
            
            # Auto-respond with success after 2 seconds
            def respond():
                time.sleep(2)
                response = {
                    "success": True,
                    "message": "Pick completed successfully",
                    "task_id": command_data.get("task_id")
                }
                response_msg = String()
                response_msg.data = json.dumps(response)
                self.robot_status_pub.publish(response_msg)
                print(f"   ✅ Responded with success")
            
            threading.Thread(target=respond).start()
            
        except:
            pass
    
    def handle_system_status(self, msg):
        """Monitor system status."""
        print(f"📊 SYSTEM STATUS: {msg.data}")
    
    def handle_destination_request(self, request, response):
        """Handle destination service requests."""
        print("🎯 Destination service called")
        response.success = True
        response.message = "kitchen"
        
        # Auto-complete navigation after 3 seconds
        def complete_nav():
            time.sleep(3)
            nav_msg = String()
            nav_msg.data = "position: {x: 2.0, y: 3.0, z: 0.0}\ndistance_to_goal: 0.0\nestimated_time_to_goal: 0.0\nready: true"
            self.nav2_status_pub.publish(nav_msg)
            print("   ✅ Navigation completed")
        
        threading.Thread(target=complete_nav).start()
        return response
    
    def run_simple_test(self):
        """Run a simple test scenario."""
        print("\n" + "="*50)
        print("🚀 STARTING SIMPLE TEST")
        print("="*50)
        print("Sending task: apple -> kitchen")
        
        # Send object request
        obj_msg = String()
        obj_msg.data = "apple"
        self.object_request_pub.publish(obj_msg)
        print("✅ Sent object request: apple")
        
        # Send destination request after short delay
        def send_dest():
            time.sleep(0.5)
            dest_msg = String()
            dest_msg.data = "kitchen"
            self.destination_request_pub.publish(dest_msg)
            print("✅ Sent destination request: kitchen")
        
        threading.Thread(target=send_dest).start()
        
        print("\nWatch for state transitions and status updates...")
        print("Expected flow: TASK_QUEUED -> SEARCHING_OBJECT -> PICKING_OBJECT -> READY_FOR_DELIVERY -> DELIVERING -> IDLE")

def main(args=None):
    rclpy.init(args=args)
    node = SimpleOrchestrationTester()
    
    print("\n🎯 Simple Orchestration Tester")
    print("Press Enter to start test, or Ctrl+C to exit")
    
    def wait_for_input():
        try:
            input()
            node.run_simple_test()
        except:
            pass
    
    threading.Thread(target=wait_for_input, daemon=True).start()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n👋 Test completed!")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
