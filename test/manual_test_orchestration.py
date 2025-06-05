#!/usr/bin/env python3
"""
Manual testing script for orchestration state machine
Allows interactive testing of the system
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
import json
import threading
import time

class ManualTester(Node):
    def __init__(self):
        super().__init__('manual_tester')

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
            {"name": "book", "x": 0.5, "y": 1.5, "z": 0.1},
            {"name": "cup", "x": 2.0, "y": 1.5, "z": 0.2}
        ]

        self.get_logger().info('Manual Tester initialized')

        # Start publishing object list
        self.object_timer = self.create_timer(2.0, self.publish_object_list)

        # Start interactive menu
        self.menu_thread = threading.Thread(target=self.interactive_menu)
        self.menu_thread.daemon = True
        self.menu_thread.start()

    def publish_object_list(self):
        """Publish object list periodically."""
        msg = String()
        msg.data = json.dumps(self.test_objects)
        self.object_list_pub.publish(msg)

    def handle_state_change(self, msg):
        """Monitor state changes."""
        try:
            state_data = json.loads(msg.data)
            print(f"\n🔄 STATE CHANGE: {state_data['entity']} {state_data['from_state']} -> {state_data['to_state']}")
            if state_data.get('reason'):
                print(f"   Reason: {state_data['reason']}")
        except:
            pass

    def handle_manipulation_command(self, msg):
        """Monitor manipulation commands."""
        try:
            command_data = json.loads(msg.data)
            print(f"\n🤖 MANIPULATION COMMAND: {command_data}")

            # Auto-respond to manipulation commands for testing
            def auto_respond():
                time.sleep(1)
                response = {
                    "success": True,
                    "message": "Pick completed successfully",
                    "task_id": command_data.get("task_id")
                }
                response_msg = String()
                response_msg.data = json.dumps(response)
                self.robot_status_pub.publish(response_msg)
                print(f"   ✅ Auto-responded with success")

            threading.Thread(target=auto_respond).start()

        except:
            pass

    def handle_system_status(self, msg):
        """Monitor system status updates."""
        print(f"\n📊 SYSTEM STATUS: {msg.data}")

    def handle_destination_request(self, request, response):
        """Handle destination service requests."""
        _ = request  # Suppress unused parameter warning
        print("\n🎯 Destination service called")
        response.success = True
        response.message = "kitchen"

        # Auto-complete navigation after delay
        def auto_navigate():
            time.sleep(3)
            nav_msg = String()
            nav_msg.data = "position: {x: 2.0, y: 3.0, z: 0.0}\ndistance_to_goal: 0.0\nestimated_time_to_goal: 0.0\nready: true"
            self.nav2_status_pub.publish(nav_msg)
            print("   ✅ Auto-completed navigation")

        threading.Thread(target=auto_navigate).start()
        return response

    def interactive_menu(self):
        """Interactive menu for manual testing."""
        time.sleep(2)  # Wait for node to initialize

        while True:
            print("\n" + "="*50)
            print("ORCHESTRATION MANUAL TESTER")
            print("="*50)
            print("1. Send object request")
            print("2. Send destination request")
            print("3. Send complete task request (object + destination)")
            print("4. Send pick failure response")
            print("5. Send navigation failure")
            print("6. Show available objects")
            print("7. Send multiple tasks")
            print("0. Exit")
            print("-"*50)

            try:
                choice = input("Enter choice: ").strip()

                if choice == '0':
                    break
                elif choice == '1':
                    self.send_object_request()
                elif choice == '2':
                    self.send_destination_request()
                elif choice == '3':
                    self.send_complete_task()
                elif choice == '4':
                    self.send_pick_failure()
                elif choice == '5':
                    self.send_navigation_failure()
                elif choice == '6':
                    self.show_objects()
                elif choice == '7':
                    self.send_multiple_tasks()
                else:
                    print("Invalid choice!")

            except KeyboardInterrupt:
                break
            except Exception as e:
                print(f"Error: {e}")

    def send_object_request(self):
        """Send object request."""
        self.show_objects()
        obj_name = input("Enter object name: ").strip()

        msg = String()
        msg.data = obj_name
        self.object_request_pub.publish(msg)
        print(f"✅ Sent object request: {obj_name}")

    def send_destination_request(self):
        """Send destination request."""
        destinations = ["kitchen", "living_room", "bedroom", "office"]
        print("Available destinations:", destinations)
        dest = input("Enter destination: ").strip()

        msg = String()
        msg.data = dest
        self.destination_request_pub.publish(msg)
        print(f"✅ Sent destination request: {dest}")

    def send_complete_task(self):
        """Send both object and destination to create a complete task."""
        self.show_objects()
        obj_name = input("Enter object name: ").strip()
        dest = input("Enter destination: ").strip()

        # Send object request
        obj_msg = String()
        obj_msg.data = obj_name
        self.object_request_pub.publish(obj_msg)

        # Send destination request
        dest_msg = String()
        dest_msg.data = dest
        self.destination_request_pub.publish(dest_msg)

        print(f"✅ Sent complete task: {obj_name} -> {dest}")

    def send_pick_failure(self):
        """Send a pick failure response."""
        response = {
            "success": False,
            "message": "Manual test: Pick operation failed",
            "task_id": 0
        }
        msg = String()
        msg.data = json.dumps(response)
        self.robot_status_pub.publish(msg)
        print("✅ Sent pick failure response")

    def send_navigation_failure(self):
        """Send navigation failure."""
        nav_msg = String()
        nav_msg.data = "position: {x: 1.0, y: 1.0, z: 0.0}\ndistance_to_goal: 5.0\nestimated_time_to_goal: 10.0\nready: false\nerror: navigation_failed"
        self.nav2_status_pub.publish(nav_msg)
        print("✅ Sent navigation failure")

    def show_objects(self):
        """Show available objects."""
        print("Available objects:")
        for obj in self.test_objects:
            print(f"  - {obj['name']} at ({obj['x']}, {obj['y']}, {obj['z']})")

    def send_multiple_tasks(self):
        """Send multiple tasks for queue testing."""
        tasks = [
            ("apple", "kitchen"),
            ("bottle", "living_room"),
            ("book", "office")
        ]

        print("Sending multiple tasks...")
        for i, (obj, dest) in enumerate(tasks):
            # Send with delays
            def send_task(object_name, destination, delay):
                time.sleep(delay)
                obj_msg = String()
                obj_msg.data = object_name
                self.object_request_pub.publish(obj_msg)

                time.sleep(0.2)
                dest_msg = String()
                dest_msg.data = destination
                self.destination_request_pub.publish(dest_msg)
                print(f"✅ Sent task {i+1}: {object_name} -> {destination}")

            threading.Thread(target=send_task, args=(obj, dest, i * 1.0)).start()

def main(args=None):
    rclpy.init(args=args)
    node = ManualTester()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
