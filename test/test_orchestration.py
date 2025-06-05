#!/usr/bin/env python3
"""
Comprehensive test suite for the orchestration state machine
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
import json
import time
import threading
from enum import Enum

class TestScenario(Enum):
    SINGLE_TASK_SUCCESS = "single_task_success"
    MULTIPLE_TASKS = "multiple_tasks"
    PICK_FAILURE = "pick_failure"
    NAVIGATION_FAILURE = "navigation_failure"
    OBJECT_NOT_FOUND = "object_not_found"

class OrchestrationTester(Node):
    def __init__(self):
        super().__init__('orchestration_tester')
        
        # Publishers to simulate external systems
        self.object_list_pub = self.create_publisher(String, '/perception/object_list', 10)
        self.robot_status_pub = self.create_publisher(String, '/robot_status', 10)
        self.nav2_status_pub = self.create_publisher(String, '/nav2_status', 10)
        self.object_request_pub = self.create_publisher(String, '/latest_object', 10)
        self.destination_request_pub = self.create_publisher(String, '/latest_destination', 10)
        
        # Subscribers to monitor orchestration
        self.state_change_sub = self.create_subscription(
            String, '/state_changes', self.handle_state_change, 10)
        self.manipulation_command_sub = self.create_subscription(
            String, '/manipulation/command', self.handle_manipulation_command, 10)
        
        # Service server for destination
        self.destination_service = self.create_service(
            Trigger, 'get_destination', self.handle_destination_request)
        
        # Test state tracking
        self.received_state_changes = []
        self.received_manipulation_commands = []
        self.current_test_scenario = None
        self.test_results = {}
        
        # Sample object list for testing
        self.test_objects = [
            {"name": "apple", "x": 1.0, "y": 2.0, "z": 0.5},
            {"name": "bottle", "x": 1.5, "y": 1.0, "z": 0.3},
            {"name": "book", "x": 0.5, "y": 1.5, "z": 0.1}
        ]
        
        self.get_logger().info('Orchestration Tester initialized')
        
        # Start publishing object list periodically
        self.object_timer = self.create_timer(1.0, self.publish_object_list)
        
    def publish_object_list(self):
        """Continuously publish object list to simulate perception."""
        msg = String()
        msg.data = json.dumps(self.test_objects)
        self.object_list_pub.publish(msg)
        
    def handle_state_change(self, msg):
        """Monitor state changes from orchestration."""
        try:
            state_data = json.loads(msg.data)
            self.received_state_changes.append(state_data)
            self.get_logger().info(f"State change: {state_data}")
        except json.JSONDecodeError:
            self.get_logger().error(f"Invalid state change JSON: {msg.data}")
    
    def handle_manipulation_command(self, msg):
        """Monitor manipulation commands from orchestration."""
        try:
            command_data = json.loads(msg.data)
            self.received_manipulation_commands.append(command_data)
            self.get_logger().info(f"Manipulation command: {command_data}")
            
            # Simulate manipulation response based on test scenario
            self.simulate_manipulation_response(command_data)
            
        except json.JSONDecodeError:
            self.get_logger().error(f"Invalid manipulation command JSON: {msg.data}")
    
    def handle_destination_request(self, request, response):
        """Handle destination service requests."""
        self.get_logger().info("Destination service called")
        response.success = True
        response.message = "kitchen"
        return response
    
    def simulate_manipulation_response(self, command):
        """Simulate manipulation team responses."""
        def delayed_response():
            time.sleep(2)  # Simulate manipulation time
            
            response_msg = String()
            if self.current_test_scenario == TestScenario.PICK_FAILURE:
                response_data = {
                    "success": False,
                    "message": "Failed to grasp object",
                    "task_id": command.get("task_id")
                }
            else:
                response_data = {
                    "success": True,
                    "message": "Object picked successfully",
                    "task_id": command.get("task_id")
                }
            
            response_msg.data = json.dumps(response_data)
            self.robot_status_pub.publish(response_msg)
            self.get_logger().info(f"Published robot status: {response_data}")
        
        # Run in separate thread to avoid blocking
        threading.Thread(target=delayed_response).start()
    
    def simulate_navigation_completion(self):
        """Simulate navigation completion."""
        def delayed_nav_completion():
            time.sleep(3)  # Simulate navigation time
            
            nav_msg = String()
            if self.current_test_scenario == TestScenario.NAVIGATION_FAILURE:
                nav_msg.data = "position: {x: 1.0, y: 1.0, z: 0.0}\ndistance_to_goal: 2.0\nestimated_time_to_goal: 5.0\nready: false"
            else:
                nav_msg.data = "position: {x: 2.0, y: 3.0, z: 0.0}\ndistance_to_goal: 0.0\nestimated_time_to_goal: 0.0\nready: true"
            
            self.nav2_status_pub.publish(nav_msg)
            self.get_logger().info(f"Published nav2 status: {nav_msg.data}")
        
        threading.Thread(target=delayed_nav_completion).start()
    
    def run_test_scenario(self, scenario: TestScenario):
        """Run a specific test scenario."""
        self.current_test_scenario = scenario
        self.received_state_changes.clear()
        self.received_manipulation_commands.clear()
        
        self.get_logger().info(f"Starting test scenario: {scenario.value}")
        
        if scenario == TestScenario.OBJECT_NOT_FOUND:
            # Test with object not in perception list
            self.send_task_request("nonexistent_object", "kitchen")
        else:
            # Test with valid object
            self.send_task_request("apple", "kitchen")
        
        # Schedule navigation simulation if needed
        if scenario not in [TestScenario.PICK_FAILURE, TestScenario.OBJECT_NOT_FOUND]:
            threading.Timer(8.0, self.simulate_navigation_completion).start()
    
    def send_task_request(self, object_name, destination):
        """Send object and destination requests to create a task."""
        # Send object request
        obj_msg = String()
        obj_msg.data = object_name
        self.object_request_pub.publish(obj_msg)
        
        # Send destination request (slight delay to test both orders)
        def send_destination():
            time.sleep(0.5)
            dest_msg = String()
            dest_msg.data = destination
            self.destination_request_pub.publish(dest_msg)
        
        threading.Thread(target=send_destination).start()
    
    def run_multiple_tasks_test(self):
        """Test multiple tasks in queue."""
        self.current_test_scenario = TestScenario.MULTIPLE_TASKS
        self.received_state_changes.clear()
        self.received_manipulation_commands.clear()
        
        self.get_logger().info("Starting multiple tasks test")
        
        # Send first task
        self.send_task_request("apple", "kitchen")
        
        # Send second task after a delay
        def send_second_task():
            time.sleep(2.0)
            self.send_task_request("bottle", "living_room")
        
        threading.Thread(target=send_second_task).start()
        
        # Schedule navigation completions
        threading.Timer(8.0, self.simulate_navigation_completion).start()
        threading.Timer(20.0, self.simulate_navigation_completion).start()
    
    def start_tests(self):
        """Start the test sequence."""
        self.get_logger().info("Starting orchestration tests...")
        
        # Test 1: Single successful task
        self.run_test_scenario(TestScenario.SINGLE_TASK_SUCCESS)
        
        # Schedule other tests
        threading.Timer(15.0, lambda: self.run_test_scenario(TestScenario.PICK_FAILURE)).start()
        threading.Timer(25.0, lambda: self.run_test_scenario(TestScenario.OBJECT_NOT_FOUND)).start()
        threading.Timer(35.0, self.run_multiple_tasks_test).start()
        
        # Print results after all tests
        threading.Timer(60.0, self.print_test_results).start()
    
    def print_test_results(self):
        """Print summary of test results."""
        self.get_logger().info("=== TEST RESULTS SUMMARY ===")
        self.get_logger().info(f"Total state changes received: {len(self.received_state_changes)}")
        self.get_logger().info(f"Total manipulation commands: {len(self.received_manipulation_commands)}")
        
        # Analyze state transitions
        transitions = {}
        for change in self.received_state_changes:
            transition = f"{change.get('from_state')} -> {change.get('to_state')}"
            transitions[transition] = transitions.get(transition, 0) + 1
        
        self.get_logger().info("State transitions observed:")
        for transition, count in transitions.items():
            self.get_logger().info(f"  {transition}: {count} times")

def main(args=None):
    rclpy.init(args=args)
    node = OrchestrationTester()
    
    # Start tests after a delay
    def start_testing():
        time.sleep(5)  # Wait for orchestration to be ready
        node.start_tests()
    
    threading.Thread(target=start_testing).start()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
