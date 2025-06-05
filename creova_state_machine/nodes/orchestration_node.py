#!/usr/bin/env python3
"""
Orchestration Node - Central state machine for the creova delivery robot system
"""

from enum import Enum
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from collections import deque
import json
import os
from datetime import datetime

class TaskState(Enum):
    PENDING = 0
    ACTIVE = 1
    PICKING = 2
    COMPLETE = 3
    FAILED = 4
    WAITING_FOR_DELIVERY = 6
    DELIVERING = 7

class Task:
    def __init__(self, task_id, object_name, user_location):
        self.task_id = task_id
        self.object_name = object_name
        self.user_location = user_location
        self.status = TaskState.PENDING
        self.created_at = datetime.now().isoformat()

    def update_state(self, new_state: TaskState, reason=None):
        old_state = self.status
        self.status = new_state
        return old_state, new_state

class OrchestrationFSM(Node):
    def __init__(self):
        super().__init__('orchestrator')
        self.task_queue = deque()

        # State directory for persistence - start fresh each time
        self.state_dir = os.path.expanduser("~/.ros/orchestrator_states")
        self.clear_and_init_state_dir()

        # Publishers
        self.manipulation_pub = self.create_publisher(String, '/manipulation/command', 10)
        self.state_change_pub = self.create_publisher(String, '/state_changes', 10)

        # Subscribers
        self.robot_status_sub = self.create_subscription(
            String, '/robot_status', self.handle_robot_status, 10)
        self.nav2_status_sub = self.create_subscription(
            String, '/nav2_status', self.handle_nav2_status, 10)
        self.perception_object_list_sub = self.create_subscription(
            String, '/perception/object_list', self.handle_object_list, 10)
        self.object_sub = self.create_subscription(
            String, '/latest_object', self.handle_object_request, 10)
        self.destination_sub = self.create_subscription(
            String, '/latest_destination', self.handle_destination_request, 10)

        # Service client for navigation
        self.get_destination_client = self.create_client(
            Trigger, 'get_destination')

        # Current object list from perception
        self.current_object_list = []

        # Task creation parameters
        self.pending_task = {
            'task_id': 0,
            'object_name': None,
            'user_location': None
        }

        # Robot states
        self.kinova_busy = False
        self.create_busy = False

        # Timer for state machine execution
        self.timer = self.create_timer(1.0, self.run_state_machine)

        self.get_logger().info('Orchestration FSM initialized and ready')

    def clear_and_init_state_dir(self):
        """Clear state directory and start fresh."""
        import shutil
        if os.path.exists(self.state_dir):
            shutil.rmtree(self.state_dir)
        os.makedirs(self.state_dir, exist_ok=True)
        self.get_logger().info(f"State directory cleared and initialized: {self.state_dir}")

    def publish_state_change(self, entity, from_state, to_state, reason=None):
        """Publish a state change event."""
        msg = String()
        msg.data = json.dumps({
            'entity': entity,
            'from_state': from_state,
            'to_state': to_state,
            'timestamp': datetime.now().isoformat(),
            'reason': reason
        })
        self.state_change_pub.publish(msg)

    def save_states(self):
        """Save all current states to files."""
        try:
            # Save tasks
            tasks_dir = os.path.join(self.state_dir, 'tasks')
            os.makedirs(tasks_dir, exist_ok=True)
            for task in self.task_queue:
                task_file = os.path.join(tasks_dir, f'task_{task.task_id}.json')
                with open(task_file, 'w') as f:
                    json.dump({
                        'task_id': task.task_id,
                        'object_name': task.object_name,
                        'user_location': task.user_location,
                        'status': task.status.name,
                        'created_at': task.created_at
                    }, f, indent=2)
        except Exception as e:
            self.get_logger().error(f"Failed to save states: {e}")

    def handle_object_request(self, msg: String):
        """Handle object request from physical_ai_node."""
        self.pending_task['object_name'] = msg.data
        self.get_logger().info(f"Received object request: {msg.data}")
        self.try_create_task()

    def handle_destination_request(self, msg: String):
        """Handle destination request from physical_ai_node."""
        self.pending_task['user_location'] = msg.data
        self.get_logger().info(f"Received destination request: {msg.data}")
        self.try_create_task()

    def try_create_task(self):
        """Check if we have all required information to create a task."""
        if self.pending_task['object_name'] and self.pending_task['user_location']:
            task = Task(
                self.pending_task['task_id'],
                self.pending_task['object_name'],
                self.pending_task['user_location']
            )
            self.add_task(task)
            # Increment task ID for next task
            self.pending_task['task_id'] += 1
            # Reset pending
            self.pending_task['object_name'] = None
            self.pending_task['user_location'] = None

    def add_task(self, task):
        """Add a task to the queue."""
        self.task_queue.append(task)
        self.get_logger().info(f"Added task {task.task_id} to queue: {task.object_name} -> {task.user_location}")
        self.publish_state_change('task', 'CREATED', 'PENDING', f"Task {task.task_id} created")
        self.save_states()

    def run_state_machine(self):
        """Main state machine logic."""
        if not self.task_queue:
            return

        current_task = self.task_queue[0]  # Peek at first task without removing it

        if current_task.status == TaskState.PENDING and not self.kinova_busy:
            self.get_logger().info(f"[Orch] Starting object detection for task {current_task.task_id}")

            # Update states
            current_task.update_state(TaskState.ACTIVE, "Starting object detection")
            self.publish_state_change('task', 'PENDING', 'ACTIVE', f"Task {current_task.task_id} started")

        elif current_task.status == TaskState.ACTIVE and not self.kinova_busy:
            # Look for object in current object list from perception
            self.find_object_in_list(current_task)

        elif current_task.status == TaskState.WAITING_FOR_DELIVERY and not self.create_busy:
            self.get_logger().info(f"[Orch] Starting delivery for task {current_task.task_id}")

            # Update states
            current_task.update_state(TaskState.DELIVERING, "Starting delivery")
            self.create_busy = True
            self.publish_state_change('task', 'WAITING_FOR_DELIVERY', 'DELIVERING', f"Task {current_task.task_id} delivering")

            # Use service to get destination and start navigation
            self.start_navigation(current_task)

    def find_object_in_list(self, task):
        """Find object in current perception list and send to manipulation."""
        for obj in self.current_object_list:
            if obj.get('name') == task.object_name:
                self.get_logger().info(f"Found object {task.object_name} in perception list")

                # Update task state
                task.update_state(TaskState.PICKING, "Object found, starting pick operation")
                self.kinova_busy = True
                self.publish_state_change('task', 'ACTIVE', 'PICKING', f"Task {task.task_id} picking")

                # Send pick command to manipulation node
                pick_msg = String()
                pick_msg.data = json.dumps({
                    'command': 'pick',
                    'object': task.object_name,
                    'x': obj.get('x', 0.0),
                    'y': obj.get('y', 0.0),
                    'z': obj.get('z', 0.0),
                    'task_id': task.task_id
                })
                self.manipulation_pub.publish(pick_msg)
                return

        # Object not found in list
        self.get_logger().warn(f"Object {task.object_name} not found in perception list")

    def start_navigation(self, task):
        """Start navigation using destination service."""
        if not self.get_destination_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Destination service not available")
            return

        request = Trigger.Request()
        future = self.get_destination_client.call_async(request)

        def handle_destination_response(future):
            try:
                response = future.result()
                if response.success:
                    self.get_logger().info(f"Got destination: {response.message}")
                    # Navigation will be handled by monitoring /nav2_status
                else:
                    self.get_logger().error(f"Failed to get destination: {response.message}")
            except Exception as e:
                self.get_logger().error(f"Service call failed: {e}")

        future.add_done_callback(handle_destination_response)

    def handle_object_list(self, msg: String):
        """Handle object list from perception team."""
        try:
            object_list = json.loads(msg.data)
            self.current_object_list = object_list
            self.get_logger().debug(f"Updated object list with {len(object_list)} objects")
        except json.JSONDecodeError:
            self.get_logger().error(f"Invalid JSON in object list: {msg.data}")
        except Exception as e:
            self.get_logger().error(f"Error processing object list: {e}")

    def handle_robot_status(self, msg: String):
        """Handle robot status from manipulation team."""
        try:
            status = json.loads(msg.data)
            success = status.get('success', False)
            message = status.get('message', '')

            if success:
                self.get_logger().info(f"Pick operation completed: {message}")
                self.kinova_busy = False

                # Update current task
                if self.task_queue:
                    current_task = self.task_queue[0]
                    current_task.update_state(TaskState.WAITING_FOR_DELIVERY, "Pick completed, ready for delivery")
                    self.publish_state_change('task', 'PICKING', 'WAITING_FOR_DELIVERY', f"Task {current_task.task_id} pick completed")
            else:
                self.get_logger().error(f"Robot error: {message}")
                self.kinova_busy = False

                # Update current task
                if self.task_queue:
                    current_task = self.task_queue[0]
                    current_task.update_state(TaskState.FAILED, f"Pick failed: {message}")
                    self.publish_state_change('task', 'PICKING', 'FAILED', f"Task {current_task.task_id} pick failed")

        except json.JSONDecodeError:
            self.get_logger().error(f"Invalid JSON in robot status: {msg.data}")
        except Exception as e:
            self.get_logger().error(f"Error processing robot status: {e}")

    def handle_nav2_status(self, msg: String):
        """Handle nav2 status from navigation team."""
        try:
            # Parse the nav2 status message
            # Expected format based on your example:
            # position: {x: 0.0, y: 0.0, z: 0.0}
            # distance_to_goal: 0.0
            # estimated_time_to_goal: 0.0
            # ready: true

            # For now, we'll check if the robot is ready and distance_to_goal is 0
            # This indicates the robot has reached its destination
            if "distance_to_goal: 0.0" in msg.data and "ready: true" in msg.data:
                if self.task_queue and self.task_queue[0].status == TaskState.DELIVERING:
                    self.get_logger().info("Navigation completed - robot reached destination")
                    self.create_busy = False

                    # Complete the current task
                    current_task = self.task_queue.popleft()  # Remove completed task
                    current_task.update_state(TaskState.COMPLETE, "Delivery completed successfully")
                    self.get_logger().info(f"Task {current_task.task_id} completed successfully")
                    self.publish_state_change('task', 'DELIVERING', 'COMPLETE', f"Task {current_task.task_id} completed")

        except Exception as e:
            self.get_logger().error(f"Error processing nav2 status: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = OrchestrationFSM()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
