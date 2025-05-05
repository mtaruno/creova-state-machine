'''
Custom state machine
'''
# src/orchestration_node/fsm.py

from enum import Enum
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped
from custom_msgs.msg import Status
from collections import deque

class OrchestrationState(Enum):
    IDLE = 0
    PARSING_INTENT = 1
    OBJECT_DETECTION = 2
    PICKING = 3
    NAVIGATE_TO_HANDOFF = 4
    HANDOFF = 5
    NAVIGATE_TO_USER = 6
    DELIVERING = 7
    RETURN = 8
    ERROR_RECOVERY = 9

class TaskState(Enum):
    PENDING = 0 # task is queued and waiting execution
    ACTIVE = 1 # task is currently being executed
    PICKING = 2 # task is currently being picked
    COMPLETE = 3 # task has completed successfully
    FAILED = 4
    CANCELLED = 5
    WAITING_FOR_DELIVERY = 6
    DELIVERING = 7

class Task:
    def __init__(self, task_id, object_name, user_location):
        self.task_id = task_id
        self.object_name = object_name
        self.user_location = user_location
        self.status = TaskState.PENDING
        self.object_pose = None # where it is kept on the table (from perception)
        self.pick_complete = False
        self.delivery_complete = False


class Create:
    def __init__(self):
        self.busy = False
        self.current_location = None
        self.status = OrchestrationState.IDLE

class Kinova:
    def __init__(self):
        self.busy = False
        self.status = OrchestrationState.IDLE
        self.holding_object = False


class OrchestrationFSM(Node):
    def __init__(self):
        super().__init__('orchestrator')
        self.task_queue = deque()
        self.create = Create()
        self.kinova = Kinova()

        # Subscribers for feeedback from other modules
        self.create_feedback_sub = self.create_subscription(Status, '/navigation/feedback', self.handle_create_feedback, 10)
        self.kinova_feedback_sub = self.create_subscription(Status, '/manipulation/feedback', self.handle_kinova_feedback, 10)

        self.timer = self.create_timer(1.0, self.run_state_machine)

    def add_task(self, task):
        self.task_queue.append(task)
        self.get_logger().info(f"Added task {task.task_id} to queue")

    def run_state_machine(self):
        if not self.task_queue:
            return

        current_task = self.task_queue.popleft()

        if not self.kinova.busy and current_task.status == TaskState.PENDING:
            self.get_logger().info(f"[Orch] Sending pick command to manipulation node")
            current_task.status = TaskState.ACTIVE
            self.kinova.busy = True
            self.kinova.status = OrchestrationState.PICKING

        elif current_task.status == TaskState.WAITING_FOR_DELIVERY:
            self.get_logger().info(f"[Orch] Sending delivery command to navigation node")
            self.create.status = OrchestrationState.DELIVERING
            self.create.busy = True
            current_task.status = TaskState.DELIVERING


    def handle_kinova_feedback(self, msg: Status):
        if msg.success and msg.status_code == 0:
            self.get_logger().info(f"Pick complete by Kinova")
            self.kinova.busy = False
            self.kinova.status = OrchestrationState.IDLE
            self.task_queue.popleft()
        else:
            self.get_logger().error(f"Kinova error: {msg.message}")

    def handle_create_feedback(self, msg: Status):
        if msg.success and msg.status_code == 0:
            self.get_logger().info(f"Navigation task complete: {msg.message}")
            self.create.busy = False
            self.create.status = OrchestrationState.IDLE

            # Check if this was a delivery completion
            if "Delivery" in msg.message:
                current_task = self.task_queue[0] if self.task_queue else None
                if current_task and current_task.status == TaskState.DELIVERING:
                    current_task.status = TaskState.COMPLETE
                    self.get_logger().info(f"Task {current_task.task_id} completed successfully")
                    self.task_queue.popleft()

            # Check if this was a return to base
            elif "base" in msg.message:
                self.get_logger().info("Robot has returned to base and is ready for new tasks")
        else:
            self.get_logger().error(f"Navigation error: {msg.message}")
            # Handle error case

def main(args=None):
    rclpy.init(args=args)
    node = OrchestrationFSM()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()


