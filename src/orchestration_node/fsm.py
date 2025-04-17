'''
Custom state machine
'''
# src/orchestration_node/fsm.py

from enum import Enum

class OrchestrationState(Enum):
    IDLE = 0
    WAIT_FOR_REQUEST = 1
    SEARCH_OBJECT = 2
    PICK_OBJECT = 3
    WAIT_FOR_ROBOT = 4
    LOAD_OBJECT = 5
    DELIVERING = 6
    RETURNING = 7
    ERROR = 8

class OrchestrationFSM:
    def __init__(self, node):
        self.node = node
        self.state = OrchestrationState.IDLE
        self.current_request = None

    def transition_to(self, new_state):
        self.node.get_logger().info(f"Transitioning from {self.state.name} to {new_state.name}")
        self.state = new_state
        # Add any entry actions for the new state here

    def handle_event(self, event):
        # Define how events trigger state transitions
        pass