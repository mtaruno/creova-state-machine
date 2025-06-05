# Creova State Machine

A ROS 2 package for voice-controlled delivery robot system with integrated state management, perception, manipulation, and navigation capabilities.

## 🧠 System Overview

**Goal**: Take a voice command like "Bring me an apple" and autonomously:

1. Parse the voice command to extract object and destination
2. Detect and locate the requested object using perception
3. Use a robot arm (Kinova) to pick up the object
4. Navigate a mobile base (Create) to the user's location
5. Deliver the object and return to idle state

## 🏗️ Architecture

The system consists of five main ROS 2 nodes that communicate via topics and maintain persistent state:

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│  Physical AI    │    │  Perception     │    │  Manipulation   │
│     Node        │    │     Node        │    │     Node        │
└─────────────────┘    └─────────────────┘    └─────────────────┘
         │                       │                       │
         │ /latest_object        │ /perception/*         │ /manipulation/*
         │ /latest_destination   │                       │
         │                       │                       │
         └───────────────────────┼───────────────────────┼─────────────┐
                                 │                       │             │
                    ┌─────────────────────────────────────────────────────┐
                    │            Orchestration Node                       │
                    │         (Central State Machine)                     │
                    └─────────────────────────────────────────────────────┘
                                 │                       │
                                 │ /navigation/*         │ /state_changes
                                 │                       │
                    ┌─────────────────┐    ┌─────────────────┐
                    │   Navigation    │    │  State Storage  │
                    │     Node        │    │  (JSON Files)   │
                    └─────────────────┘    └─────────────────┘
```

## 🔄 End-to-End Flow

### Complete Task Execution Flow

1. **Voice Command Input**
   ```bash
   ros2 topic pub /robot_command std_msgs/String '{"data": "{\"id\": 1, \"object\": \"apple\", \"destination\": \"kitchen\"}"}'
   ```

2. **Physical AI Processing**
   - Receives command on `/robot_command`
   - Parses JSON to extract object and destination
   - Publishes to `/latest_object` and `/latest_destination`

3. **Task Creation**
   - Orchestration node receives object and destination
   - Creates new task with unique ID
   - Adds task to queue with state `PENDING`
   - Saves task state to JSON file

4. **Object Detection Phase**
   - Orchestration transitions task to `ACTIVE`
   - Perception team continuously publishes object list to `/perception/object_list`
   - Orchestration searches for requested object in the current object list
   - When found, object coordinates are extracted for manipulation

5. **Picking Phase**
   - Orchestration transitions task to `PICKING`
   - Sends pick command with coordinates to manipulation: `/manipulation/command`
   - Manipulation team publishes status updates to `/robot_status`
   - Task transitions to `WAITING_FOR_DELIVERY` when pick completes

6. **Delivery Phase**
   - Orchestration transitions task to `DELIVERING`
   - Uses `get_destination` service from `destination_server_node.py`
   - Navigation team publishes status every second to `/nav2_status`:
     ```
     position:
       x: 0.0
       y: 0.0
       z: 0.0
     distance_to_goal: 0.0
     estimated_time_to_goal: 0.0
     ready: true
     ```
   - Task transitions to `COMPLETE` when `distance_to_goal: 0.0` and `ready: true`

7. **State Persistence**
   - All state changes published to `/state_changes`
   - Task states saved to `~/.ros/orchestrator_states/tasks/`
   - System starts fresh on restart (clears state directory)

## Task States

Each task progresses through the following states:

- **PENDING**: Task is queued and waiting for execution
- **ACTIVE**: Task is currently being processed (object detection phase)
- **PICKING**: Robot arm is picking up the object
- **WAITING_FOR_DELIVERY**: Pick completed, ready for navigation
- **DELIVERING**: Robot is navigating to deliver the object
- **COMPLETE**: Task successfully completed
- **FAILED**: Task failed due to an error

## 🤖 ROS 2 Nodes

### 1. Orchestration Node (`orchestration_node.py`)
**Central state machine that coordinates all other nodes**

- **Subscribes to:**
  - `/latest_object` - Object requests from Physical AI
  - `/latest_destination` - Destination requests from Physical AI
  - `/perception/object_list` - Continuous list of detected objects with coordinates
  - `/robot_status` - Pick operation status from manipulation team
  - `/nav2_status` - Navigation status from navigation team

- **Publishes to:**
  - `/manipulation/command` - Pick commands with object coordinates
  - `/state_changes` - Real-time state change events

- **Service Clients:**
  - `get_destination` - Service to initiate navigation to destination

- **State Management:**
  - Maintains task queue with persistent JSON storage
  - Tracks robot states (Kinova arm, Create base)
  - Handles task lifecycle from creation to completion

### 2. Physical AI Node (`physical_ai_node.py`)
**Processes voice commands and extracts intent**

- **Subscribes to:**
  - `/robot_command` - Voice command input (JSON format)

- **Publishes to:**
  - `/latest_object` - Extracted object name
  - `/latest_destination` - Extracted destination
  - `/latest_task_id` - Task identifier

### 3. Perception Node (`perception_node.py`)
**Object detection and localization (placeholder - actual implementation by perception team)**

- **Publishes to:**
  - `/perception/object_list` - Continuous list of detected objects with coordinates

- **Expected object list format:**
  ```json
  [
    {"name": "apple", "x": 0.5, "y": 0.2, "z": 0.1},
    {"name": "banana", "x": 0.3, "y": -0.1, "z": 0.1}
  ]
  ```

### 4. Manipulation Node (`manipulation_node.py`)
**Controls Kinova robot arm (placeholder - actual implementation by manipulation team)**

- **Subscribes to:**
  - `/manipulation/command` - Pick commands with object coordinates

- **Publishes to:**
  - `/robot_status` - Operation results and status updates

- **Expected command format:**
  ```json
  {
    "command": "pick",
    "object": "apple",
    "x": 0.5, "y": 0.2, "z": 0.1,
    "task_id": 0
  }
  ```

### 5. Destination Server Node (`destination_server_node.py`)
**Navigation service interface (implemented by navigation team)**

- **Service:**
  - `get_destination` - Triggers navigation to user location

    publish a string

    wall, sink, ....


- **Publishes to:**
  - `/nav2_status` - Real-time navigation status every second

- **Status format:**
  ```
  position: {x: 0.0, y: 0.0, z: 0.0}
  distance_to_goal: 0.0
  estimated_time_to_goal: 0.0
  ready: true
  ```

## 🛠️ Message Types

### Custom Messages
- **Status.msg**: `bool success`, `int32 status_code`, `string message`
- **PickCommand.msg**: `string label`, `geometry_msgs/Pose pose`

### Standard ROS Messages
- **std_msgs/String**: JSON-formatted commands and status
- **geometry_msgs/Pose**: Object and robot poses

## 📊 State Monitoring

### Real-time State Changes
Monitor all state transitions:
```bash
ros2 topic echo /state_changes
```

Example output:
```json
{
  "entity": "task",
  "from_state": "PENDING",
  "to_state": "ACTIVE",
  "timestamp": "2025-05-29T15:10:55.778883",
  "reason": "Task 0 started"
}
```

### Persistent State Storage
Task states are saved to: `~/.ros/orchestrator_states/tasks/`

Example task file:
```json
{
  "task_id": 0,
  "object_name": "apple",
  "user_location": "kitchen",
  "status": "COMPLETE",
  "created_at": "2025-05-29T15:10:55.630883"
}
```

## 🚀 Running the System

### Build the Package
```bash
colcon build --packages-select creova_state_machine
source install/setup.bash
```

### Launch All Nodes
```bash
ros2 launch creova_state_machine integrated_system.launch.py
```

### Test Single Task
```bash
ros2 topic pub /robot_command std_msgs/String '{"data": "{\"id\": 1, \"object\": \"apple\", \"destination\": \"kitchen\"}"}'
```

### Test Multiple Tasks
```bash
# Send first task
ros2 topic pub /robot_command std_msgs/String '{"data": "{\"id\": 1, \"object\": \"apple\", \"destination\": \"kitchen\"}"}'

# Send second task (will be queued)
ros2 topic pub /robot_command std_msgs/String '{"data": "{\"id\": 2, \"object\": \"banana\", \"destination\": \"living_room\"}"}'
```

### Monitor System
```bash
# Monitor state changes
ros2 topic echo /state_changes

# Check task states
cat ~/.ros/orchestrator_states/tasks/task_*.json

# Monitor specific topics
ros2 topic echo /robot_status
ros2 topic echo /nav2_status
ros2 topic echo /perception/object_list
```

## 🔧 Integration with Other Teams

### Perception Team Integration
- Replace the placeholder perception node with your implementation
- Ensure `/perception/object_list` publishes continuous object list in the expected JSON format
- Objects should include `name`, `x`, `y`, `z` coordinates

### Manipulation Team Integration
- Subscribe to `/manipulation/command` for pick commands with object coordinates
- When you receive a command, extract the x, y, z coordinates
- Publish to your arm movement topic to automatically move the arm to those coordinates
- After pick operation completes, publish status to `/robot_status`:
  ```json
  {"status": "success", "message": "Object picked", "task_id": 1}
  ```
- If pick fails, publish failure status:
  ```json
  {"status": "failed", "message": "Unable to reach object", "task_id": 1}
  ```

### Navigation Team Integration
- The orchestration node will call the `get_destination` service when ready for delivery
- Your implementation should validate the destination from `/latest_destination`
- Only accept valid destinations: "sink", "wall", or "elevator"
- If destination is valid, start navigation and return success response
- If destination is invalid, return failure response and don't move
- Publish navigation status to `/nav2_status` every second with:
  ```
  position: {x: current_x, y: current_y, z: current_z}
  distance_to_goal: remaining_distance
  estimated_time_to_goal: estimated_seconds
  ready: true/false  # true when navigation is complete
  ```

## 🧪 Testing Strategy

1. **Unit Testing**: Test each node independently
2. **Integration Testing**: Test node-to-node communication
3. **End-to-End Testing**: Test complete task execution flow
4. **Queue Testing**: Test multiple tasks in queue
5. **Error Testing**: Test error handling and recovery

## 📝 Network Topics

When running in a distributed environment, ensure all nodes can see these topics:

### Core Topics
- `/robot_command` - Voice command input
- `/latest_object` - Object extraction from Physical AI
- `/latest_destination` - Destination extraction from Physical AI
- `/state_changes` - System state monitoring

### Team Integration Topics
- `/perception/object_list` - From perception team
- `/manipulation/command` - To manipulation team
- `/robot_status` - From manipulation team
- `/nav2_status` - From navigation team

### Services
- `get_destination` - Navigation service
