# creova-state-machine

- .srv
- .msg


# Voice-to-Delivery Robot System: FSM and ROS Integration

This README describes the high-level Finite State Machine (FSM) architecture, component interactions, and ROS interface design for a fully autonomous robot system that receives a user's voice command and delivers the correct object using perception, manipulation (Kinova arm), and mobility (Creator Bot).

---

## 🧠 System Overview

**Goal**: Take a voice command like "Bring me an apple" and autonomously:

1. Detect the object (e.g., apple)
2. Use a robot arm to pick it up
3. Navigate a mobile base to the user's location
4. Deliver the object

---

## 🧭 FSM States

The orchestration node maintains the following states:

- IDLE: System is waiting for a new task
- PARSING_INTENT: Processing the user's request
- OBJECT_DETECTION: Detecting and locating the requested object
- PICKING: Using the robot arm to pick up the object
- NAVIGATE_TO_HANDOFF: Moving the mobile base to the handoff position
- HANDOFF: Transferring the object from the arm to the mobile base
- NAVIGATE_TO_USER: Moving the mobile base to the user's location
- DELIVERING: Delivering the object to the user
- RETURN: Returning to the base position
- ERROR_RECOVERY: Handling errors and attempting recovery

## 🔁 Example Event Sequence

### Example: User says "Bring me an apple"

| Step | From → To                   | Topic/Service/Action             | Message Type                | Notes/Key Fields                   | FSM Transition                      |
| ---- | --------------------------- | -------------------------------- | --------------------------- | ---------------------------------- | ----------------------------------- |
| 1    | Physical AI → Orchestration | `/pai/intent_out` (topic)        | `custom_msgs/Intent`        | `action: "fetch", object: "apple"` | `IDLE → PARSING_INTENT`             |
| 2    | FSM internal logic          | —                                | —                           | —                                  | `PARSING_INTENT → OBJECT_DETECTION` |
| 3    | FSM → Perception            | `/perception/detect_object`      | `custom_msgs/ObjectRequest` | `class_label: "apple"`             | —                                   |
| 4    | Perception → FSM            | `/perception/object_pose`        | `custom_msgs/ObjectPose`    | `x, y, z, label, confidence`       | `OBJECT_DETECTION → PICK`           |
| 5    | FSM → Manipulation          | `/manipulation/pick`             | `custom_msgs/PickCommand`   | `pose + label`                     | —                                   |
| 6    | Manipulation → FSM          | `/manipulation/object_acquired`  | `custom_msgs/Status`        | `success: true`                    | `PICK → NAVIGATE_TO_HANDOFF`        |
| 7    | FSM → Navigation            | `/navigation/go_to_arm`          | `geometry_msgs/PoseStamped` | `handoff pose`                     | —                                   |
| 8    | Navigation → FSM            | `/navigation/at_location`        | `custom_msgs/Status`        | `success: true`                    | `NAVIGATE_TO_HANDOFF → HANDOFF`     |
| 9    | FSM → Manipulation          | `/manipulation/handoff`          | `std_msgs/Empty`            | —                                  | —                                   |
| 10   | Manipulation → FSM          | `/manipulation/handoff_complete` | `custom_msgs/Status`        | `success: true`                    | `HANDOFF → NAVIGATE_TO_USER`        |
| 11   | FSM → Navigation            | `/navigation/go_to_user`         | `geometry_msgs/PoseStamped` | `user_location (x, y)`             | —                                   |
| 12   | Navigation → FSM            | `/navigation/delivery_complete`  | `custom_msgs/Status`        | `success: true`                    | `NAVIGATE_TO_USER → DELIVERY`       |
| 13   | FSM → Navigation            | `/navigation/return_to_base`     | `std_msgs/Empty`            | —                                  | —                                   |
| 14   | Navigation → FSM            | `/navigation/at_base`            | `custom_msgs/Status`        | `success: true`                    | `RETURN → IDLE`                     |

---

## 🤖 ROS Components

| Component     | Responsibilities                        | Type                |
| ------------- | --------------------------------------- | ------------------- |
| Physical AI   | Speech recognition → intent generation  | ROS 2 Node (Python) |
| Orchestration | FSM and task sequencing                 |                     |
| Perception    | Object detection and pose estimation    | ROS 2 (Python)      |
| Manipulation  | Kinova arm control (pick/place/handoff) | ROS 2 Node          |
| Navigation    | Creator Bot navigation via Nav2         | ROS 2 Node          |

---

## 🛠️ Message Types Summary

* `custom_msgs/Intent`: intent name, object label, optional location
* `custom_msgs/ObjectRequest`: object class label
* `custom_msgs/ObjectPose`: x, y, z, label, confidence
* `custom_msgs/Status`: success flag, optional error code
* `geometry_msgs/PoseStamped`: target location
* `std_msgs/Empty`: control signal (no payload)

---

## 🔁 Retry & Recovery

In case of errors (e.g., object not found, navigation failed):

* Transition to `ERROR/RECOVERY`
* Re-attempt last task, notify user, or cancel

---

## 🧪 Testing Suggestions

1. Test each subsystem independently via ROS topics/actions
2. Mock intent input → confirm correct FSM state progression
3. Use simulation for navigation and Kinova arm testing
4. Log all FSM state transitions with timestamps

---

## ✅ Final Notes

* Designed for multi-step concurrency: perception, manipulation, and navigation can overlap
* Designed for real robots: Kinova arm + Creator Bot
* Implementation uses Python ROS 2 nodes with a simple state machine pattern

## 🚀 Implementation Details

The system has been implemented with the following files:

1. **src/nodes/orchestration_node.py**: Central state machine that coordinates all other nodes
2. **src/nodes/perception_node.py**: Handles object detection and localization
3. **src/nodes/manipulation_node.py**: Controls the robot arm for picking and handoff operations
4. **src/nodes/navigation_node.py**: Controls the mobile base for navigation
5. **src/custom_msgs/msg/**: Custom message definitions
6. **src/launch/system_launch.py**: Launch file to start all nodes
7. **src/test_system.py**: Test script to demonstrate the system

## 🏃‍♂️ Running the System

To run the system, use the launch file:

```bash
ros2 launch creova_state_machine system_launch.py
```

To test the system, run the test script:

```bash
ros2 run creova_state_machine test_system.py
```

# Creova State Machine

A ROS 2 package for voice-controlled delivery robot system with navigation and perception capabilities.

## Prerequisites

- ROS 2 Humble
- Python 3.10
- Required ROS 2 packages:
  - std_msgs
  - geometry_msgs
  - nav_msgs
  - sensor_msgs
  - tf2_ros

## Setup Instructions

1. Clone the repository:
```bash
cd ~/new_ws/src
git clone <repository-url> creova_state_machine
```

2. Install dependencies:
```bash
cd ~/new_ws
rosdep install --from-paths src --ignore-src -r -y
```

3. Build the package:
```bash
colcon build --symlink-install
```

4. Source the workspace:
```bash
source install/setup.bash
```

## Testing with Navigation Team

### Topic Structure

The navigation node uses the following topics:

1. Subscribed Topics:
   - `/navigation/goal` (std_msgs/String)
     - Receives location requests in JSON format
     - Example: `{"name": "office", "x": 5.2, "y": 3.1}`
   
   - `/navigation/status` (std_msgs/String)
     - Receives navigation status updates in JSON format
     - Example: `{"status": "arrived", "location": "office", "distance": 2.4, "time": 1.8}`

2. Published Topics:
   - `/navigation/set_goal` (std_msgs/String)
     - Forwards location requests to the navigation system
     - Same JSON format as received from `/navigation/goal`
   
   - `/navigation/status_summary` (std_msgs/String)
     - Publishes navigation status summaries
     - Same JSON format as received from `/navigation/status`

### Testing Steps

1. Start the navigation node:
```bash
ros2 run creova_state_machine navigation_node
```

2. Send a location request:
```bash
ros2 topic pub --once /navigation/goal std_msgs/String "data: '{\"name\": \"office\", \"x\": 5.2, \"y\": 3.1}'"
```

3. Send a navigation status update:
```bash
ros2 topic pub --once /navigation/status std_msgs/String "data: '{\"status\": \"arrived\", \"location\": \"office\", \"distance\": 2.4, \"time\": 1.8}'"
```

4. Monitor the status summary:
```bash
ros2 topic echo /navigation/status_summary
```

### Launch File

To launch the entire system:
```bash
ros2 launch creova_state_machine orchestration.launch.py
```

With simulation time:
```bash
ros2 launch creova_state_machine orchestration.launch.py use_sim_time:=true
```

## Message Formats

### Location Request
```json
{
    "name": "office",
    "x": 5.2,
    "y": 3.1
}
```

### Navigation Status
```json
{
    "status": "arrived",
    "location": "office",
    "distance": 2.4,
    "time": 1.8
}
```

## Troubleshooting

1. If topics are not found:
   - Check if the node is running
   - Verify topic names and remappings
   - Use `ros2 topic list` to see available topics

2. If messages are not received:
   - Check JSON format
   - Verify topic names
   - Monitor node logs for errors

3. Common Issues:
   - Make sure to source the workspace
   - Check ROS 2 network configuration
   - Verify all dependencies are installed

## Support

For issues or questions, contact the development team or create an issue in the repository.
