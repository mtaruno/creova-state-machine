# Orchestration State Machine Testing Guide

## Quick Start Instructions

### 1. Build and Setup
```bash
cd ~/Dev/creova-state-machine
colcon build --packages-select creova_state_machine
source install/setup.bash
```

### 2. Run the System
```bash
# Terminal 1: Launch the orchestration system
ros2 launch creova_state_machine test_orchestration.launch.py

# Terminal 2: Monitor system status (for Physical AI team)
ros2 topic echo /system_status

# Terminal 3: Manual testing (optional)
ros2 run creova_state_machine manual_test_orchestration
```

## How Your Implementation Works

## End-to-End Flow: How Your State Machine Works

### Step 1: Task Creation
1. **Physical AI sends object request** → `/latest_object` topic (e.g., "apple")
2. **Physical AI sends destination request** → `/latest_destination` topic (e.g., "kitchen")
3. **Orchestration creates task** → Task goes to PENDING state
4. **Status published** → `/system_status` shows "TASK_QUEUED"

### Step 2: Object Detection
1. **State machine starts** → Task transitions PENDING → ACTIVE
2. **Status published** → `/system_status` shows "SEARCHING_OBJECT"
3. **Orchestration checks perception** → Looks for "apple" in `/perception/object_list`

### Step 3: Object Found & Manipulation
1. **Object found** → Task transitions ACTIVE → PICKING
2. **Status published** → `/system_status` shows "PICKING_OBJECT"
3. **Manipulation command sent** → `/manipulation/command` with pick coordinates
4. **Manipulation responds** → `/robot_status` with success/failure

### Step 4: Ready for Delivery
1. **Pick successful** → Task transitions PICKING → WAITING_FOR_DELIVERY
2. **Status published** → `/system_status` shows "READY_FOR_DELIVERY"

### Step 5: Navigation
1. **Navigation starts** → Task transitions WAITING_FOR_DELIVERY → DELIVERING
2. **Status published** → `/system_status` shows "DELIVERING"
3. **Destination service called** → `get_destination` service
4. **Navigation completes** → `/nav2_status` indicates arrival

### Step 6: Task Complete
1. **Task finished** → Task transitions DELIVERING → COMPLETE
2. **Status published** → `/system_status` shows "IDLE"
3. **Next task starts** (if any in queue)

## System Status Values
- `IDLE` - No active tasks
- `TASK_QUEUED` - Task created, waiting to start
- `SEARCHING_OBJECT` - Looking for requested object
- `PICKING_OBJECT` - Robot arm picking up object
- `READY_FOR_DELIVERY` - Object picked, ready to navigate
- `DELIVERING` - Robot navigating to destination
- `ERROR` - Task failed at some point

## Testing Your Implementation

### Automated Testing
The test suite automatically:
1. Publishes fake object lists (apple, bottle, book, cup)
2. Sends task requests (object + destination)
3. Simulates manipulation responses (success/failure)
4. Simulates navigation completion
5. Verifies all state transitions work correctly

### Manual Testing
Use the interactive menu to:
1. Send individual object/destination requests
2. Test complete tasks
3. Simulate failures
4. Test multiple tasks in queue
5. Monitor real-time status changes

### What You'll See
When you run the tests, you'll see:
- State transitions logged in the orchestration node
- System status updates on `/system_status`
- Manipulation commands being sent
- Navigation service calls
- Task queue management

## Key Topics for Integration

### Topics Your Teams Need to Know:

**Input Topics (what orchestration listens to):**
- `/latest_object` - Object requests from Physical AI team
- `/latest_destination` - Destination requests from Physical AI team
- `/perception/object_list` - Available objects from Perception team
- `/robot_status` - Manipulation results from Manipulation team
- `/nav2_status` - Navigation status from Navigation team

**Output Topics (what orchestration publishes):**
- `/system_status` - Simple status string for Physical AI team
- `/manipulation/command` - Pick commands for Manipulation team
- `/state_changes` - Detailed state transitions (for debugging)

**Services:**
- `get_destination` - Called by orchestration to get destination coordinates

## Example Test Run

When you run the system, here's what happens:

```bash
# Terminal 1: Launch system
ros2 launch creova_state_machine test_orchestration.launch.py

# You'll see:
[orchestration_node]: Orchestration FSM initialized and ready
[state_monitor_node]: State Monitor Node initialized
[test_orchestration]: Starting orchestration tests...

# Test sends: object="apple", destination="kitchen"
[orchestration_node]: Received object request: apple
[orchestration_node]: Received destination request: kitchen
[orchestration_node]: Added task 0 to queue: apple -> kitchen

# State machine runs:
[orchestration_node]: [Orch] Starting object detection for task 0
[orchestration_node]: Found object apple in perception list
[orchestration_node]: Pick operation completed: Object picked successfully
[orchestration_node]: [Orch] Starting delivery for task 0
[orchestration_node]: Got destination: kitchen
[orchestration_node]: Navigation completed - robot reached destination
[orchestration_node]: Task 0 completed successfully
```

```bash
# Terminal 2: Monitor status
ros2 topic echo /system_status

# You'll see status changes:
data: "TASK_QUEUED"
data: "SEARCHING_OBJECT"
data: "PICKING_OBJECT"
data: "READY_FOR_DELIVERY"
data: "DELIVERING"
data: "IDLE"
```

## State Persistence
Tasks are saved to: `~/.ros/orchestrator_states/tasks/task_<id>.json`

## Ready for Team Integration
Once testing works, your teams can integrate by:
1. **Physical AI:** Subscribe to `/system_status`, publish to `/latest_object` and `/latest_destination`
2. **Perception:** Publish object lists to `/perception/object_list`
3. **Manipulation:** Subscribe to `/manipulation/command`, publish results to `/robot_status`
4. **Navigation:** Provide `get_destination` service, publish status to `/nav2_status`
