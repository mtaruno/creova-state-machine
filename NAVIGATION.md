# Navigation Node Documentation

## Overview
The Navigation Node serves as the central component for managing robot navigation tasks. It acts as a bridge between high-level navigation requests and the actual navigation system, handling location requests and monitoring navigation status.

## Topics

### Subscribers

#### 1. `/requested_location` (std_msgs/String)
- **Purpose**: Receives navigation requests from other system components
- **Message Format**: JSON string
```json
{
    "name": "Zubin's office",
    "x": 5.2,
    "y": 3.1
}
```
- **Required Fields**:
  - `name`: Location identifier
  - `x`: X coordinate (float)
  - `y`: Y coordinate (float)
- **Example Usage**:
```bash
ros2 topic pub /requested_location std_msgs/String '{"data": "{\"name\": \"Zubin'\''s office\", \"x\": 5.2, \"y\": 3.1}"}' --once
```

#### 2. `/nav_status` (std_msgs/String)
- **Purpose**: Receives navigation system status updates
- **Message Format**: JSON string
```json
{
    "status": "arrived",
    "location": "Zubin's office",
    "distance": 2.4,
    "time": 1.8
}
```
- **Required Fields**:
  - `status`: Current navigation status (e.g., "arrived", "moving", "error")
  - `location`: Current location name
  - `distance`: Distance to goal in meters
  - `time`: Time taken/remaining in seconds
- **Example Usage**:
```bash
ros2 topic pub /nav_status std_msgs/String '{"data": "{\"status\": \"arrived\", \"location\": \"Zubin'\''s office\", \"distance\": 2.4, \"time\": 1.8}"}' --once
```

### Publishers

#### 1. `/go_to_location` (std_msgs/String)
- **Purpose**: Sends navigation goals to the navigation system
- **Message Format**: Same as `/requested_location`
- **Expected Behavior**: Navigation system should process these goals and move the robot
- **Queue Size**: 10 messages

#### 2. `/pai_details` (std_msgs/String)
- **Purpose**: Forwards navigation status to other system components
- **Message Format**: Same as `/nav_status`
- **Expected Behavior**: Other components can monitor navigation progress
- **Queue Size**: 10 messages

## Key Features

### State Management
- Maintains current system status in `self.system_status`
- Updates status on each `/nav_status` message
- Provides status information to other components

### Message Handling
- Forwards location requests to navigation system
- Processes and validates JSON messages
- Handles malformed messages gracefully

### Logging
- Logs all navigation activities
- Provides detailed error messages
- Tracks state changes and message processing

## Error Handling

### JSON Parsing
- Validates JSON format
- Handles missing or malformed fields
- Provides clear error messages

### Navigation Status
- Monitors navigation progress
- Handles timeout scenarios
- Reports navigation failures

## Integration Guidelines

### For Navigation System Team
1. **Message Format**
   - Ensure all messages follow the specified JSON format
   - Include all required fields in status updates
   - Use consistent location naming

2. **Status Updates**
   - Provide regular status updates through `/nav_status`
   - Include accurate distance and time information
   - Report any navigation errors or issues

3. **Navigation Goals**
   - Process goals received through `/go_to_location`
   - Validate coordinates before execution
   - Report progress through status updates

### For Other Teams
1. **Sending Requests**
   - Use proper JSON format for location requests
   - Include all required fields
   - Handle navigation status responses

2. **Monitoring Progress**
   - Subscribe to `/pai_details` for status updates
   - Handle all possible status values
   - Implement appropriate error handling

## Building and Running

### Build
```bash
cd ~/new_ws
colcon build --symlink-install --packages-select creova_state_machine
source install/setup.bash
```

### Run
```bash
ros2 run creova_state_machine navigation_node
```

## Testing

### Basic Test
```bash
# Terminal 1: Run the node
ros2 run creova_state_machine navigation_node

# Terminal 2: Send a location request
ros2 topic pub /requested_location std_msgs/String '{"data": "{\"name\": \"Zubin'\''s office\", \"x\": 5.2, \"y\": 3.1}"}' --once

# Terminal 3: Monitor navigation status
ros2 topic echo /pai_details
```

### Status Update Test
```bash
# Send a navigation status update
ros2 topic pub /nav_status std_msgs/String '{"data": "{\"status\": \"arrived\", \"location\": \"Zubin'\''s office\", \"distance\": 2.4, \"time\": 1.8}"}' --once
```

## Troubleshooting

### Common Issues
1. **Message Format Errors**
   - Check JSON syntax
   - Verify all required fields are present
   - Ensure proper data types

2. **Navigation Failures**
   - Check coordinate validity
   - Verify location names
   - Monitor status updates

3. **Communication Issues**
   - Verify topic names
   - Check message queue sizes
   - Monitor node status

## Support
For issues or questions, contact the navigation team or create an issue in the repository. 
