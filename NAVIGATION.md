# Navigation Node Documentation

## Overview
The Navigation Node is responsible for handling location requests and navigation status updates in the Creova State Machine system. It acts as an intermediary between the high-level system commands and the navigation system.

## Topics

### Subscribed Topics

1. `/requested_location` (std_msgs/String)
   - Receives location requests in JSON format
   - Example payload:
     ```json
     {
       "name": "office",
       "x": 5.2,
       "y": 3.1
     }
     ```

2. `/nav2_status` (std_msgs/String)
   - Receives navigation status updates in JSON format
   - Example payload:
     ```json
     {
       "status": "arrived",
       "location": "office",
       "distance": 2.4,
       "time": 1.8
     }
     ```

### Published Topics

1. `/go_to_location` (std_msgs/String)
   - Forwards location requests to the navigation system
   - Uses the same JSON format as received from `/requested_location`

2. `/pai_details` (std_msgs/String)
   - Publishes navigation status summaries
   - Example payload:
     ```json
     {
       "location": "office",
       "status": "arrived",
       "distance": 2.4,
       "time": 1.8
     }
     ```

## Node Behavior

1. Location Request Handling:
   - Receives location requests via `/requested_location`
   - Logs the received request
   - Forwards the request to `/go_to_location`
   - Maintains internal state of the current navigation goal

2. Navigation Status Processing:
   - Receives status updates via `/nav2_status`
   - Logs the received status
   - Updates internal system status
   - Publishes status summary to `/pai_details`

## Testing

To test the navigation node:

1. Start the node:
   ```bash
   ros2 run creova_state_machine navigation_node
   ```

2. Send a location request:
   ```bash
   ros2 topic pub --once /requested_location std_msgs/String "data: '{\"name\": \"office\", \"x\": 5.2, \"y\": 3.1}'"
   ```

3. Send a navigation status update:
   ```bash
   ros2 topic pub --once /nav2_status std_msgs/String "data: '{\"status\": \"arrived\", \"location\": \"office\", \"distance\": 2.4, \"time\": 1.8}'"
   ```

## Error Handling

The node includes comprehensive error handling for:
- JSON parsing errors
- Invalid message formats
- Missing required fields

All errors are logged with appropriate error messages.

## Dependencies

- ROS 2 Humble
- std_msgs
- rclpy

## Key Features

### State Management
- Maintains current system status in `self.system_status`
- Updates status on each `/nav2_status` message
- Provides status information to other components

### Message Handling
- Forwards location requests to navigation system
- Processes and validates JSON messages
- Handles malformed messages gracefully

### Logging
- Logs all navigation activities
- Provides detailed error messages
- Tracks state changes and message processing

## Integration Guidelines

### For Navigation System Team
1. **Message Format**
   - Ensure all messages follow the specified JSON format
   - Include all required fields in status updates
   - Use consistent location naming

2. **Status Updates**
   - Provide regular status updates through `/nav2_status`
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
