# Perception Node Documentation

## Overview
The Perception Node manages object detection and manipulation requests. It processes lists of detected objects, matches them against pick requests, and coordinates the manipulation process by providing object details or requesting validation when objects aren't found.

## Topics

### Subscribers

#### 1. `/object_list` (std_msgs/String)
- **Purpose**: Receives list of detected objects from the perception system
- **Message Format**: JSON string
```json
[
    {
        "name": "apple",
        "x": 1.2,
        "y": 0.4,
        "z": 0.1
    },
    {
        "name": "banana",
        "x": 2.0,
        "y": 0.5,
        "z": 0.2
    }
]
```
- **Required Fields**:
  - `name`: Object identifier
  - `x`: X coordinate (float)
  - `y`: Y coordinate (float)
  - `z`: Z coordinate (float)
- **Example Usage**:
```bash
ros2 topic pub /object_list std_msgs/String '{"data": "[{\"name\": \"apple\", \"x\": 1.2, \"y\": 0.4, \"z\": 0.1}, {\"name\": \"banana\", \"x\": 2.0, \"y\": 0.5, \"z\": 0.2}]"}' --once
```

#### 2. `/pick_object` (std_msgs/String)
- **Purpose**: Receives object pick requests
- **Message Format**: Simple string with object name
- **Example Usage**:
```bash
ros2 topic pub /pick_object std_msgs/String '{"data": "apple"}' --once
```

### Publishers

#### 1. `/manipulation_object` (std_msgs/String)
- **Purpose**: Sends object details for manipulation when found
- **Message Format**: JSON string with full object details
```json
{
    "name": "apple",
    "x": 1.2,
    "y": 0.4,
    "z": 0.1
}
```
- **Queue Size**: 10 messages

#### 2. `/validation_object` (std_msgs/String)
- **Purpose**: Sends object name for validation when not found
- **Message Format**: Simple string with object name
- **Queue Size**: 10 messages

## Key Features

### State Management
- Maintains latest object list in `self.latest_object_list`
- Tracks current target object in `self.target_object`
- Processes objects only when both list and target are available
- Resets state after each processing cycle

### Object Processing
- Matches requested objects against detected objects
- Provides full object details for manipulation
- Handles cases where objects aren't found
- Maintains object coordinate information

### Logging
- Logs all object detection activities
- Tracks object processing results
- Provides detailed error messages
- Monitors state changes

## Error Handling

### JSON Parsing
- Validates JSON format for object lists
- Handles missing or malformed fields
- Provides clear error messages

### Object Matching
- Handles cases where objects aren't found
- Validates object names
- Ensures coordinate data is complete

## Integration Guidelines

### For Perception System Team
1. **Object Detection**
   - Provide complete object lists with coordinates
   - Use consistent object naming
   - Include all required fields in object data

2. **Message Format**
   - Follow specified JSON format
   - Ensure proper coordinate values
   - Maintain consistent object identifiers

3. **Error Handling**
   - Handle detection failures gracefully
   - Provide meaningful error messages
   - Implement appropriate recovery strategies

### For Manipulation Team
1. **Object Details**
   - Process object coordinates from `/manipulation_object`
   - Handle object validation requests
   - Implement appropriate error handling

2. **Coordinate System**
   - Use provided x, y, z coordinates
   - Verify coordinate validity
   - Handle coordinate transformations if needed

## Building and Running

### Build
```bash
cd ~/new_ws
colcon build --symlink-install --packages-select creova_state_machine
source install/setup.bash
```

### Run
```bash
ros2 run creova_state_machine perception_node
```

## Testing

### Basic Test
```bash
# Terminal 1: Run the node
ros2 run creova_state_machine perception_node

# Terminal 2: Send an object list
ros2 topic pub /object_list std_msgs/String '{"data": "[{\"name\": \"apple\", \"x\": 1.2, \"y\": 0.4, \"z\": 0.1}, {\"name\": \"banana\", \"x\": 2.0, \"y\": 0.5, \"z\": 0.2}]"}' --once

# Terminal 3: Send a pick request
ros2 topic pub /pick_object std_msgs/String '{"data": "apple"}' --once

# Terminal 4: Monitor manipulation object
ros2 topic echo /manipulation_object
```

### Validation Test
```bash
# Send a pick request for non-existent object
ros2 topic pub /pick_object std_msgs/String '{"data": "orange"}' --once

# Monitor validation object
ros2 topic echo /validation_object
```

## Troubleshooting

### Common Issues
1. **Object Detection**
   - Check object list format
   - Verify coordinate values
   - Ensure consistent object naming

2. **Object Matching**
   - Verify object names match
   - Check coordinate system
   - Monitor processing results

3. **Communication Issues**
   - Verify topic names
   - Check message queue sizes
   - Monitor node status

## Support
For issues or questions, contact the perception team or create an issue in the repository. 
