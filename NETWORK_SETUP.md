# ROS 2 Network Setup Guide

This guide explains how to set up ROS 2 network communication between multiple machines for testing the Creova State Machine package.

## Prerequisites

- Both machines must have ROS 2 Humble installed
- Both machines must be on the same network
- Both machines must be able to ping each other

## Network Configuration

### 1. Find Your IP Address

On each machine, run:
```bash
ip addr show
```
Look for your network interface (usually `wlan0` for WiFi or `eth0` for Ethernet) and note the IP address.

### 2. Configure .bashrc

Add the following to your `~/.bashrc` file:

```bash
# ROS 2 Network Configuration
export ROS_DOMAIN_ID=0  # Must be the same on all machines
export ROS_LOCALHOST_ONLY=0  # Allow network communication
export ROS_IP=<YOUR_IP_ADDRESS>  # Replace with your actual IP
export ROS_NODE_NAME="navigation_team"  # Unique name for this machine

# ROS 2 Setup
source /opt/ros/humble/setup.bash
source ~/new_ws/install/setup.bash  # Source your workspace
```

### 3. Apply Changes

After editing `.bashrc`, run:
```bash
source ~/.bashrc
```

## Testing Network Communication

### 1. Basic Network Test

On both machines, run:
```bash
ping <OTHER_MACHINE_IP>
```

### 2. ROS 2 Network Test

1. On the Creova robot machine:
```bash
ros2 run creova_state_machine navigation_node
```

2. On the navigation team's machine:
```bash
ros2 run creova_state_machine navigation_tester
```

### 3. Verify Topics

On either machine, run:
```bash
ros2 topic list
```

You should see the navigation topics:
- `/navigation/goal`
- `/navigation/status`
- `/navigation/set_goal`
- `/navigation/status_summary`

## Troubleshooting

### Common Issues

1. **Topics not visible**
   - Check if both machines have the same `ROS_DOMAIN_ID`
   - Verify `ROS_LOCALHOST_ONLY=0`
   - Ensure both machines can ping each other

2. **Messages not received**
   - Check firewall settings
   - Verify IP addresses are correct
   - Ensure both machines are on the same network

3. **Node not found**
   - Verify workspace is built and sourced
   - Check if the package is installed
   - Ensure all dependencies are installed

### Network Commands

Useful commands for debugging:
```bash
# List all ROS 2 nodes
ros2 node list

# List all topics
ros2 topic list

# Echo a topic
ros2 topic echo /navigation/status_summary

# Check network interfaces
ip addr show

# Test network connectivity
ping <OTHER_MACHINE_IP>
```

## Security Considerations

1. Only use this setup on trusted networks
2. Consider using ROS 2's security features for production
3. Keep ROS 2 and all packages updated
4. Monitor network traffic for unusual activity

## Support

If you encounter any issues:
1. Check the troubleshooting section
2. Verify network configuration
3. Contact the development team
4. Check ROS 2 documentation for network setup 