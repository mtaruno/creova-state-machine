from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """Generate launch description for the complete system."""
    
    return LaunchDescription([
        Node(
            package='creova_state_machine',
            executable='orchestration_node.py',
            name='orchestration',
            output='screen'
        ),
        Node(
            package='creova_state_machine',
            executable='perception_node.py',
            name='perception',
            output='screen'
        ),
        Node(
            package='creova_state_machine',
            executable='manipulation_node.py',
            name='manipulation',
            output='screen'
        ),
        Node(
            package='creova_state_machine',
            executable='navigation_node.py',
            name='navigation',
            output='screen'
        )
    ])
