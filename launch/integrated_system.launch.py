from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Orchestration Node - Central state machine
        Node(
            package='creova_state_machine',
            executable='orchestration_node',
            name='orchestrator',
            output='screen',
            parameters=[
                {'use_sim_time': False}
            ]
        ),
        
        # Manipulation Node - Controls robot arm
        Node(
            package='creova_state_machine',
            executable='manipulation_node',
            name='manipulation_node',
            output='screen'
        ),
        
        # Navigation Node - Controls robot movement
        Node(
            package='creova_state_machine',
            executable='navigation_node',
            name='navigation_node',
            output='screen'
        ),
        
        # Physical AI Node - Processes voice commands
        Node(
            package='creova_state_machine',
            executable='physical_ai_node',
            name='physical_ai_node',
            output='screen'
        ),
        
        # Perception Node - Object detection and localization
        Node(
            package='creova_state_machine',
            executable='perception_node',
            name='perception_node',
            output='screen'
        ),
    ])
