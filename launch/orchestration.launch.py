from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='creova_state_machine',
            executable='orchestration_node',
            name='orchestration_node',
            output='screen'
        ),
        Node(
            package='creova_state_machine',
            executable='navigation_node',
            name='navigation_node',
            output='screen'
        ),
        Node(
            package='creova_state_machine',
            executable='manipulation_node',
            name='manipulation_node',
            output='screen'
        ),
        Node(
            package='creova_state_machine',
            executable='perception_node',
            name='perception_node',
            output='screen'
        ),
        Node(
            package='creova_state_machine',
            executable='task_input_node',
            name='task_input_node',
            output='screen'
        ),

    ])