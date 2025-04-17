from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='orchestration_node',
            executable='robot_brain_node',
            name='robot_brain',
            output='screen'
        )
    ])