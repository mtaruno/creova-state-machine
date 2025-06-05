from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),

        # Orchestration Node
        Node(
            package='creova_state_machine',
            executable='orchestration_node',
            name='orchestration_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
            }]
        ),

        # State Monitor Node (for Physical AI team)
        Node(
            package='creova_state_machine',
            executable='state_monitor_node',
            name='state_monitor_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
            }]
        ),



        # Destination Server Node (needed for testing)
        Node(
            package='creova_state_machine',
            executable='destination_server_node',
            name='destination_server_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
            }]
        )
    ])
