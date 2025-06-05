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
            description='Use simulation time for real robot deployment'
        ),

        # Core Orchestration Node - Central state machine
        Node(
            package='creova_state_machine',
            executable='orchestration_node',
            name='orchestration_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
            }],
            respawn=True,
            respawn_delay=2.0
        ),

        # State Monitor Node - Publishes system status for Physical AI team
        Node(
            package='creova_state_machine',
            executable='state_monitor_node',
            name='state_monitor_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
            }],
            respawn=True,
            respawn_delay=2.0
        ),

        # Physical AI Node - Handles voice commands and user interaction
        Node(
            package='creova_state_machine',
            executable='physical_ai_node',
            name='physical_ai_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
            }],
            respawn=True,
            respawn_delay=2.0
        ),

        # Perception Node - Object detection and recognition
        Node(
            package='creova_state_machine',
            executable='perception_node',
            name='perception_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
            }],
            remappings=[
                ('object_list', '/perception/object_list'),
                ('pick_request', '/perception/pick_request'),
                ('pick_result', '/perception/pick_result')
            ],
            respawn=True,
            respawn_delay=2.0
        ),

        # Manipulation Node - Kinova arm control
        Node(
            package='creova_state_machine',
            executable='manipulation_node',
            name='manipulation_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
            }],
            respawn=True,
            respawn_delay=2.0
        ),

        # Navigation Node - Create3 robot navigation
        Node(
            package='creova_state_machine',
            executable='navigation_node',
            name='navigation_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
            }],
            remappings=[
                ('nav2_status', '/nav2_status'),
                ('requested_location', '/navigation/goal'),
                ('go_to_location', '/navigation/set_goal'),
                ('pai_details', '/navigation/status_summary')
            ],
            respawn=True,
            respawn_delay=2.0
        ),

        # Destination Server Node - Provides destination lookup service
        Node(
            package='creova_state_machine',
            executable='destination_server_node',
            name='destination_server_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
            }],
            respawn=True,
            respawn_delay=2.0
        ),

        # Get Destination Client - Helper for destination requests
        Node(
            package='creova_state_machine',
            executable='get_destination_client',
            name='get_destination_client',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
            }],
            respawn=True,
            respawn_delay=2.0
        ),

        # Final Detection Subscriber - Handles final object detection results
        Node(
            package='creova_state_machine',
            executable='final_detection_subscriber',
            name='final_detection_subscriber',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
            }],
            respawn=True,
            respawn_delay=2.0
        ),

        # Status Logger Node - Logs system status for debugging
        Node(
            package='creova_state_machine',
            executable='status_logger_node',
            name='status_logger_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
            }],
            respawn=True,
            respawn_delay=2.0
        )
    ])
