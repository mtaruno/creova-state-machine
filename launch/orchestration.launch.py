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
        
        # Navigation Node
        Node(
            package='creova_state_machine',
            executable='navigation_node',
            name='navigation_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
            }],
            remappings=[
                ('nav2_status', '/navigation/status'),
                ('requested_location', '/navigation/goal'),
                ('go_to_location', '/navigation/set_goal'),
                ('pai_details', '/navigation/status_summary')
            ]
        ),
        
        # Perception Node
        Node(
            package='creova_state_machine',
            executable='perception_node',
            name='perception_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
            }],
            remappings=[
                ('object_list', '/perception/objects'),
                ('pick_request', '/perception/pick_request'),
                ('pick_result', '/perception/pick_result')
            ]
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
        
        # Manipulation Node
        Node(
            package='creova_state_machine',
            executable='manipulation_node',
            name='manipulation_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
            }]
        ),

        # Destination Server Node
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