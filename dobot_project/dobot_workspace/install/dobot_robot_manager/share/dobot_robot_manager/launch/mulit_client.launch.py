"""
다중 클라이언트를 시작하는 launch 파일
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'num_robots',
            default_value='3',
            description='Number of robot clients to start'
        ),
        
        DeclareLaunchArgument(
            'server_host',
            default_value='127.0.1.1',
            description='Server host address'
        ),
        
        DeclareLaunchArgument(
            'server_port',
            default_value='9988',
            description='Server port'
        ),
        
        DeclareLaunchArgument(
            'communication_mode',
            default_value='hybrid',
            description='Communication mode for all clients'
        ),
        
        # Robot 1
        Node(
            package='dobot_robot_manager',
            executable='dobot_client',
            name='dobot_client_robot_01',
            output='screen',
            parameters=[{
                'robot_id': 'robot_01',
                'server_host': LaunchConfiguration('server_host'),
                'server_port': LaunchConfiguration('server_port'),
                'communication_mode': LaunchConfiguration('communication_mode'),
            }]
        ),
        
        # Robot 2
        Node(
            package='dobot_robot_manager',
            executable='dobot_client',
            name='dobot_client_robot_02',
            output='screen',
            parameters=[{
                'robot_id': 'robot_02',
                'server_host': LaunchConfiguration('server_host'),
                'server_port': LaunchConfiguration('server_port'),
                'communication_mode': LaunchConfiguration('communication_mode'),
            }]
        ),
        
        # Robot 3
        Node(
            package='dobot_robot_manager',
            executable='dobot_client',
            name='dobot_client_robot_03',
            output='screen',
            parameters=[{
                'robot_id': 'robot_03',
                'server_host': LaunchConfiguration('server_host'),
                'server_port': LaunchConfiguration('server_port'),
                'communication_mode': LaunchConfiguration('communication_mode'),
            }]
        ),
    ])