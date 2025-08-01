"""
클라이언트만 시작하는 launch 파일
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_id',
            default_value='robot_01',
            description='Robot ID'
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
            description='Communication mode: tcp_only, ros2_only, hybrid'
        ),
        
        DeclareLaunchArgument(
            'enable_tcp',
            default_value='true',
            description='Enable TCP communication'
        ),
        
        DeclareLaunchArgument(
            'enable_ros2',
            default_value='true',
            description='Enable ROS2 communication'
        ),
        
        Node(
            package='dobot_robot_manager',
            executable='dobot_client',
            name=['dobot_client_', LaunchConfiguration('robot_id')],
            output='screen',
            parameters=[{
                'robot_id': LaunchConfiguration('robot_id'),
                'server_host': LaunchConfiguration('server_host'),
                'server_port': LaunchConfiguration('server_port'),
                'communication_mode': LaunchConfiguration('communication_mode'),
                'enable_tcp': LaunchConfiguration('enable_tcp'),
                'enable_ros2': LaunchConfiguration('enable_ros2'),
            }]
        ),
    ])

