"""
서버만 시작하는 launch 파일
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'tcp_host',
            default_value='127.0.1.1',
            description='TCP server host address'
        ),
        
        DeclareLaunchArgument(
            'tcp_port',
            default_value='9988',
            description='TCP server port'
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
        
        DeclareLaunchArgument(
            'max_robots',
            default_value='10',
            description='Maximum number of robots'
        ),
        
        DeclareLaunchArgument(
            'interactive',
            default_value='true',
            description='Enable interactive mode'
        ),
        
        Node(
            package='dobot_robot_manager',
            executable='dobot_server',
            name='dobot_multi_robot_server',
            output='screen',
            parameters=[{
                'tcp_host': LaunchConfiguration('tcp_host'),
                'tcp_port': LaunchConfiguration('tcp_port'),
                'enable_tcp': LaunchConfiguration('enable_tcp'),
                'enable_ros2': LaunchConfiguration('enable_ros2'),
                'max_robots': LaunchConfiguration('max_robots'),
            }],
            arguments=['--interactive']
        ),
    ])

