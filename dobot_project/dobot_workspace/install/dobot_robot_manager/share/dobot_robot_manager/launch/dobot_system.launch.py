"""
전체 Dobot 다중 로봇 시스템을 시작하는 launch 파일
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    return LaunchDescription([
        # Launch 인수 선언
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
            'server_host',
            default_value='127.0.1.1',
            description='TCP server host'
        ),
        DeclareLaunchArgument(
            'server_port',
            default_value='9988',
            description='TCP server port'
        ),
        DeclareLaunchArgument(
            'max_robots',
            default_value='5',
            description='Maximum number of robots'
        ),
        DeclareLaunchArgument(
            'interactive',
            default_value='false',
            description='Enable interactive server mode'
        ),
        
        # 서버 노드 시작
        Node(
            package='dobot_robot_manager',
            executable='dobot_server',
            name='dobot_multi_robot_server',
            output='screen',
            parameters=[{
                'tcp_host': LaunchConfiguration('server_host'),
                'tcp_port': LaunchConfiguration('server_port'),
                'enable_tcp': LaunchConfiguration('enable_tcp'),
                'enable_ros2': LaunchConfiguration('enable_ros2'),
                'max_robots': LaunchConfiguration('max_robots'),
            }],
            arguments=['--interactive'],
            condition=IfCondition(LaunchConfiguration('interactive'))
        ),
        
        Node(
            package='dobot_robot_manager',
            executable='dobot_server',
            name='dobot_multi_robot_server',
            output='screen',
            parameters=[{
                'tcp_host': LaunchConfiguration('server_host'),
                'tcp_port': LaunchConfiguration('server_port'),
                'enable_tcp': LaunchConfiguration('enable_tcp'),
                'enable_ros2': LaunchConfiguration('enable_ros2'),
                'max_robots': LaunchConfiguration('max_robots'),
            }],
            condition=UnlessCondition(LaunchConfiguration('interactive'))
        ),
        
        # 시스템 정보 출력
        LogInfo(
            msg=[
                'Dobot Multi-Robot System Starting...\n',
                'Server Host: ', LaunchConfiguration('server_host'), '\n',
                'Server Port: ', LaunchConfiguration('server_port'), '\n',
                'TCP Enabled: ', LaunchConfiguration('enable_tcp'), '\n',
                'ROS2 Enabled: ', LaunchConfiguration('enable_ros2'), '\n',
                'Max Robots: ', LaunchConfiguration('max_robots')
            ]
        ),
    ])

