#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 파라미터 파일 경로
    pkg_share = get_package_share_directory('dobot_ros2_client')
    params_file = os.path.join(pkg_share, 'config', 'dobot_params.yaml')
    
    # 파라미터 파일이 없으면 기본값 사용
    if not os.path.exists(params_file):
        params_file = None
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'server_url',
            default_value='http://localhost:8080',
            description='HTTP API server URL'
        ),
        
        DeclareLaunchArgument(
            'tcp_host',
            default_value='127.0.0.1',
            description='TCP server host'
        ),
        
        DeclareLaunchArgument(
            'tcp_port',
            default_value='9988',
            description='TCP server port'
        ),
        
        DeclareLaunchArgument(
            'client_name',
            default_value='ROS2_Client',
            description='Client name for TCP connection'
        ),
        
        Node(
            package='dobot_ros2_client',
            executable='dobot_client_node',
            name='dobot_client',
            parameters=[{
                'server_url': LaunchConfiguration('server_url'),
                'tcp_server_host': LaunchConfiguration('tcp_host'),
                'tcp_server_port': LaunchConfiguration('tcp_port'),
                'client_name': LaunchConfiguration('client_name'),
            }] if params_file is None else [params_file],
            output='screen',
            emulate_tty=True,
        )
    ])
