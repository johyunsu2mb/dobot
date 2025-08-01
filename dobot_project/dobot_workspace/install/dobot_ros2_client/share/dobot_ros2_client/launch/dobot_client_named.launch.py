#!/usr/bin/env python3
"""
고유 이름을 가진 ROS2 클라이언트 Launch 파일
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
import time
import socket

def generate_launch_description():
    # 고유 이름 생성
    hostname = socket.gethostname()
    timestamp = str(int(time.time()))[-4:]  # 마지막 4자리
    default_name = f"ROS2_{hostname}_{timestamp}"
    
    return LaunchDescription([
        # 클라이언트 이름 파라미터
        DeclareLaunchArgument(
            'client_name',
            default_value=default_name,
            description='Unique client name for TCP connection'
        ),
        
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
            'update_rate',
            default_value='1.0',
            description='Status update rate in Hz'
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
                'update_rate': LaunchConfiguration('update_rate'),
            }],
            output='screen',
            emulate_tty=True,
        )
    ])
