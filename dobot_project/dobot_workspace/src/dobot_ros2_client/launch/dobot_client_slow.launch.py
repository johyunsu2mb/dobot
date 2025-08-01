#!/usr/bin/env python3
"""
느린 업데이트 주기를 가진 Launch 파일
"""

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
    
    return LaunchDescription([
        # 업데이트 주기 파라미터 (기본값: 1Hz)
        DeclareLaunchArgument(
            'update_rate',
            default_value='1.0',
            description='Status update rate in Hz (lower = less frequent)'
        ),
        
        DeclareLaunchArgument(
            'server_url',
            default_value='http://localhost:8080',
            description='HTTP API server URL'
        ),
        
        DeclareLaunchArgument(
            'log_level',
            default_value='warn',  # 로그 레벨도 낮춤
            description='Logging level'
        ),
        
        Node(
            package='dobot_ros2_client',
            executable='dobot_client_node',
            name='dobot_client',
            parameters=[{
                'server_url': LaunchConfiguration('server_url'),
                'update_rate': LaunchConfiguration('update_rate'),
            }],
            output='screen',
            emulate_tty=True,
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        )
    ])
