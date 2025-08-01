"""
Dobot Multi-Robot Management System

ROS2 패키지로 구성된 다중 로봇 관리 시스템
TCP와 ROS2 통신을 모두 지원합니다.
"""

__version__ = "1.0.0"
__author__ = "johyunsu2mb"
__email__ = "johyunsu61@gmail.com"

# 주요 클래스 임포트
try:
    from .server_node import DobotMultiRobotServerNode
    from .client_node import DobotROS2ClientNode
    from .utils import (
        TaskStatus, CommunicationType, Robot, Task,
        create_robot_command, parse_robot_status, validate_position
    )
    from .interfaces import (
        RobotCommandMsg, RobotStatusMsg, TaskRequestMsg, SystemStatusMsg
    )
except ImportError:
    # 모듈들이 아직 빌드되지 않았을 때
    pass

# 패키지 정보
PACKAGE_INFO = {
    'name': 'dobot_robot_manager',
    'version': __version__,
    'description': 'Multi-robot management system for Dobot robots',
    'author': __author__,
    'email': __email__,
    'license': 'MIT',
    'supported_robots': ['Dobot Magician', 'Dobot M1', 'Dobot CR series'],
    'communication': ['TCP', 'ROS2', 'Hybrid'],
    'features': [
        'Multi-robot coordination',
        'Task queue management', 
        'Real-time monitoring',
        'Emergency stop',
        'YOLO object detection',
        'Automatic reconnection'
    ]
}