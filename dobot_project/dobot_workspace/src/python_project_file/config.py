"""
config.py - 설정 및 상수 정의
Enhanced Dobot Robot & YOLO Object Detection System
"""

from dataclasses import dataclass
from enum import Enum
from typing import Dict, Any

@dataclass
class RobotConfig:
    """로봇 설정 (통신 안정성 강화)"""
    ip_address: str = "192.168.1.6"
    dashboard_port: int = 29999
    move_port: int = 30003
    feed_port: int = 30004
    movement_timeout: float = 45.0  # 타임아웃 시간 증가 (30 -> 45초)
    position_tolerance: float = 2.0  # 위치 허용 오차 증가 (1.0 -> 2.0)
    gripper_delay: float = 1.5  # 그리퍼 딜레이 증가 (1.0 -> 1.5초)
    safety_height_offset: float = 50.0
    connection_check_interval: float = 5.0  # 연결 상태 확인 간격
    max_retry_count: int = 3  # 최대 재시도 횟수
    retry_delay: float = 1.0  # 재시도 간격

@dataclass
class WorkspaceLimit:
    """작업 공간 제한"""
    x_min: float = -400
    x_max: float = 400
    y_min: float = -400
    y_max: float = 400
    z_min: float = -200
    z_max: float = 200
    r_min: float = -180
    r_max: float = 180

class RobotStatus(Enum):
    """로봇 상태"""
    IDLE = "대기"
    MOVING = "이동 중"
    PICKING = "픽업 중"
    CARRYING = "운반 중"
    PLACING = "배치 중"
    ERROR = "오류"

# 가구 정보 딕셔너리
FURNITURE_INFO = {
    "소파": {
        "emoji": "[SOFA]",
        "price": 150000,
        "time": 3.0,
        "color": "#e74c3c",
        "position": [290, 218, -129, -59]
    },
    "의자": {
        "emoji": "[CHAIR]", 
        "price": 45000,
        "time": 2.0,
        "color": "#3498db",
        "position": [292, -188, -115, -59]
    },
    "책상": {
        "emoji": "[DESK]",
        "price": 120000,
        "time": 2.5,
        "color": "#2ecc71",
        "position": [141, -248, -127, -59]
    },
    "침대": {
        "emoji": "[BED]",
        "price": 250000,
        "time": 4.0,
        "color": "#9b59b6",
        "position": [133, 249, -147, -59]
    }
}

# UI 색상 설정
UI_COLORS = {
    'primary_bg': '#2c3e50',
    'secondary_bg': '#34495e',
    'text_primary': '#ecf0f1',
    'text_secondary': '#95a5a6',
    'success': '#2ecc71',
    'warning': '#f39c12',
    'error': '#e74c3c',
    'info': '#3498db'
}

# 로봇팔 설정
ROBOT_ARM_CONFIG = {
    'link_lengths': [10, 20, 20, 15, 10],
    'base_position': [300, -30, 5, 0],
    'final_position': [350, 0, 0, 0]
}

# 의존성 체크 결과를 저장할 전역 변수
DEPENDENCIES = {
    'DOBOT_AVAILABLE': False,
    'YOLO_AVAILABLE': False,
    'CV2_AVAILABLE': False,
    'PIL_AVAILABLE': False
}

def check_dependencies():
    """의존성 패키지 확인"""
    global DEPENDENCIES
    
    # YOLO 및 OpenCV 체크
    try:
        from ultralytics import YOLO
        import cv2
        DEPENDENCIES['CV2_AVAILABLE'] = True
        DEPENDENCIES['YOLO_AVAILABLE'] = True
        print("✅ YOLOv8 및 OpenCV 사용 가능")
    except ImportError as e:
        DEPENDENCIES['CV2_AVAILABLE'] = False
        DEPENDENCIES['YOLO_AVAILABLE'] = False
        print(f"⚠️ YOLO/OpenCV 임포트 실패: {e}")

    # PIL 체크
    try:
        from PIL import Image, ImageTk, ImageFont, ImageDraw
        DEPENDENCIES['PIL_AVAILABLE'] = True
        print("✅ PIL 사용 가능")
    except ImportError:
        print("⚠️ PIL을 찾을 수 없습니다. 일부 이미지 기능이 비활성화됩니다.")
        DEPENDENCIES['PIL_AVAILABLE'] = False

    # Dobot API 체크 (향상된 핸들러 사용)
    try:
        from dobot_api_handler import DOBOT_API_AVAILABLE
        DEPENDENCIES['DOBOT_AVAILABLE'] = DOBOT_API_AVAILABLE
        if DOBOT_API_AVAILABLE:
            print("✅ Dobot API 사용 가능")
        else:
            print("⚠️ Dobot API 없음 - 시뮬레이션 모드로 실행됩니다.")
    except ImportError:
        DEPENDENCIES['DOBOT_AVAILABLE'] = False
        print("⚠️ Dobot API 핸들러를 찾을 수 없습니다. 기본 시뮬레이션 모드로 실행됩니다.")

# 초기화 시 의존성 체크 실행
check_dependencies()
