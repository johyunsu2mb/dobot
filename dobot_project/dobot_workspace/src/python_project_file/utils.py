"""
utils.py - 유틸리티 함수들 및 에러 클래스들
Enhanced Dobot Robot & YOLO Object Detection System
"""

import os
import platform
import tkinter.font as tkFont
from typing import List
from config import WorkspaceLimit

# ——————————————————————————————
# 에러 클래스 정의
# ——————————————————————————————
class RobotSystemError(Exception):
    """로봇 시스템 기본 예외"""
    pass

class RobotConnectionError(RobotSystemError):
    """로봇 연결 관련 예외"""
    pass

class RobotMovementError(RobotSystemError):
    """로봇 이동 관련 예외"""
    pass

class InvalidPositionError(RobotSystemError):
    """잘못된 위치 관련 예외"""
    pass

class GripperError(RobotSystemError):
    """그리퍼 제어 관련 예외"""
    pass

class TimeoutError(RobotSystemError):
    """타임아웃 관련 예외"""
    pass

# ——————————————————————————————
# 유틸리티 함수들
# ——————————————————————————————
def validate_position(position: List[float], workspace: WorkspaceLimit = WorkspaceLimit()) -> bool:
    """위치 좌표 유효성 검증"""
    if len(position) != 4:
        return False
    
    x, y, z, r = position
    return (
        workspace.x_min <= x <= workspace.x_max and
        workspace.y_min <= y <= workspace.y_max and
        workspace.z_min <= z <= workspace.z_max and
        workspace.r_min <= r <= workspace.r_max
    )

def safe_float_conversion(value: str, default: float = 0.0) -> float:
    """안전한 float 변환"""
    try:
        return float(value)
    except (ValueError, TypeError):
        return default

def darken_color(color: str) -> str:
    """색상을 어둡게 만드는 헬퍼 함수"""
    color_map = {
        "#e74c3c": "#c0392b",
        "#3498db": "#2980b9", 
        "#2ecc71": "#27ae60",
        "#9b59b6": "#8e44ad"
    }
    return color_map.get(color, color)

# ——————————————————————————————
# 폰트 관련 함수들
# ——————————————————————————————
def register_font_file():
    """폰트 파일 등록 함수"""
    font_paths = ["fonts/NanumGothic.otf", "fonts/NanumGothic.ttf", "./NanumGothic.otf", "./NanumGothic.ttf"]
   
    font_file = None
    for path in font_paths:
        if os.path.exists(path):
            font_file = path
            print(f"폰트 파일 발견: {path}")
            break
   
    if font_file:
        try:
            if platform.system() == "Windows":
                import ctypes
                gdi32 = ctypes.windll.gdi32
                FR_PRIVATE = 0x10
                result = gdi32.AddFontResourceExW(
                    ctypes.c_wchar_p(os.path.abspath(font_file)),
                    ctypes.c_ulong(FR_PRIVATE),
                    ctypes.c_void_p(0)
                )
               
                if result:
                    print(f"Windows에 폰트 임시 등록 성공: {font_file}")
                    return get_font_family_name(font_file) or "Arial"
                   
            return "Arial"
               
        except Exception as e:
            print(f"폰트 등록 실패: {e}")
            return get_system_korean_font()
   
    return get_system_korean_font()

def get_font_family_name(font_path):
    """폰트 파일에서 family name 추출"""
    try:
        if "NanumGothic" in font_path or "Nanum" in font_path:
            return "NanumGothic"
        elif "malgun" in font_path.lower():
            return "Malgun Gothic"
        elif "Noto" in font_path:
            return "Noto Sans CJK KR"
        return None
    except:
        return None

def get_system_korean_font():
    """시스템에 맞는 한글 폰트를 찾아서 반환"""
    system = platform.system()
   
    test_fonts = ["NanumGothic", "Nanum Gothic"]
   
    if system == "Windows":
        fonts = test_fonts + ["Malgun Gothic", "Arial Unicode MS", "Gulim", "Arial"]
    elif system == "Darwin":  # macOS
        fonts = test_fonts + ["AppleGothic", "Apple SD Gothic Neo", "Helvetica", "Arial"]
    else:  # Linux
        fonts = test_fonts + ["Noto Sans CJK KR", "DejaVu Sans", "Liberation Sans", "Arial"]
   
    for font in fonts:
        try:
            test_font = tkFont.Font(family=font, size=12)
            actual_family = test_font.actual()['family']
           
            if font.lower() in actual_family.lower() or \
               any(name.lower() in actual_family.lower() for name in ["nanum", "gothic", "malgun"]):
                print(f"사용할 폰트: {font} (실제: {actual_family})")
                return font
               
        except Exception:
            continue
   
    print("적절한 한글 폰트를 찾을 수 없어 기본 폰트 사용")
    return "Arial"

def create_font_directory():
    """fonts 디렉터리 생성 및 안내"""
    if not os.path.exists("./fonts"):
        os.makedirs("./fonts")
        print("\n=== 폰트 설정 안내 ===")
        print("1. 'fonts' 폴더가 생성되었습니다.")
        print("2. NanumGothic.otf 또는 NanumGothic.ttf 파일을 fonts 폴더에 넣으면")
        print("   더 나은 한글 표시가 가능합니다.")
        print("====================\n")
