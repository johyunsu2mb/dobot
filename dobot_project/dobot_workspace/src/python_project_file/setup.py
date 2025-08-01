"""
setup.py - 설치 스크립트
Enhanced Dobot Robot & YOLO Object Detection System
"""

import os
import sys
import subprocess
import platform
from pathlib import Path

def check_python_version():
    """Python 버전 확인"""
    print("🐍 Python 버전 확인 중...")
    if sys.version_info < (3, 8):
        print("❌ Python 3.8 이상이 필요합니다.")
        print(f"현재 버전: {sys.version}")
        return False
    else:
        print(f"✅ Python {sys.version} 확인됨")
        return True

def install_requirements():
    """필수 패키지 설치"""
    print("\n📦 필수 패키지 설치 중...")
    
    # 기본 필수 패키지
    basic_requirements = [
        "numpy>=1.21.0",
        "matplotlib>=3.5.0", 
        "opencv-python>=4.5.0",
        "ultralytics>=8.0.0",
        "pillow>=8.0.0",
        "pyyaml>=6.0"
    ]
    
    # Dobot 관련 패키지 (선택사항)
    dobot_requirements = [
        "pydobot",
        "pyserial>=3.5"
    ]
    
    failed_packages = []
    
    # 기본 패키지 설치
    for package in basic_requirements:
        try:
            print(f"  설치 중: {package}")
            result = subprocess.run([
                sys.executable, "-m", "pip", "install", package
            ], capture_output=True, text=True, timeout=300)
            
            if result.returncode == 0:
                print(f"  ✅ {package} 설치 완료")
            else:
                print(f"  ❌ {package} 설치 실패")
                failed_packages.append(package)
                
        except subprocess.TimeoutExpired:
            print(f"  ⏰ {package} 설치 타임아웃")
            failed_packages.append(package)
        except Exception as e:
            print(f"  ❌ {package} 설치 중 오류: {e}")
            failed_packages.append(package)
    
    # Dobot API 설치 (선택사항)
    print("\n🤖 Dobot API 설치 (선택사항)")
    print("실제 로봇 제어를 위해 권장되지만, 없어도 시뮬레이션 모드로 실행 가능")
    
    dobot_installed = False
    for package in dobot_requirements:
        try:
            print(f"  시도 중: {package}")
            result = subprocess.run([
                sys.executable, "-m", "pip", "install", package
            ], capture_output=True, text=True, timeout=180)
            
            if result.returncode == 0:
                print(f"  ✅ {package} 설치 완료")
                dobot_installed = True
                break  # 하나라도 성공하면 중단
            else:
                print(f"  ⚠️ {package} 설치 실패 (계속 진행)")
                
        except Exception as e:
            print(f"  ⚠️ {package} 설치 중 오류: {e} (계속 진행)")
    
    if not dobot_installed:
        print("  ℹ️ Dobot API 설치 실패 - 시뮬레이션 모드로 실행됩니다")
        print("  💡 나중에 'pip install pydobot'로 설치할 수 있습니다")
    
    return failed_packages

def create_directories():
    """필요한 디렉터리 생성"""
    print("\n📁 디렉터리 생성 중...")
    
    directories = ["logs", "fonts", "models", "data"]
    
    for directory in directories:
        path = Path(directory)
        if not path.exists():
            path.mkdir(parents=True, exist_ok=True)
            print(f"  ✅ {directory} 디렉터리 생성됨")
        else:
            print(f"  📁 {directory} 디렉터리 이미 존재")

def download_yolo_model():
    """YOLO 모델 다운로드"""
    print("\n🎯 YOLO 모델 다운로드 중...")
    
    try:
        import ultralytics
        from ultralytics import YOLO
        
        print("  YOLOv8 모델 초기화 중...")
        model = YOLO("yolov8n.pt")  # 가장 작은 모델부터 다운로드
        print("  ✅ YOLOv8 모델 다운로드 완료")
        return True
        
    except ImportError:
        print("  ❌ ultralytics 패키지가 설치되지 않음")
        return False
    except Exception as e:
        print(f"  ❌ YOLO 모델 다운로드 실패: {e}")
        return False

def setup_fonts():
    """폰트 설정 안내"""
    print("\n🔤 폰트 설정 안내")
    
    fonts_dir = Path("fonts")
    nanum_files = list(fonts_dir.glob("*Nanum*"))
    
    if nanum_files:
        print(f"  ✅ 나눔고딕 폰트 발견: {nanum_files[0]}")
    else:
        print("  📝 나눔고딕 폰트 설치 권장:")
        print("     1. https://hangeul.naver.com/2017/nanum 에서 다운로드")
        print("     2. NanumGothic.ttf 파일을 fonts 폴더에 복사")
        print("     3. 더 나은 한글 표시를 위해 권장됨")

def check_camera():
    """카메라 접근 확인"""
    print("\n📷 카메라 접근 확인 중...")
    
    try:
        import cv2
        cap = cv2.VideoCapture(0)
        
        if cap.isOpened():
            print("  ✅ 카메라 접근 가능")
            cap.release()
            return True
        else:
            print("  ⚠️ 카메라에 접근할 수 없음 (선택사항)")
            return False
            
    except ImportError:
        print("  ❌ OpenCV가 설치되지 않음")
        return False
    except Exception as e:
        print(f"  ❌ 카메라 확인 중 오류: {e}")
        return False

def create_run_script():
    """실행 스크립트 생성"""
    print("\n📜 실행 스크립트 생성 중...")
    
    if platform.system() == "Windows":
        script_content = """@echo off
echo Enhanced Dobot Robot System Starting...
python main.py
pause
"""
        with open("run.bat", "w", encoding="utf-8") as f:
            f.write(script_content)
        print("  ✅ run.bat 생성됨")
        
    else:
        script_content = """#!/bin/bash
echo "Enhanced Dobot Robot System Starting..."
python3 main.py
"""
        with open("run.sh", "w") as f:
            f.write(script_content)
        os.chmod("run.sh", 0o755)
        print("  ✅ run.sh 생성됨")

def run_basic_test():
    """기본 기능 테스트"""
    print("\n🧪 기본 기능 테스트 중...")
    
    try:
        # 설정 파일 임포트 테스트
        import config
        print("  ✅ config.py 로드 성공")
        
        # 로거 설정 테스트
        from logger_setup import setup_logging
        logger = setup_logging()
        logger.info("설치 테스트 로그")
        print("  ✅ logger_setup.py 동작 확인")
        
        # 유틸리티 함수 테스트
        from utils import validate_position
        test_result = validate_position([100, 100, -100, 0])
        print(f"  ✅ utils.py 동작 확인 (테스트 결과: {test_result})")
        
        print("  🎉 모든 기본 기능 테스트 통과!")
        return True
        
    except Exception as e:
        print(f"  ❌ 기본 기능 테스트 실패: {e}")
        return False

def main():
    """메인 설치 프로세스"""
    print("🤖 Enhanced Dobot Robot & YOLO System 설치 시작")
    print("=" * 60)
    
    # 1. Python 버전 확인
    if not check_python_version():
        sys.exit(1)
    
    # 2. 디렉터리 생성
    create_directories()
    
    # 3. 패키지 설치
    failed_packages = install_requirements()
    
    if failed_packages:
        print(f"\n⚠️ 다음 패키지 설치 실패: {failed_packages}")
        print("수동으로 설치를 시도해보세요:")
        for package in failed_packages:
            print(f"  pip install {package}")
    
    # 4. YOLO 모델 다운로드
    download_yolo_model()
    
    # 5. 폰트 설정 안내
    setup_fonts()
    
    # 6. 카메라 확인
    check_camera()
    
    # 7. 실행 스크립트 생성
    create_run_script()
    
    # 8. 기본 테스트
    test_passed = run_basic_test()
    
    # 9. 설치 완료 메시지
    print("\n" + "=" * 60)
    if test_passed and not failed_packages:
        print("🎉 설치가 성공적으로 완료되었습니다!")
        print("\n실행 방법:")
        print("  python main.py")
        if platform.system() == "Windows":
            print("  또는 run.bat 더블클릭")
        else:
            print("  또는 ./run.sh")
    else:
        print("⚠️ 설치가 부분적으로 완료되었습니다.")
        print("문제가 있는 부분을 확인하고 수동으로 해결해주세요.")
    
    print("\n추가 정보:")
    print("  - README.md 파일을 참조하세요")
    print("  - logs/ 폴더에서 로그를 확인할 수 있습니다")
    print("  - 실제 Dobot 로봇 없이도 시뮬레이션 모드로 실행 가능합니다")
    
    # Dobot API 진단 안내
    print("\n🤖 Dobot API 문제가 있는 경우:")
    print("  python diagnose_dobot.py  # 진단 도구 실행")
    print("  pip install pydobot       # 권장 API 설치")

if __name__ == "__main__":
    main()
