#!/usr/bin/env python3
"""
diagnose_dobot.py - Dobot API 진단 및 설치 도구
Enhanced Dobot Robot & YOLO Object Detection System
"""

import sys
import subprocess
import platform
import os
from pathlib import Path

def print_header():
    """헤더 출력"""
    print("🤖" + "="*60 + "🤖")
    print("     Enhanced Dobot Robot System - API 진단 도구")
    print("🤖" + "="*60 + "🤖")
    print()

def check_python_environment():
    """Python 환경 확인"""
    print("🐍 Python 환경 확인")
    print("-" * 30)
    print(f"Python 버전: {sys.version}")
    print(f"Python 경로: {sys.executable}")
    print(f"플랫폼: {platform.platform()}")
    print(f"아키텍처: {platform.architecture()}")
    print()

def check_dobot_api_options():
    """다양한 Dobot API 옵션 확인"""
    print("🔍 Dobot API 라이브러리 확인")
    print("-" * 30)
    
    api_options = [
        ("dobot_api", "공식 Dobot API"),
        ("pydobot", "PyDobot 라이브러리"),
        ("DobotDllType", "Dobot DLL Type"),
        ("serial", "시리얼 통신 기반"),
    ]
    
    available_apis = []
    
    for module_name, description in api_options:
        try:
            __import__(module_name)
            print(f"✅ {description} ({module_name}) - 설치됨")
            available_apis.append((module_name, description))
        except ImportError:
            print(f"❌ {description} ({module_name}) - 설치 안됨")
    
    print()
    return available_apis

def check_dobot_ports():
    """사용 가능한 시리얼 포트 확인"""
    print("🔌 시리얼 포트 확인")
    print("-" * 30)
    
    try:
        import serial.tools.list_ports
        ports = list(serial.tools.list_ports.comports())
        
        if ports:
            print("사용 가능한 포트:")
            for port in ports:
                print(f"  📡 {port.device} - {port.description}")
                if "dobot" in port.description.lower() or "arduino" in port.description.lower():
                    print(f"    🎯 Dobot 관련 포트일 가능성 높음!")
        else:
            print("❌ 사용 가능한 시리얼 포트가 없습니다.")
    except ImportError:
        print("⚠️ pyserial이 설치되지 않아 포트를 확인할 수 없습니다.")
        print("설치: pip install pyserial")
    
    print()

def test_dobot_connection():
    """Dobot 연결 테스트"""
    print("🔗 Dobot 연결 테스트")
    print("-" * 30)
    
    # dobot_api_handler 사용
    try:
        from dobot_api_handler import DOBOT_API_AVAILABLE, DobotAPIInstaller
        
        if DOBOT_API_AVAILABLE:
            print("✅ Dobot API 핸들러가 API를 찾았습니다.")
            success, message = DobotAPIInstaller.check_dobot_connection()
            print(f"연결 테스트: {message}")
        else:
            print("⚠️ Dobot API가 없습니다. 시뮬레이션 모드로 실행됩니다.")
            
    except ImportError:
        print("❌ dobot_api_handler.py를 찾을 수 없습니다.")
        print("파일이 같은 폴더에 있는지 확인하세요.")
    
    print()

def get_installation_recommendations():
    """설치 권장사항"""
    print("💡 설치 권장사항")
    print("-" * 30)
    
    recommendations = [
        "1. 가장 쉬운 방법:",
        "   pip install pydobot",
        "",
        "2. 공식 Dobot Studio와 함께 설치:",
        "   - https://www.dobot.cc/downloadcenter.html",
        "   - Dobot Studio 다운로드 및 설치",
        "   - Python API 라이브러리 추가 설치",
        "",
        "3. 대체 방법:",
        "   pip install pyserial",
        "   (시리얼 통신 기반)",
        "",
        "4. 설치 후 확인:",
        "   python -c \"import pydobot; print('설치 성공')\"",
        "",
        "5. 하드웨어 연결:",
        "   - USB 케이블로 Dobot 연결",
        "   - 전원 어댑터 연결",
        "   - 드라이버 자동 설치 확인",
        "",
        "⭐ 중요: API 없이도 시뮬레이션 모드로 모든 기능 테스트 가능!"
    ]
    
    for rec in recommendations:
        print(rec)
    print()

def auto_install_pydobot():
    """PyDobot 자동 설치 시도"""
    print("🚀 PyDobot 자동 설치 시도")
    print("-" * 30)
    
    try:
        print("pip install pydobot 실행 중...")
        result = subprocess.run([
            sys.executable, "-m", "pip", "install", "pydobot"
        ], capture_output=True, text=True, timeout=120)
        
        if result.returncode == 0:
            print("✅ PyDobot 설치 성공!")
            print("설치 확인 중...")
            
            # 설치 확인
            try:
                import pydobot
                print("✅ PyDobot 임포트 성공!")
                return True
            except ImportError:
                print("❌ 설치되었지만 임포트 실패")
                return False
        else:
            print(f"❌ 설치 실패: {result.stderr}")
            return False
            
    except subprocess.TimeoutExpired:
        print("⏰ 설치 시간 초과")
        return False
    except Exception as e:
        print(f"❌ 설치 중 오류: {e}")
        return False

def test_system_without_dobot():
    """Dobot 없이 시스템 테스트"""
    print("🧪 시뮬레이션 모드 시스템 테스트")
    print("-" * 30)
    
    try:
        # 기본 모듈들 테스트
        modules_to_test = [
            ("config", "설정 모듈"),
            ("utils", "유틸리티 모듈"),
            ("logger_setup", "로깅 시스템"),
        ]
        
        all_passed = True
        
        for module_name, description in modules_to_test:
            try:
                __import__(module_name)
                print(f"✅ {description} ({module_name}) - 로드 성공")
            except ImportError as e:
                print(f"❌ {description} ({module_name}) - 로드 실패: {e}")
                all_passed = False
        
        # 시뮬레이션 모드 테스트
        try:
            from robot_controller import RobotController
            from config import RobotConfig
            
            controller = RobotController(RobotConfig())
            controller.connect()  # 시뮬레이션 모드로 연결
            
            # 간단한 이동 테스트
            success = controller.move_to_position([100, 100, -100, 0])
            if success:
                print("✅ 시뮬레이션 모드 이동 테스트 성공")
            else:
                print("❌ 시뮬레이션 모드 이동 테스트 실패")
                all_passed = False
                
            controller.disconnect()
            
        except Exception as e:
            print(f"❌ 시뮬레이션 테스트 실패: {e}")
            all_passed = False
        
        if all_passed:
            print("🎉 모든 시뮬레이션 테스트 통과!")
            print("Dobot API 없이도 시스템이 정상 작동합니다.")
        else:
            print("⚠️ 일부 테스트가 실패했습니다.")
        
        return all_passed
        
    except Exception as e:
        print(f"❌ 시스템 테스트 중 오류: {e}")
        return False

def main():
    """메인 진단 프로세스"""
    print_header()
    
    # 1. Python 환경 확인
    check_python_environment()
    
    # 2. Dobot API 옵션 확인
    available_apis = check_dobot_api_options()
    
    # 3. 시리얼 포트 확인
    check_dobot_ports()
    
    # 4. 연결 테스트
    test_dobot_connection()
    
    # 5. 설치 권장사항
    get_installation_recommendations()
    
    # 6. 자동 설치 제안
    if not available_apis:
        print("❓ PyDobot 자동 설치를 시도하시겠습니까? (y/n): ", end="")
        try:
            user_input = input().lower().strip()
            if user_input in ['y', 'yes', '예', 'ㅇ']:
                success = auto_install_pydobot()
                if success:
                    print("🎉 설치 완료! 시스템을 다시 실행해보세요.")
                else:
                    print("⚠️ 자동 설치 실패. 수동 설치를 시도해보세요.")
        except KeyboardInterrupt:
            print("\n설치 취소됨.")
    
    # 7. 시뮬레이션 모드 테스트
    print()
    test_system_without_dobot()
    
    # 8. 최종 요약
    print("\n" + "="*60)
    print("📋 진단 요약")
    print("-" * 30)
    
    if available_apis:
        print("✅ Dobot API 사용 가능 - 실제 로봇 제어 모드")
        print(f"   사용 가능한 API: {', '.join([api[0] for api in available_apis])}")
    else:
        print("⚠️ Dobot API 없음 - 시뮬레이션 모드")
        print("   pip install pydobot 로 설치하거나")
        print("   시뮬레이션 모드로 테스트 가능")
    
    print("\n🚀 시스템 실행 방법:")
    print("   python main.py")
    print("\n💡 팁: Dobot API가 없어도 모든 기능을 테스트할 수 있습니다!")
    print("="*60)

if __name__ == "__main__":
    main()
