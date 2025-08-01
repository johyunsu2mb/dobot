#!/usr/bin/env python3
"""
integration_test.py - 통합 테스트 스크립트
Enhanced Dobot Robot & YOLO Object Detection System

전체 시스템의 통합 테스트를 수행하고 문제점을 진단합니다.
"""

import sys
import os
import time
import threading
import traceback
from datetime import datetime
from pathlib import Path

# 테스트 결과 저장
test_results = {
    'passed': [],
    'failed': [],
    'warnings': [],
    'skipped': []
}

def print_header():
    """테스트 헤더 출력"""
    print("🧪" + "="*70 + "🧪")
    print("    Enhanced Dobot Robot System - 통합 테스트")
    print("🧪" + "="*70 + "🧪")
    print(f"테스트 시작 시간: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print()

def test_result(test_name: str, passed: bool, message: str = "", warning: bool = False):
    """테스트 결과 기록"""
    if warning:
        status = "⚠️ WARNING"
        test_results['warnings'].append((test_name, message))
    elif passed:
        status = "✅ PASS"
        test_results['passed'].append(test_name)
    else:
        status = "❌ FAIL"
        test_results['failed'].append((test_name, message))
    
    print(f"{status:<12} {test_name}")
    if message:
        print(f"             {message}")

def test_skip(test_name: str, reason: str):
    """테스트 스킵 기록"""
    print(f"⏭️ SKIP       {test_name}")
    print(f"             {reason}")
    test_results['skipped'].append((test_name, reason))

class IntegrationTest:
    """통합 테스트 클래스"""
    
    def __init__(self):
        self.start_time = time.time()
        
    def test_python_environment(self):
        """Python 환경 테스트"""
        print("🐍 Python 환경 테스트")
        print("-" * 40)
        
        # Python 버전 확인
        try:
            version = sys.version_info
            if version >= (3, 8):
                test_result("Python 버전", True, f"{version.major}.{version.minor}.{version.micro}")
            else:
                test_result("Python 버전", False, f"Python 3.8+ 필요, 현재: {version.major}.{version.minor}")
        except Exception as e:
            test_result("Python 버전", False, str(e))
        
        # 필수 내장 모듈 확인
        builtin_modules = ['tkinter', 'threading', 'logging', 'json', 'os', 'sys']
        for module in builtin_modules:
            try:
                __import__(module)
                test_result(f"내장 모듈 {module}", True)
            except ImportError as e:
                test_result(f"내장 모듈 {module}", False, str(e))
        
        print()
    
    def test_file_structure(self):
        """파일 구조 테스트"""
        print("📁 파일 구조 테스트")
        print("-" * 40)
        
        required_files = [
            "main.py",
            "config.py", 
            "logger_setup.py",
            "utils.py",
            "robot_controller.py",
            "dobot_api_handler.py",
            "yolo_detector.py",
            "ui_components.py",
            "requirements.txt"
        ]
        
        for file in required_files:
            if Path(file).exists():
                test_result(f"파일 {file}", True)
            else:
                test_result(f"파일 {file}", False, "파일이 존재하지 않음")
        
        # 디렉터리 생성 테스트
        directories = ["logs", "fonts", "models", "data"]
        for directory in directories:
            try:
                Path(directory).mkdir(exist_ok=True)
                test_result(f"디렉터리 {directory}", True)
            except Exception as e:
                test_result(f"디렉터리 {directory}", False, str(e))
        
        print()
    
    def test_basic_imports(self):
        """기본 모듈 임포트 테스트"""
        print("📦 기본 모듈 임포트 테스트")
        print("-" * 40)
        
        # 로컬 모듈 임포트 테스트
        local_modules = [
            ("config", "설정 모듈"),
            ("logger_setup", "로깅 설정"),
            ("utils", "유틸리티"),
            ("dobot_api_handler", "Dobot API 핸들러"),
            ("robot_controller", "로봇 컨트롤러"),
            ("yolo_detector", "YOLO 검출기"),
            ("ui_components", "UI 컴포넌트")
        ]
        
        for module_name, description in local_modules:
            try:
                module = __import__(module_name)
                test_result(f"{description} ({module_name})", True)
            except ImportError as e:
                test_result(f"{description} ({module_name})", False, str(e))
            except Exception as e:
                test_result(f"{description} ({module_name})", False, f"임포트 오류: {e}")
        
        print()
    
    def test_external_packages(self):
        """외부 패키지 테스트"""
        print("📚 외부 패키지 테스트")
        print("-" * 40)
        
        # 필수 패키지
        essential_packages = [
            ("numpy", "NumPy"),
            ("matplotlib", "Matplotlib"),
            ("yaml", "PyYAML")
        ]
        
        for package, name in essential_packages:
            try:
                __import__(package)
                test_result(f"{name} 패키지", True)
            except ImportError:
                test_result(f"{name} 패키지", False, "설치되지 않음")
        
        # 선택적 패키지
        optional_packages = [
            ("cv2", "OpenCV"),
            ("PIL", "Pillow"),
            ("ultralytics", "YOLOv8")
        ]
        
        for package, name in optional_packages:
            try:
                __import__(package)
                test_result(f"{name} 패키지", True)
            except ImportError:
                test_result(f"{name} 패키지", False, "설치되지 않음 (선택사항)", warning=True)
        
        print()
    
    def test_dobot_api_integration(self):
        """Dobot API 통합 테스트"""
        print("🤖 Dobot API 통합 테스트")
        print("-" * 40)
        
        try:
            from dobot_api_handler import (
                DOBOT_API_AVAILABLE, DobotApiDashboard, DobotApiMove, 
                DobotApi, check_dobot_api, diagnose_dobot_setup
            )
            
            test_result("Dobot API 핸들러 로드", True)
            
            # API 가용성 확인
            if DOBOT_API_AVAILABLE:
                test_result("Dobot API 감지", True, "실제 API 사용 가능")
            else:
                test_result("Dobot API 감지", True, "시뮬레이션 모드 (정상)", warning=True)
            
            # 더미 클래스 테스트
            try:
                dashboard = DobotApiDashboard("192.168.1.6", 29999)
                move = DobotApiMove("192.168.1.6", 30003)
                api = DobotApi("192.168.1.6", 30004)
                
                test_result("Dobot API 클래스 인스턴스", True)
                
                # 기본 메서드 테스트
                dashboard.EnableRobot()
                dashboard.DisableRobot()
                dashboard.DO(1, 1)
                move.MovL(100, 100, -100, 0)
                
                test_result("Dobot API 메서드 호출", True)
                
            except Exception as e:
                test_result("Dobot API 클래스 테스트", False, str(e))
            
        except ImportError as e:
            test_result("Dobot API 핸들러", False, str(e))
        except Exception as e:
            test_result("Dobot API 핸들러", False, f"예상치 못한 오류: {e}")
        
        print()
    
    def test_robot_controller(self):
        """로봇 컨트롤러 테스트"""
        print("🎮 로봇 컨트롤러 테스트")
        print("-" * 40)
        
        try:
            from robot_controller import RobotController
            from config import RobotConfig
            
            test_result("로봇 컨트롤러 임포트", True)
            
            # 컨트롤러 인스턴스 생성
            config = RobotConfig()
            controller = RobotController(config)
            
            test_result("로봇 컨트롤러 인스턴스", True)
            
            # 연결 테스트 (시뮬레이션)
            try:
                controller.connect()
                test_result("로봇 연결 (시뮬레이션)", True)
            except Exception as e:
                test_result("로봇 연결", False, str(e))
            
            # 기본 동작 테스트
            try:
                # 위치 이동 테스트
                success = controller.move_to_position([100, 100, -100, 0])
                test_result("위치 이동 테스트", success)
                
                # 그리퍼 제어 테스트
                success = controller.control_gripper(True)
                test_result("그리퍼 제어 테스트", success)
                
                # 연결 해제
                controller.disconnect()
                test_result("로봇 연결 해제", True)
                
            except Exception as e:
                test_result("로봇 기본 동작", False, str(e))
            
        except ImportError as e:
            test_result("로봇 컨트롤러", False, f"임포트 실패: {e}")
        except Exception as e:
            test_result("로봇 컨트롤러", False, f"예상치 못한 오류: {e}")
        
        print()
    
    def test_yolo_detector(self):
        """YOLO 검출기 테스트"""
        print("👁️ YOLO 검출기 테스트")
        print("-" * 40)
        
        try:
            from yolo_detector import YOLODetector
            test_result("YOLO 검출기 임포트", True)
            
            # 검출기 인스턴스 생성
            detector = YOLODetector()
            test_result("YOLO 검출기 인스턴스", True)
            
            # 모델 로드 상태 확인
            if detector.model_loaded:
                test_result("YOLO 모델 로드", True)
                
                # 더미 이미지로 검출 테스트 (numpy 사용)
                try:
                    import numpy as np
                    dummy_frame = np.zeros((480, 640, 3), dtype=np.uint8)
                    annotated_frame, detections = detector.detect_objects(dummy_frame)
                    test_result("YOLO 객체 검출 테스트", True, f"검출된 객체: {len(detections)}개")
                except Exception as e:
                    test_result("YOLO 객체 검출 테스트", False, str(e))
            else:
                test_result("YOLO 모델 로드", True, "모델 없음 (정상)", warning=True)
                test_skip("YOLO 객체 검출 테스트", "모델이 로드되지 않음")
            
        except ImportError as e:
            test_result("YOLO 검출기", False, f"임포트 실패: {e}")
        except Exception as e:
            test_result("YOLO 검출기", False, f"예상치 못한 오류: {e}")
        
        print()
    
    def test_ui_components(self):
        """UI 컴포넌트 테스트"""
        print("🖼️ UI 컴포넌트 테스트")
        print("-" * 40)
        
        try:
            import tkinter as tk
            from ui_components import RobotArmVisualizer, CameraDisplay, LogDisplay, DetectionDisplay
            
            test_result("UI 컴포넌트 임포트", True)
            
            # 임시 루트 윈도우 생성 (표시하지 않음)
            root = tk.Tk()
            root.withdraw()
            
            try:
                # 테스트 프레임 생성
                test_frame = tk.Frame(root)
                
                # 각 UI 컴포넌트 테스트
                components = [
                    ("LogDisplay", lambda: LogDisplay(test_frame)),
                    ("DetectionDisplay", lambda: DetectionDisplay(test_frame)),
                    ("CameraDisplay", lambda: CameraDisplay(test_frame))
                ]
                
                for name, creator in components:
                    try:
                        component = creator()
                        test_result(f"UI 컴포넌트 {name}", True)
                    except Exception as e:
                        test_result(f"UI 컴포넌트 {name}", False, str(e))
                
                # 3D 시각화 테스트 (matplotlib 필요)
                try:
                    visualizer = RobotArmVisualizer(test_frame)
                    test_result("3D 로봇팔 시각화", True)
                except Exception as e:
                    test_result("3D 로봇팔 시각화", False, str(e))
                
            finally:
                root.destroy()
            
        except ImportError as e:
            test_result("UI 컴포넌트", False, f"임포트 실패: {e}")
        except Exception as e:
            test_result("UI 컴포넌트", False, f"예상치 못한 오류: {e}")
        
        print()
    
    def test_logging_system(self):
        """로깅 시스템 테스트"""
        print("📝 로깅 시스템 테스트")
        print("-" * 40)
        
        try:
            from logger_setup import setup_logging, OrderLogger
            
            # 시스템 로거 테스트
            logger = setup_logging()
            test_result("시스템 로거 설정", True)
            
            # 테스트 로그 메시지
            logger.info("통합 테스트 로그 메시지")
            logger.warning("통합 테스트 경고 메시지")
            test_result("로그 메시지 출력", True)
            
            # 주문 로거 테스트
            order_logger = OrderLogger()
            order_logger.log_order("테스트", "진행 중", "통합 테스트")
            test_result("주문 로거 테스트", True)
            
            # 로그 파일 생성 확인
            if Path("logs/robot_system.log").exists():
                test_result("로그 파일 생성", True)
            else:
                test_result("로그 파일 생성", False, "로그 파일이 생성되지 않음")
            
        except Exception as e:
            test_result("로깅 시스템", False, str(e))
        
        print()
    
    def test_configuration(self):
        """설정 시스템 테스트"""
        print("⚙️ 설정 시스템 테스트")
        print("-" * 40)
        
        try:
            from config import (
                RobotConfig, WorkspaceLimit, RobotStatus, 
                FURNITURE_INFO, UI_COLORS, DEPENDENCIES, check_dependencies
            )
            
            test_result("설정 모듈 임포트", True)
            
            # 설정 클래스 테스트
            robot_config = RobotConfig()
            workspace = WorkspaceLimit()
            test_result("설정 클래스 인스턴스", True)
            
            # 가구 정보 검증
            if FURNITURE_INFO and len(FURNITURE_INFO) > 0:
                test_result("가구 정보 로드", True, f"{len(FURNITURE_INFO)}개 가구")
            else:
                test_result("가구 정보 로드", False, "가구 정보가 비어있음")
            
            # 의존성 확인
            check_dependencies()
            test_result("의존성 확인", True, f"확인된 패키지: {sum(DEPENDENCIES.values())}개")
            
        except Exception as e:
            test_result("설정 시스템", False, str(e))
        
        print()
    
    def test_integration_workflow(self):
        """통합 워크플로우 테스트"""
        print("🔄 통합 워크플로우 테스트")
        print("-" * 40)
        
        try:
            # 전체 시스템 시뮬레이션
            from robot_controller import RobotController
            from config import RobotConfig, FURNITURE_INFO
            
            controller = RobotController(RobotConfig())
            controller.connect()
            
            # 가구 픽업 시뮬레이션
            furniture_name = "의자"
            if furniture_name in FURNITURE_INFO:
                position = FURNITURE_INFO[furniture_name]['position']
                
                # 1. 안전 위치로 이동
                safe_position = [position[0], position[1], position[2] + 50, position[3]]
                step1 = controller.move_to_position(safe_position)
                
                # 2. 목표 위치로 이동
                step2 = controller.move_to_position(position)
                
                # 3. 그리퍼 조작
                step3 = controller.control_gripper(True)
                
                # 4. 최종 위치로 이동
                final_position = [350, 0, position[2], position[3]]
                step4 = controller.move_to_position(final_position)
                
                if all([step1, step2, step3, step4]):
                    test_result("가구 픽업 워크플로우", True, f"{furniture_name} 픽업 시뮬레이션 성공")
                else:
                    test_result("가구 픽업 워크플로우", False, "일부 단계 실패")
            else:
                test_skip("가구 픽업 워크플로우", "가구 정보 없음")
            
            controller.disconnect()
            
        except Exception as e:
            test_result("통합 워크플로우", False, str(e))
        
        print()
    
    def test_performance(self):
        """성능 테스트"""
        print("⚡ 성능 테스트")
        print("-" * 40)
        
        try:
            # 모듈 임포트 시간 측정
            start_time = time.time()
            from robot_controller import RobotController
            from config import RobotConfig
            import_time = time.time() - start_time
            
            if import_time < 2.0:
                test_result("모듈 임포트 속도", True, f"{import_time:.2f}초")
            else:
                test_result("모듈 임포트 속도", False, f"{import_time:.2f}초 (느림)", warning=True)
            
            # 로봇 동작 시간 측정
            controller = RobotController(RobotConfig())
            controller.connect()
            
            start_time = time.time()
            controller.move_to_position([100, 100, -100, 0])
            move_time = time.time() - start_time
            
            if move_time < 1.0:
                test_result("로봇 이동 응답 시간", True, f"{move_time:.2f}초")
            else:
                test_result("로봇 이동 응답 시간", False, f"{move_time:.2f}초 (느림)", warning=True)
            
            controller.disconnect()
            
        except Exception as e:
            test_result("성능 테스트", False, str(e))
        
        print()
    
    def print_summary(self):
        """테스트 결과 요약"""
        end_time = time.time()
        duration = end_time - self.start_time
        
        print("📊 테스트 결과 요약")
        print("=" * 70)
        
        total_tests = (len(test_results['passed']) + 
                      len(test_results['failed']) + 
                      len(test_results['warnings']) + 
                      len(test_results['skipped']))
        
        print(f"전체 테스트: {total_tests}개")
        print(f"✅ 성공: {len(test_results['passed'])}개")
        print(f"❌ 실패: {len(test_results['failed'])}개") 
        print(f"⚠️ 경고: {len(test_results['warnings'])}개")
        print(f"⏭️ 건너뜀: {len(test_results['skipped'])}개")
        print(f"⏱️ 소요 시간: {duration:.2f}초")
        
        # 실패한 테스트 상세 정보
        if test_results['failed']:
            print("\n❌ 실패한 테스트:")
            for test_name, message in test_results['failed']:
                print(f"   • {test_name}: {message}")
        
        # 경고 테스트 상세 정보  
        if test_results['warnings']:
            print("\n⚠️ 경고가 있는 테스트:")
            for test_name, message in test_results['warnings']:
                print(f"   • {test_name}: {message}")
        
        # 건너뛴 테스트
        if test_results['skipped']:
            print("\n⏭️ 건너뛴 테스트:")
            for test_name, reason in test_results['skipped']:
                print(f"   • {test_name}: {reason}")
        
        print("\n" + "=" * 70)
        
        # 전체 결과 판정
        if len(test_results['failed']) == 0:
            if len(test_results['warnings']) == 0:
                print("🎉 모든 테스트가 성공했습니다! 시스템이 완벽히 준비되었습니다.")
            else:
                print("✅ 테스트 성공! 일부 경고가 있지만 시스템은 정상 작동합니다.")
        else:
            print("⚠️ 일부 테스트가 실패했습니다. 문제를 해결한 후 다시 시도하세요.")
        
        print("\n🚀 시스템 실행: python main.py")
        print("🔧 문제 해결: python diagnose_dobot.py")

def main():
    """메인 실행 함수"""
    print_header()
    
    tester = IntegrationTest()
    
    try:
        # 테스트 단계별 실행
        tester.test_python_environment()
        tester.test_file_structure()
        tester.test_basic_imports()
        tester.test_external_packages()
        tester.test_dobot_api_integration()
        tester.test_robot_controller()
        tester.test_yolo_detector()
        tester.test_ui_components()
        tester.test_logging_system()
        tester.test_configuration()
        tester.test_integration_workflow()
        tester.test_performance()
        
    except KeyboardInterrupt:
        print("\n\n⚠️ 테스트가 사용자에 의해 중단되었습니다.")
    except Exception as e:
        print(f"\n\n❌ 테스트 실행 중 치명적 오류 발생: {e}")
        traceback.print_exc()
    finally:
        tester.print_summary()

if __name__ == "__main__":
    main()
