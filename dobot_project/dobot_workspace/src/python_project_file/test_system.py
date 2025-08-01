"""
test_system.py - 시스템 테스트
Enhanced Dobot Robot & YOLO Object Detection System
"""

import unittest
import sys
import os
import time
import threading
from unittest.mock import Mock, patch

# 로컬 모듈 임포트를 위한 경로 추가
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

class TestSystemComponents(unittest.TestCase):
    """시스템 컴포넌트 테스트"""
    
    def setUp(self):
        """테스트 환경 설정"""
        self.test_position = [100, 100, -100, 0]
        self.invalid_position = [1000, 1000, 1000, 1000]  # 작업 공간 밖
    
    def test_config_import(self):
        """설정 파일 임포트 테스트"""
        try:
            import config
            self.assertTrue(hasattr(config, 'RobotConfig'))
            self.assertTrue(hasattr(config, 'FURNITURE_INFO'))
            self.assertTrue(hasattr(config, 'DEPENDENCIES'))
            print("✅ config.py 임포트 성공")
        except ImportError as e:
            self.fail(f"config.py 임포트 실패: {e}")
    
    def test_logger_setup(self):
        """로거 설정 테스트"""
        try:
            from logger_setup import setup_logging, OrderLogger
            logger = setup_logging()
            self.assertIsNotNone(logger)
            
            order_logger = OrderLogger()
            order_logger.log_order("테스트", "시작", "단위 테스트")
            print("✅ logger_setup.py 동작 확인")
        except Exception as e:
            self.fail(f"로거 설정 실패: {e}")
    
    def test_utils_functions(self):
        """유틸리티 함수 테스트"""
        try:
            from utils import validate_position, safe_float_conversion
            
            # 유효한 위치 테스트
            self.assertTrue(validate_position(self.test_position))
            
            # 무효한 위치 테스트
            self.assertFalse(validate_position(self.invalid_position))
            
            # 안전한 float 변환 테스트
            self.assertEqual(safe_float_conversion("123.45"), 123.45)
            self.assertEqual(safe_float_conversion("invalid", 0.0), 0.0)
            
            print("✅ utils.py 함수들 동작 확인")
        except Exception as e:
            self.fail(f"유틸리티 함수 테스트 실패: {e}")
    
    def test_robot_controller_simulation(self):
        """로봇 컨트롤러 시뮬레이션 모드 테스트"""
        try:
            from robot_controller import RobotController
            from config import RobotConfig
            
            # 시뮬레이션 모드 컨트롤러 생성
            config = RobotConfig()
            controller = RobotController(config)
            
            # 연결 시도 (시뮬레이션 모드)
            controller.connect()
            
            # 위치 이동 테스트 (시뮬레이션)
            success = controller.move_to_position(self.test_position)
            self.assertTrue(success)
            
            # 그리퍼 제어 테스트 (시뮬레이션)
            success = controller.control_gripper(True)
            self.assertTrue(success)
            
            controller.disconnect()
            print("✅ robot_controller.py 시뮬레이션 모드 확인")
        except Exception as e:
            self.fail(f"로봇 컨트롤러 테스트 실패: {e}")
    
    def test_yolo_detector_initialization(self):
        """YOLO 감지기 초기화 테스트"""
        try:
            from yolo_detector import YOLODetector
            
            detector = YOLODetector()
            self.assertIsNotNone(detector)
            
            # YOLO가 사용 가능한 경우 모델 로드 확인
            from config import DEPENDENCIES
            if DEPENDENCIES['YOLO_AVAILABLE']:
                self.assertTrue(detector.model_loaded)
                print("✅ yolo_detector.py YOLO 모델 로드 확인")
            else:
                self.assertFalse(detector.model_loaded)
                print("ℹ️ yolo_detector.py YOLO 모델 없음 (정상)")
                
        except Exception as e:
            self.fail(f"YOLO 감지기 테스트 실패: {e}")
    
    def test_ui_components_creation(self):
        """UI 컴포넌트 생성 테스트"""
        try:
            import tkinter as tk
            from ui_components import LogDisplay, DetectionDisplay
            
            # 임시 루트 윈도우 생성
            root = tk.Tk()
            root.withdraw()  # 윈도우 숨기기
            
            # 테스트 프레임 생성
            test_frame = tk.Frame(root)
            
            # 로그 디스플레이 테스트
            log_display = LogDisplay(test_frame)
            self.assertIsNotNone(log_display.text_widget)
            
            # 감지 결과 디스플레이 테스트
            detection_display = DetectionDisplay(test_frame)
            self.assertIsNotNone(detection_display.text_widget)
            
            root.destroy()
            print("✅ ui_components.py 컴포넌트 생성 확인")
        except Exception as e:
            self.fail(f"UI 컴포넌트 테스트 실패: {e}")

class TestSystemIntegration(unittest.TestCase):
    """시스템 통합 테스트"""
    
    def test_dependency_check(self):
        """의존성 확인 테스트"""
        try:
            from config import DEPENDENCIES, check_dependencies
            
            # 의존성 다시 확인
            check_dependencies()
            
            # 기본 의존성 키 확인
            required_keys = ['DOBOT_AVAILABLE', 'YOLO_AVAILABLE', 'CV2_AVAILABLE', 'PIL_AVAILABLE']
            for key in required_keys:
                self.assertIn(key, DEPENDENCIES)
            
            print("✅ 의존성 확인 완료")
            for key, value in DEPENDENCIES.items():
                status = "사용가능" if value else "사용불가"
                print(f"  {key}: {status}")
                
        except Exception as e:
            self.fail(f"의존성 확인 실패: {e}")
    
    def test_full_system_simulation(self):
        """전체 시스템 시뮬레이션 테스트"""
        try:
            from robot_controller import RobotController
            from config import RobotConfig, FURNITURE_INFO
            
            # 시뮬레이션 컨트롤러 생성
            controller = RobotController(RobotConfig())
            controller.connect()
            
            # 가구 픽업 시뮬레이션
            furniture_name = "의자"
            if furniture_name in FURNITURE_INFO:
                position = FURNITURE_INFO[furniture_name]['position']
                
                # 안전 위치로 이동
                safe_position = [position[0], position[1], position[2] + 50, position[3]]
                success1 = controller.move_to_position(safe_position)
                
                # 목표 위치로 이동
                success2 = controller.move_to_position(position)
                
                # 그리퍼 조작
                success3 = controller.control_gripper(True)
                
                # 최종 위치로 이동
                final_position = [350, 0, position[2], position[3]]
                success4 = controller.move_to_position(final_position)
                
                self.assertTrue(all([success1, success2, success3, success4]))
                print(f"✅ {furniture_name} 픽업 시뮬레이션 완료")
            
            controller.disconnect()
            
        except Exception as e:
            self.fail(f"전체 시스템 시뮬레이션 실패: {e}")

def run_performance_test():
    """성능 테스트"""
    print("\n🚀 성능 테스트 실행 중...")
    
    try:
        from robot_controller import RobotController
        from config import RobotConfig
        
        controller = RobotController(RobotConfig())
        controller.connect()
        
        # 이동 성능 테스트
        test_positions = [
            [100, 100, -100, 0],
            [200, 150, -120, 45],
            [150, 200, -110, -30],
            [0, 0, 0, 0]
        ]
        
        start_time = time.time()
        for position in test_positions:
            controller.move_to_position(position)
        end_time = time.time()
        
        total_time = end_time - start_time
        avg_time = total_time / len(test_positions)
        
        print(f"  평균 이동 시간: {avg_time:.2f}초")
        print(f"  총 테스트 시간: {total_time:.2f}초")
        
        controller.disconnect()
        
    except Exception as e:
        print(f"  ❌ 성능 테스트 실패: {e}")

def run_stress_test():
    """스트레스 테스트"""
    print("\n💪 스트레스 테스트 실행 중...")
    
    def worker_thread(thread_id, num_operations):
        try:
            from utils import validate_position, safe_float_conversion
            
            for i in range(num_operations):
                # 유효성 검증 반복
                validate_position([i % 400, i % 400, -100, 0])
                safe_float_conversion(str(i * 0.1))
                
                if i % 10 == 0:
                    time.sleep(0.001)  # 간단한 딜레이
                    
            print(f"  스레드 {thread_id}: {num_operations}개 작업 완료")
            
        except Exception as e:
            print(f"  스레드 {thread_id} 오류: {e}")
    
    # 멀티스레드 테스트
    threads = []
    num_threads = 5
    operations_per_thread = 100
    
    start_time = time.time()
    
    for i in range(num_threads):
        thread = threading.Thread(
            target=worker_thread, 
            args=(i, operations_per_thread)
        )
        threads.append(thread)
        thread.start()
    
    for thread in threads:
        thread.join()
    
    end_time = time.time()
    total_operations = num_threads * operations_per_thread
    
    print(f"  총 {total_operations}개 작업을 {end_time - start_time:.2f}초에 완료")

def main():
    """메인 테스트 실행"""
    print("🧪 Enhanced Dobot Robot System 테스트 시작")
    print("=" * 60)
    
    # 단위 테스트 실행
    print("\n📋 단위 테스트 실행 중...")
    
    # 테스트 스위트 생성
    loader = unittest.TestLoader()
    suite = unittest.TestSuite()
    
    # 테스트 클래스 추가
    suite.addTests(loader.loadTestsFromTestCase(TestSystemComponents))
    suite.addTests(loader.loadTestsFromTestCase(TestSystemIntegration))
    
    # 테스트 실행
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    
    # 성능 테스트
    run_performance_test()
    
    # 스트레스 테스트
    run_stress_test()
    
    # 결과 요약
    print("\n" + "=" * 60)
    if result.wasSuccessful():
        print("🎉 모든 테스트가 성공적으로 완료되었습니다!")
        print("시스템이 정상적으로 작동할 준비가 되었습니다.")
    else:
        print("⚠️ 일부 테스트가 실패했습니다.")
        print(f"실패한 테스트: {len(result.failures)}")
        print(f"오류가 발생한 테스트: {len(result.errors)}")
    
    print("\n실행 방법:")
    print("  python main.py")

if __name__ == "__main__":
    main()
