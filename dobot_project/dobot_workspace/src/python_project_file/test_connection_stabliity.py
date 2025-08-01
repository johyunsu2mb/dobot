#!/usr/bin/env python3
"""
test_connection_stability.py - 연결 안정성 테스트
Enhanced Dobot Robot & YOLO Object Detection System

로봇 연결의 안정성을 테스트하고 에러 복구 기능을 검증합니다.
"""

import time
import threading
from datetime import datetime
from robot_controller import RobotController
from config import RobotConfig

class ConnectionStabilityTest:
    """연결 안정성 테스트 클래스"""
    
    def __init__(self):
        self.robot_config = RobotConfig()
        self.robot_controller = RobotController(self.robot_config)
        self.test_results = {
            'total_tests': 0,
            'successful_moves': 0,
            'failed_moves': 0,
            'connection_recoveries': 0,
            'timeouts': 0
        }
    
    def run_stability_test(self, test_duration_minutes: int = 5):
        """연결 안정성 테스트 실행"""
        print("🧪 로봇 연결 안정성 테스트 시작")
        print("=" * 50)
        print(f"테스트 시간: {test_duration_minutes}분")
        print(f"시작 시간: {datetime.now().strftime('%H:%M:%S')}")
        print()
        
        # 로봇 연결
        try:
            connected = self.robot_controller.connect()
            if connected:
                print("✅ 실제 로봇 연결됨")
            else:
                print("⚠️ 시뮬레이션 모드로 테스트 진행")
        except Exception as e:
            print(f"❌ 연결 실패: {e}")
            print("⚠️ 시뮬레이션 모드로 테스트 진행")
        
        print()
        
        # 테스트 포지션들
        test_positions = [
            [100, 100, -100, 0],
            [150, 150, -120, 45],
            [200, 100, -110, -30],
            [100, 200, -130, 90],
            [0, 0, 0, 0]  # 홈 위치
        ]
        
        start_time = time.time()
        test_end_time = start_time + (test_duration_minutes * 60)
        position_index = 0
        
        print("🏃‍♂️ 테스트 실행 중...")
        print("Press Ctrl+C to stop")
        print()
        
        try:
            while time.time() < test_end_time:
                position = test_positions[position_index % len(test_positions)]
                position_index += 1
                
                self.test_results['total_tests'] += 1
                test_start = time.time()
                
                try:
                    # 연결 상태 확인
                    if not self.robot_controller._check_connection():
                        print(f"⚠️ 연결 끊어짐 감지, 재연결 시도 중...")
                        if self.robot_controller._reconnect():
                            print(f"✅ 재연결 성공")
                            self.test_results['connection_recoveries'] += 1
                        else:
                            print(f"❌ 재연결 실패")
                    
                    # 이동 테스트
                    success = self.robot_controller.move_to_position(position, retry_count=2)
                    test_duration = time.time() - test_start
                    
                    if success:
                        self.test_results['successful_moves'] += 1
                        status = "✅"
                    else:
                        self.test_results['failed_moves'] += 1
                        status = "❌"
                    
                    # 결과 출력
                    elapsed = time.time() - start_time
                    print(f"{status} Test {self.test_results['total_tests']:3d}: "
                          f"Position {position} - {test_duration:.2f}s "
                          f"(Elapsed: {elapsed:.1f}s)")
                    
                    # 잠시 대기
                    time.sleep(0.5)
                    
                except Exception as e:
                    self.test_results['failed_moves'] += 1
                    if "timeout" in str(e).lower():
                        self.test_results['timeouts'] += 1
                    
                    print(f"❌ Test {self.test_results['total_tests']:3d}: ERROR - {e}")
                    
                    # 에러 복구 시도
                    try:
                        self.robot_controller._reconnect()
                        time.sleep(1)
                    except:
                        pass
        
        except KeyboardInterrupt:
            print("\n⚠️ 테스트가 사용자에 의해 중단되었습니다.")
        
        # 결과 출력
        self.print_test_results(time.time() - start_time)
        
        # 정리
        self.robot_controller.disconnect()
    
    def print_test_results(self, actual_duration: float):
        """테스트 결과 출력"""
        print("\n" + "=" * 50)
        print("📊 연결 안정성 테스트 결과")
        print("=" * 50)
        
        results = self.test_results
        total = results['total_tests']
        
        if total > 0:
            success_rate = (results['successful_moves'] / total) * 100
            failure_rate = (results['failed_moves'] / total) * 100
        else:
            success_rate = failure_rate = 0
        
        print(f"🕒 실제 테스트 시간: {actual_duration:.1f}초")
        print(f"📈 총 테스트 수행: {total}회")
        print(f"✅ 성공한 이동: {results['successful_moves']}회 ({success_rate:.1f}%)")
        print(f"❌ 실패한 이동: {results['failed_moves']}회 ({failure_rate:.1f}%)")
        print(f"🔄 연결 복구: {results['connection_recoveries']}회")
        print(f"⏰ 타임아웃: {results['timeouts']}회")
        
        # 성능 평가
        print("\n📋 성능 평가:")
        if success_rate >= 95:
            print("🎉 우수: 연결이 매우 안정적입니다!")
        elif success_rate >= 80:
            print("✅ 양호: 연결이 대체로 안정적입니다.")
        elif success_rate >= 60:
            print("⚠️ 보통: 일부 연결 문제가 있을 수 있습니다.")
        else:
            print("❌ 불량: 연결에 심각한 문제가 있습니다.")
        
        # 권장사항
        print("\n💡 권장사항:")
        if results['timeouts'] > total * 0.2:  # 20% 이상 타임아웃
            print("- 타임아웃이 많이 발생합니다. 네트워크 상태를 확인하세요.")
            print("- config.py에서 movement_timeout 값을 늘려보세요.")
        
        if results['connection_recoveries'] > 0:
            print("- 연결 끊김이 발생했지만 자동 복구가 작동했습니다.")
            print("- USB 케이블과 전원 연결을 확인하세요.")
        
        if success_rate < 80:
            print("- 로봇 드라이버와 펌웨어를 업데이트하세요.")
            print("- 다른 프로그램이 로봇을 사용하고 있지 않은지 확인하세요.")
        
        print("\n🔧 추가 진단: python diagnose_dobot.py")

def run_quick_test():
    """빠른 연결 테스트"""
    print("⚡ 빠른 연결 테스트 (30초)")
    tester = ConnectionStabilityTest()
    tester.run_stability_test(0.5)  # 30초

def run_full_test():
    """전체 연결 테스트"""
    print("🔋 전체 연결 안정성 테스트 (5분)")
    tester = ConnectionStabilityTest()
    tester.run_stability_test(5)  # 5분

def run_stress_test():
    """스트레스 테스트"""
    print("💪 스트레스 테스트 (10분)")
    tester = ConnectionStabilityTest()
    tester.run_stability_test(10)  # 10분

def main():
    """메인 실행 함수"""
    print("🤖 Enhanced Dobot Robot 연결 안정성 테스트")
    print("=" * 60)
    print()
    print("테스트 옵션을 선택하세요:")
    print("1. 빠른 테스트 (30초)")
    print("2. 표준 테스트 (5분)")  
    print("3. 스트레스 테스트 (10분)")
    print("4. 사용자 정의")
    print()
    
    try:
        choice = input("선택 (1-4): ").strip()
        
        if choice == "1":
            run_quick_test()
        elif choice == "2":
            run_full_test()
        elif choice == "3":
            run_stress_test()
        elif choice == "4":
            try:
                minutes = float(input("테스트 시간 (분): "))
                if minutes > 0:
                    tester = ConnectionStabilityTest()
                    tester.run_stability_test(minutes)
                else:
                    print("❌ 잘못된 시간입니다.")
            except ValueError:
                print("❌ 숫자를 입력해주세요.")
        else:
            print("❌ 잘못된 선택입니다.")
            
    except KeyboardInterrupt:
        print("\n프로그램이 종료되었습니다.")
    except Exception as e:
        print(f"❌ 오류 발생: {e}")

if __name__ == "__main__":
    main()
