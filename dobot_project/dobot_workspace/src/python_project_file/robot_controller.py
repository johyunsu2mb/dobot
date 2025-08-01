"""
robot_controller.py - 향상된 로봇 제어 클래스 (버그 수정)
Enhanced Dobot Robot & YOLO Object Detection System
"""

import threading
import time
import logging
import numpy as np
from typing import List
from config import RobotConfig, WorkspaceLimit, RobotStatus, DEPENDENCIES
from utils import (
    validate_position, RobotConnectionError, RobotMovementError, 
    InvalidPositionError, GripperError, TimeoutError
)

# 전역 변수 선언 (Dobot 로봇 제어용)
current_actual = None
algorithm_queue = None
enableStatus_robot = None
robotErrorState = False
globalLockValue = threading.Lock()

# 향상된 Dobot API 핸들러 사용
try:
    from dobot_api_handler import (
        DOBOT_API_AVAILABLE, DobotApiDashboard, DobotApiMove, 
        DobotApi, MyType, alarmAlarmJsonFile
    )
    print("✅ Dobot API 핸들러 로드 성공")
except ImportError as e:
    print(f"⚠️ Dobot API 핸들러 로드 실패: {e}")
    # 기본 더미 클래스 정의
    DOBOT_API_AVAILABLE = False
    
    class DobotApiDashboard:
        def __init__(self, ip, port): pass
        def EnableRobot(self): return True
        def DisableRobot(self): return True
        def GetErrorID(self): return "0"
        def ClearError(self): return True
        def Continue(self): return True
        def DO(self, idx, status): return True
   
    class DobotApiMove:
        def __init__(self, ip, port): pass
        def MovL(self, x, y, z, r): 
            time.sleep(0.5)  # 시뮬레이션 딜레이
            return True
   
    class DobotApi:
        def __init__(self, ip, port):
            self.socket_dobot = None
   
    class MyType:
        pass
   
    def alarmAlarmJsonFile():
        return [], []

class RobotController:
    """향상된 로봇 제어 클래스 (버그 수정)"""
    
    def __init__(self, config: RobotConfig = RobotConfig()):
        self.config = config
        self.workspace = WorkspaceLimit()
        self.dashboard = None
        self.move = None
        self.feed = None
        self.is_connected = False
        self.status = RobotStatus.IDLE
        self.current_position = [0, 0, 0, 0]
        self.logger = logging.getLogger('robot_system.controller')
        self._stop_monitoring = False
        
    def connect(self) -> bool:
        """로봇 연결 (안전한 로깅)"""
        if not DOBOT_API_AVAILABLE:
            try:
                self.logger.warning("Dobot API not available, running in simulation mode")
            except:
                print("⚠️ Dobot API가 사용 불가능합니다. 시뮬레이션 모드로 실행됩니다.")
            # 시뮬레이션 모드에서도 더미 객체 생성
            self.dashboard = DobotApiDashboard(self.config.ip_address, self.config.dashboard_port)
            self.move = DobotApiMove(self.config.ip_address, self.config.move_port)
            self.feed = DobotApi(self.config.ip_address, self.config.feed_port)
            self.is_connected = False  # 시뮬레이션 표시
            return False
            
        try:
            try:
                self.logger.info(f"Attempting robot connection: {self.config.ip_address}")
            except:
                print(f"🔌 로봇 연결 시도 중: {self.config.ip_address}")
            
            self.dashboard = DobotApiDashboard(self.config.ip_address, self.config.dashboard_port)
            self.move = DobotApiMove(self.config.ip_address, self.config.move_port)
            self.feed = DobotApi(self.config.ip_address, self.config.feed_port)
            
            # 로봇 활성화
            self.dashboard.EnableRobot()
            self.is_connected = True
            self._stop_monitoring = False
            
            try:
                self.logger.info("Robot connection and activation completed")
            except:
                print("✅ 로봇 연결 및 활성화 완료")
            
            # 피드백 및 에러 모니터링 스레드 시작
            self._start_monitoring_threads()
            
            return True
            
        except Exception as e:
            try:
                self.logger.error(f"Robot connection failed: {e}")
            except:
                print(f"❌ 로봇 연결 실패: {e}")
            # 연결 실패 시에도 시뮬레이션 모드 객체 생성
            self.dashboard = DobotApiDashboard(self.config.ip_address, self.config.dashboard_port)
            self.move = DobotApiMove(self.config.ip_address, self.config.move_port)
            self.feed = DobotApi(self.config.ip_address, self.config.feed_port)
            self.is_connected = False
            raise RobotConnectionError(f"Robot connection failed: {e}")
    
    def disconnect(self):
        """로봇 연결 해제 (안전한 로깅)"""
        try:
            self._stop_monitoring = True
            if self.is_connected and self.dashboard:
                self.dashboard.DisableRobot()
                self.is_connected = False
                try:
                    self.logger.info("Robot disconnection completed")
                except:
                    print("Robot disconnection completed")
        except Exception as e:
            try:
                self.logger.error(f"Error during robot disconnection: {e}")
            except:
                print(f"Error during robot disconnection: {e}")
    
    def move_to_position(self, position: List[float], validate_pos: bool = True, retry_count: int = 2) -> bool:
        """
        안전한 위치 이동 (통신 안정성 강화)
        
        Args:
            position: 목표 위치 [x, y, z, r]
            validate_pos: 위치 유효성 검증 여부
            retry_count: 재시도 횟수
            
        Returns:
            bool: 이동 성공 여부
            
        Raises:
            InvalidPositionError: 잘못된 위치
            RobotMovementError: 이동 실패
        """
        if validate_pos and not validate_position(position, self.workspace):
            raise InvalidPositionError(f"Position out of workspace: {position}")
        
        # 재시도 로직 추가
        for attempt in range(retry_count + 1):
            try:
                if self.is_connected and self.move:
                    try:
                        if attempt > 0:
                            self.logger.warning(f"Robot movement retry {attempt}/{retry_count}: {position}")
                        else:
                            self.logger.info(f"Robot movement started: {position}")
                    except:
                        print(f"🤖 로봇 이동 {'재시도' if attempt > 0 else '시작'}: {position}")
                    
                    self.status = RobotStatus.MOVING
                    
                    # 연결 상태 확인
                    if not self._check_connection():
                        self.logger.warning("Connection lost, attempting reconnection...")
                        if not self._reconnect():
                            raise RobotConnectionError("Failed to reconnect")
                    
                    # 수정된 타임아웃 처리 - 재시도마다 시간 증가
                    timeout = self.config.movement_timeout * (1 + attempt * 0.5)
                    success = self._move_with_timeout(position, timeout)
                    
                    if success:
                        self.current_position = position.copy()
                        self.status = RobotStatus.IDLE
                        
                        try:
                            self.logger.info(f"Robot movement completed: {position}")
                        except:
                            print(f"✅ 로봇 이동 완료: {position}")
                        return True
                    else:
                        if attempt < retry_count:
                            self.logger.warning(f"Movement failed, retrying in 1 second...")
                            time.sleep(1)
                            continue
                        else:
                            self.status = RobotStatus.ERROR
                            raise RobotMovementError(f"Movement timeout after {retry_count + 1} attempts: {position}")
                else:
                    # 시뮬레이션 모드 (API 없거나 연결 실패 시)
                    try:
                        self.logger.info(f"Simulation: Moving to position {position}")
                    except:
                        print(f"🎮 시뮬레이션: 위치 이동 {position}")
                    
                    # 시뮬레이션에서도 더미 객체가 있으면 호출
                    if hasattr(self, 'move') and self.move:
                        self.move.MovL(*position)
                    else:
                        time.sleep(0.5)  # 시뮬레이션 딜레이
                        
                    self.current_position = position.copy()
                    return True
                    
            except RobotConnectionError:
                if attempt < retry_count:
                    self.logger.warning("Connection error, attempting reconnection...")
                    time.sleep(2)
                    continue
                else:
                    raise
            except Exception as e:
                if attempt < retry_count:
                    self.logger.warning(f"Movement error (attempt {attempt + 1}): {e}")
                    time.sleep(1)
                    continue
                else:
                    self.status = RobotStatus.ERROR
                    try:
                        self.logger.error(f"Robot movement failed after all retries: {e}")
                    except:
                        print(f"❌ 로봇 이동 실패: {e}")
                    raise RobotMovementError(f"Robot movement failed: {e}")
        
        return False
    
    def _move_with_timeout(self, position: List[float], timeout: float) -> bool:
        """
        타임아웃을 적용한 이동 함수 (연결 안정성 강화)
        """
        result = [False]
        exception = [None]
        
        def move_target():
            try:
                # 연결 상태 재확인
                if not self._check_connection():
                    raise RobotConnectionError("Connection lost during movement")
                
                self.move.MovL(*position)
                if DOBOT_API_AVAILABLE and self.is_connected:
                    self._wait_arrive(position)
                result[0] = True
            except Exception as e:
                exception[0] = e
        
        thread = threading.Thread(target=move_target, daemon=True)
        thread.start()
        thread.join(timeout)
        
        if thread.is_alive():
            # 타임아웃 발생 시 연결 상태 확인
            self.logger.warning(f"Movement timeout after {timeout}s, checking connection...")
            return False
        
        if exception[0]:
            raise exception[0]
        
        return result[0]
    
    def _check_connection(self) -> bool:
        """연결 상태 확인"""
        if not DOBOT_API_AVAILABLE:
            return True  # 시뮬레이션 모드에서는 항상 연결된 것으로 간주
        
        try:
            if self.dashboard and self.is_connected:
                # 간단한 상태 확인 명령
                error_id = self.dashboard.GetErrorID()
                return True
            return False
        except Exception as e:
            self.logger.warning(f"Connection check failed: {e}")
            return False
    
    def _reconnect(self) -> bool:
        """자동 재연결 시도"""
        try:
            self.logger.info("Attempting automatic reconnection...")
            
            # 기존 연결 정리
            self.disconnect()
            time.sleep(1)
            
            # 재연결 시도
            success = self.connect()
            if success:
                self.logger.info("Automatic reconnection successful")
            else:
                self.logger.warning("Automatic reconnection failed")
            
            return success
            
        except Exception as e:
            self.logger.error(f"Reconnection failed: {e}")
            return False
    
    def control_gripper(self, activate: bool, retry_count: int = 2) -> bool:
        """
        그리퍼 제어 (연결 안정성 강화)
        
        Args:
            activate: True for 활성화, False for 비활성화
            retry_count: 재시도 횟수
            
        Returns:
            bool: 제어 성공 여부
        """
        for attempt in range(retry_count + 1):
            try:
                action = "activated" if activate else "deactivated"
                
                if self.is_connected and self.dashboard:
                    # 연결 상태 확인
                    if not self._check_connection():
                        self.logger.warning("Connection lost during gripper control, attempting reconnection...")
                        if not self._reconnect():
                            raise RobotConnectionError("Failed to reconnect for gripper control")
                    
                    self.dashboard.DO(1, 1 if activate else 0)
                    try:
                        if attempt > 0:
                            self.logger.info(f"Gripper {action} (retry {attempt})")
                        else:
                            self.logger.info(f"Gripper {action}")
                    except:
                        print(f"🤏 그리퍼 {action}")
                else:
                    # 시뮬레이션 모드 또는 연결 없음
                    if hasattr(self, 'dashboard') and self.dashboard:
                        self.dashboard.DO(1, 1 if activate else 0)
                    try:
                        self.logger.info(f"Simulation: Gripper {action}")
                    except:
                        print(f"🎮 시뮬레이션: 그리퍼 {action}")
                
                time.sleep(self.config.gripper_delay)
                return True
                
            except RobotConnectionError:
                if attempt < retry_count:
                    self.logger.warning("Gripper control connection error, retrying...")
                    time.sleep(1)
                    continue
                else:
                    raise
            except Exception as e:
                if attempt < retry_count:
                    self.logger.warning(f"Gripper control error (attempt {attempt + 1}): {e}")
                    time.sleep(0.5)
                    continue
                else:
                    try:
                        self.logger.error(f"Gripper control error: {e}")
                    except:
                        print(f"❌ 그리퍼 제어 오류: {e}")
                    raise GripperError(f"Gripper control failed: {e}")
        
        return False
    
    def _wait_arrive(self, target: List[float]):
        """목표 위치 도달 대기 (버그 수정)"""
        global current_actual, globalLockValue
        
        start_time = time.time()
        while time.time() - start_time < self.config.movement_timeout:
            if self._stop_monitoring:
                break
                
            arrived = True
            with globalLockValue:
                # None 체크 추가 (버그 수정)
                if current_actual is not None and len(current_actual) >= 4:
                    for i in range(4):
                        if abs(current_actual[i] - target[i]) > self.config.position_tolerance:
                            arrived = False
                            break
                    if arrived:
                        return
                else:
                    # current_actual이 None이거나 길이가 부족한 경우
                    arrived = False
                    
            time.sleep(0.01)
        
        raise TimeoutError(f"위치 도달 타임아웃: {target}")
    
    def _start_monitoring_threads(self):
        """모니터링 스레드 시작"""
        if self.feed and DOBOT_API_AVAILABLE and self.is_connected:
            feed_thread = threading.Thread(target=self._feed_monitor, daemon=True)
            feed_thread.start()
        
        if self.dashboard and DOBOT_API_AVAILABLE and self.is_connected:
            error_thread = threading.Thread(target=self._error_monitor, daemon=True)
            error_thread.start()
    
    def _feed_monitor(self):
        """피드백 모니터링 (버그 수정)"""
        global current_actual, algorithm_queue, enableStatus_robot, robotErrorState
        
        while self.is_connected and not self._stop_monitoring:
            try:
                if not hasattr(self.feed, 'socket_dobot') or self.feed.socket_dobot is None:
                    break
                    
                data = bytes()
                read_len = 0
                while read_len < 1440 and not self._stop_monitoring:
                    try:
                        chunk = self.feed.socket_dobot.recv(1440 - read_len)
                        if not chunk:
                            break
                        read_len += len(chunk)
                        data += chunk
                    except:
                        break
                
                if len(data) >= 1440:
                    feedInfo = np.frombuffer(data, dtype=MyType)
                    
                    if hex(feedInfo['test_value'][0]) == '0x123456789abcdef':
                        with globalLockValue:
                            current_actual = feedInfo["tool_vector_actual"][0]
                            algorithm_queue = feedInfo['isRunQueuedCmd'][0]
                            enableStatus_robot = feedInfo['EnableStatus'][0]
                            robotErrorState = feedInfo['ErrorStatus'][0]
                        
            except Exception as e:
                try:
                    self.logger.error(f"피드백 모니터링 오류: {e}")
                except:
                    print(f"피드백 모니터링 오류: {e}")
                break
            
            time.sleep(0.001)
    
    def _error_monitor(self):
        """에러 모니터링 (버그 수정)"""
        global robotErrorState, enableStatus_robot, algorithm_queue, globalLockValue
        
        try:
            dataController, dataServo = alarmAlarmJsonFile()
        except:
            dataController, dataServo = [], []
        
        while self.is_connected and not self._stop_monitoring:
            try:
                with globalLockValue:
                    # None 체크 추가 (버그 수정)
                    if robotErrorState:
                        try:
                            error_id = self.dashboard.GetErrorID()
                            self.logger.error(f"로봇 에러 감지: {error_id}")
                            
                            # 자동 에러 클리어 시도
                            self.dashboard.ClearError()
                            time.sleep(0.1)
                            self.dashboard.Continue()
                        except Exception as e:
                            self.logger.error(f"에러 처리 중 오류: {e}")
                        
                    elif (enableStatus_robot is not None and 
                          algorithm_queue is not None and
                          len(enableStatus_robot) > 0 and
                          len(algorithm_queue) > 0 and
                          int(enableStatus_robot[0]) == 1 and 
                          int(algorithm_queue[0]) == 0):
                        try:
                            self.dashboard.Continue()
                        except Exception as e:
                            self.logger.error(f"Continue 명령 중 오류: {e}")
                        
            except Exception as e:
                try:
                    self.logger.error(f"에러 모니터링 중 오류: {e}")
                except:
                    print(f"에러 모니터링 중 오류: {e}")
            
            time.sleep(5)