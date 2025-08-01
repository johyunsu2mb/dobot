#!/usr/bin/env python3
"""
dobot_tcp_server.py 메서드 이름 수정 버전
"""
import socket
import threading
import time
import logging
from typing import Optional, Dict, Any

class DobotSimulator:
    """Dobot 시뮬레이터"""
    def __init__(self):
        self.position = [200.0, 0.0, 150.0, 0.0]
        self.is_connected = False
        
    def connect(self):
        self.is_connected = True
        return True
        
    def disconnect(self):
        self.is_connected = False
        
    def send_command(self, command: str) -> str:
        if command.startswith("MovJ"):
            try:
                # MovJ(x,y,z,r) 파싱
                coords = command[5:-1].split(',')
                self.position = [float(x.strip()) for x in coords[:4]]
                return f"OK: {self.position}"
            except:
                return "ERROR: Invalid command format"
        elif command == "GetPose()":
            return f"OK: {self.position}"
        elif command == "EnableRobot()":
            return "OK: Robot enabled"
        elif command == "DisableRobot()":
            return "OK: Robot disabled"
        elif command == "ClearError()":
            return "OK: Errors cleared"
        elif command == "EmergencyStop()":
            return "OK: Emergency stop"
        elif command.startswith("Suction"):
            return "OK: Suction set"
        elif command.startswith("Gripper"):
            return "OK: Gripper set"
        else:
            return f"OK: {command}"
    
    def get_status(self):
        return {
            'connected': self.is_connected,
            'position': self.position,
            'simulation': True
        }

class DobotTCPServer:
    """Dobot TCP 서버 - GUI와 호환되는 메서드 이름"""
    
    def __init__(self, host: str = '192.168.1.6', port: int = 29999, simulation_mode: bool = True):
        self.host = host
        self.port = port
        self.simulation_mode = simulation_mode
        
        # 연결 상태
        self.is_connected = False
        self.socket = None
        
        # 시뮬레이터
        self.simulator = None
        if simulation_mode:
            self.simulator = DobotSimulator()
        
        # 로깅
        self.logger = logging.getLogger(__name__)
        
        if simulation_mode:
            self.logger.info("🎮 시뮬레이션 모드로 초기화됨")
    
    # GUI가 호출하는 메서드들
    def connect_to_dobot(self) -> bool:
        """GUI용 연결 메서드"""
        return self.connect()
    
    def disconnect_from_dobot(self):
        """GUI용 연결 해제 메서드"""
        self.disconnect()
    
    def connect(self) -> bool:
        """Dobot에 연결"""
        try:
            if self.simulation_mode:
                # 시뮬레이션 모드
                if self.simulator:
                    result = self.simulator.connect()
                    self.is_connected = result
                    if result:
                        self.logger.info("✅ 시뮬레이션 로봇에 연결됨")
                    return result
            else:
                # 실제 로봇 모드
                if self.is_connected:
                    self.logger.warning("이미 연결되어 있습니다")
                    return True
                
                self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.socket.settimeout(5.0)
                
                self.logger.info(f"🔌 {self.host}:{self.port}에 연결 시도 중...")
                self.socket.connect((self.host, self.port))
                
                self.is_connected = True
                self.logger.info("✅ 실제 로봇에 연결됨")
                return True
                
        except socket.timeout:
            self.logger.error("❌ 연결 시간 초과")
            # 시뮬레이션 모드로 전환
            if not self.simulation_mode:
                self.logger.warning("🎮 시뮬레이션 모드로 전환")
                self.simulation_mode = True
                self.simulator = DobotSimulator()
                return self.connect()
            return False
        except socket.error as e:
            self.logger.error(f"❌ 소켓 연결 실패: {e}")
            # 시뮬레이션 모드로 전환
            if not self.simulation_mode:
                self.logger.warning("🎮 시뮬레이션 모드로 전환")
                self.simulation_mode = True
                self.simulator = DobotSimulator()
                return self.connect()
            return False
        except Exception as e:
            self.logger.error(f"❌ 연결 오류: {e}")
            return False
    
    def disconnect(self):
        """연결 해제"""
        self.is_connected = False
        
        if self.simulation_mode and self.simulator:
            self.simulator.disconnect()
            self.logger.info("🔌 시뮬레이션 연결 해제됨")
        elif self.socket:
            try:
                self.socket.close()
                self.logger.info("🔌 실제 로봇 연결 해제됨")
            except:
                pass
            finally:
                self.socket = None
    
    def send_command(self, command: str) -> str:
        """명령 전송"""
        if not self.is_connected:
            return "ERROR: Not connected"
        
        try:
            if self.simulation_mode and self.simulator:
                # 시뮬레이션 모드
                response = self.simulator.send_command(command)
                self.logger.debug(f"명령: {command} -> 응답: {response}")
                return response
            else:
                # 실제 로봇 모드
                if not self.socket:
                    return "ERROR: Socket is None"
                
                self.socket.sendall((command + '\n').encode('utf-8'))
                response = self.socket.recv(1024).decode('utf-8').strip()
                self.logger.debug(f"명령: {command} -> 응답: {response}")
                return response
                
        except socket.error as e:
            error_msg = f"ERROR: Communication error - {e}"
            self.logger.error(error_msg)
            return error_msg
        except Exception as e:
            error_msg = f"ERROR: Command error - {e}"
            self.logger.error(error_msg)
            return error_msg
    
    def get_pose(self) -> Optional[list]:
        """현재 위치 조회"""
        self.socket.settimeout(5.0)
        try:
            response = self.send_command("GetPose()")
            
            if response.startswith("OK:"):
                # "OK: [x, y, z, r]" 형태의 응답 파싱
                pose_str = response[3:].strip()
                
                # 다양한 형식 처리
                if pose_str.startswith('[') and pose_str.endswith(']'):
                    pose_str = pose_str[1:-1]
                
                pose_values = [float(x.strip()) for x in pose_str.split(',')]
                return pose_values[:4]  # x, y, z, r만 반환
            else:
                self.logger.error(f"위치 조회 실패: {response}")
                return None
                
        except Exception as e:
            self.logger.error(f"위치 조회 오류: {e}")
            return None
    
    def home(self) -> str:
        """홈 위치로 이동"""
        return self.send_command("MovJ(200,0,150,0)")
    
    def move_to(self, x: float, y: float, z: float, r: float = 0.0) -> str:
        """지정된 위치로 이동"""
        command = f"MovJ({x},{y},{z},{r})"
        return self.send_command(command)
    
    def set_suction(self, enable: bool) -> str:
        """석션 제어"""
        command = f"Suction({1 if enable else 0})"
        return self.send_command(command)
    
    def set_gripper(self, enable: bool) -> str:
        """그리퍼 제어"""
        command = f"Gripper({1 if enable else 0})"
        return self.send_command(command)
    
    def emergency_stop(self) -> str:
        """비상정지"""
        return self.send_command("EmergencyStop()")
    
    def clear_alarms(self) -> str:
        """알람 해제"""
        return self.send_command("ClearError()")
    
    def enable_robot(self) -> str:
        """로봇 활성화"""
        return self.send_command("EnableRobot()")
    
    def disable_robot(self) -> str:
        """로봇 비활성화"""
        return self.send_command("DisableRobot()")
    
    def get_status(self) -> Dict[str, Any]:
        """시스템 상태 반환"""
        base_status = {
            'connected': self.is_connected,
            'simulation_mode': self.simulation_mode,
            'host': self.host,
            'port': self.port
        }
        
        if self.is_connected:
            current_pose = self.get_pose()
            base_status['current_pose'] = current_pose
        
        if self.simulation_mode and self.simulator:
            # 시뮬레이션 상세 상태 추가
            sim_status = self.simulator.get_status()
            base_status.update(sim_status)
        
        return base_status

# 테스트 코드
if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    
    # 시뮬레이션 모드로 테스트
    server = DobotTCPServer(simulation_mode=True)
    
    print("🧪 Dobot TCP 서버 테스트")
    
    # 연결 테스트
    if server.connect_to_dobot():
        print("✅ 연결 성공")
        
        # 위치 조회
        pose = server.get_pose()
        print(f"현재 위치: {pose}")
        
        # 이동 명령
        result = server.move_to(250, 100, 200, 45)
        print(f"이동 결과: {result}")
        
        # 위치 확인
        pose = server.get_pose()
        print(f"이동 후 위치: {pose}")
        
        # 상태 확인
        status = server.get_status()
        print(f"상태: {status}")
        
        # 연결 해제
        server.disconnect_from_dobot()
        print("✅ 연결 해제")
    else:
        print("❌ 연결 실패")