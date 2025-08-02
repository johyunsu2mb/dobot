#!/usr/bin/env python3
"""
개선된 통합 Dobot 제어 시스템 - 서버-클라이언트 통신 문제 해결
"""
import sys
import os
import cv2
import numpy as np
import time
import threading
import socket
import queue
import re
import json
import platform
import logging
import traceback
from typing import Dict, Any, Optional, Tuple

# 필요한 모듈들 import
try:
    from dobot_api import (
        DobotApiDashboard, DobotApi, DobotApiMove, MyType, alarmAlarmJsonFile
    )
    DOBOT_API_AVAILABLE = True
except ImportError:
    print("⚠️  Dobot API가 없습니다. 시뮬레이션 모드로 실행됩니다.")
    DOBOT_API_AVAILABLE = False

try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    print("⚠️  YOLO가 없습니다. 객체 감지 기능이 비활성화됩니다.")
    YOLO_AVAILABLE = False

try:
    import flask
    FLASK_AVAILABLE = True
except ImportError:
    print("⚠️  Flask가 없습니다. 기본 HTTP 서버를 사용합니다.")
    FLASK_AVAILABLE = False

# 로깅 설정 강화
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(name)s: %(message)s',
    handlers=[
        logging.StreamHandler(),
        logging.FileHandler(f'dobot_system_{time.strftime("%Y%m%d")}.log')
    ]
)
logger = logging.getLogger(__name__)

# --- 설정 상수 ---
HOST = '0.0.0.0'  # 모든 인터페이스에서 수신하도록 변경
PORT = 8080
CLIENT_PORT = 9999
DESTINATIONS = ['카페', '레스토랑', '건담베이스', '실내골프장']
ROBOT_NAMES = ['pinky_1', 'pinky_2']

# --- 전역 변수 ---
command_queue = queue.Queue()
clients = {}
pending_furn = {}

# Dobot 상태 모니터링 변수
current_actual = None
algorithm_queue = None
enableStatus_robot = None
robotErrorState = False
state_lock = threading.Lock()

# 시뮬레이션 모드
SIMULATION_MODE = not DOBOT_API_AVAILABLE


class SimulatedDobot:
    """Dobot API가 없을 때 사용하는 시뮬레이션 클래스"""
    
    def __init__(self):
        self.position = [294.0, 2.0, 0.0, 0.0]
        self.vacuum_state = False
        self.blow_state = False
        
    def EnableRobot(self):
        logger.info("🎮 [시뮬레이션] 로봇 활성화")
        return "EnableRobot() 시뮬레이션"
        
    def MovL(self, x, y, z, r):
        logger.info(f"🎮 [시뮬레이션] 이동: ({x}, {y}, {z}, {r})")
        self.position = [x, y, z, r]
        time.sleep(0.5)
        
    def DO(self, port, value):
        if port == 1:
            self.vacuum_state = bool(value)
            logger.info(f"🎮 [시뮬레이션] 진공 그리퍼: {'ON' if value else 'OFF'}")
        elif port == 2:
            self.blow_state = bool(value)
            logger.info(f"🎮 [시뮬레이션] 블로우 오프: {'ON' if value else 'OFF'}")
            
    def GetErrorID(self):
        return "[0]"
        
    def ClearError(self):
        logger.info("🎮 [시뮬레이션] 에러 클리어")
        
    def Continue(self):
        logger.info("🎮 [시뮬레이션] 계속")


class ImprovedClientManager:
    """개선된 클라이언트 연결 관리 클래스"""
    
    def __init__(self, host='0.0.0.0', port=9999):
        self.host = host
        self.port = port
        self.clients = {}
        self.lock = threading.Lock()
        self.server_socket = None
        self.running = False
        self.client_counter = 0
        
        # 통신 설정
        self.socket_timeout = 30.0
        self.keepalive_interval = 10.0
        self.max_message_size = 4096
        
    def start_server(self):
        """TCP 서버 시작 - 개선된 버전"""
        try:
            # 기존 소켓이 있다면 정리
            if self.server_socket:
                try:
                    self.server_socket.close()
                except:
                    pass
            
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            
            # 소켓 옵션 설정
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            if hasattr(socket, 'SO_REUSEPORT'):
                self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
            
            # Keep-alive 설정 (Linux/Windows)
            if hasattr(socket, 'SO_KEEPALIVE'):
                self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
            
            # 바인드 및 리슨
            self.server_socket.bind((self.host, self.port))
            self.server_socket.listen(10)
            self.running = True
            
            logger.info(f"📡 클라이언트 서버 시작: {self.host}:{self.port}")
            
            # 서버 상태 확인
            self._verify_server_binding()
            
            # 클라이언트 연결 대기 스레드 시작
            threading.Thread(target=self.accept_clients, daemon=True).start()
            
            # Keep-alive 스레드 시작
            threading.Thread(target=self.keepalive_monitor, daemon=True).start()
            
            return True
            
        except Exception as e:
            logger.error(f"❌ 클라이언트 서버 시작 실패: {e}")
            logger.error(f"🔍 포트 {self.port} 사용 중인지 확인하세요")
            self._show_port_usage_info()
            self.running = False
            return False
    
    def _verify_server_binding(self):
        """서버 바인딩 상태 확인"""
        try:
            # 로컬 주소 정보 출력
            local_addr = self.server_socket.getsockname()
            logger.info(f"✅ 서버 바인딩 성공: {local_addr}")
            
            # 네트워크 인터페이스 정보
            import subprocess
            if platform.system() == "Linux":
                result = subprocess.run(['ip', 'addr'], capture_output=True, text=True)
                logger.debug(f"네트워크 인터페이스:\n{result.stdout}")
            elif platform.system() == "Windows":
                result = subprocess.run(['ipconfig'], capture_output=True, text=True)
                logger.debug(f"네트워크 구성:\n{result.stdout}")
                
        except Exception as e:
            logger.warning(f"서버 상태 확인 중 오류: {e}")
    
    def _show_port_usage_info(self):
        """포트 사용 정보 표시"""
        try:
            import subprocess
            if platform.system() == "Linux":
                cmd = ['lsof', '-i', f':{self.port}']
            elif platform.system() == "Windows":
                cmd = ['netstat', '-an']
            else:
                return
                
            result = subprocess.run(cmd, capture_output=True, text=True)
            if result.returncode == 0:
                logger.info(f"포트 {self.port} 사용 정보:\n{result.stdout}")
        except Exception as e:
            logger.debug(f"포트 사용 정보 확인 실패: {e}")
    
    def accept_clients(self):
        """클라이언트 연결 수락 - 개선된 버전"""
        logger.info("🔄 클라이언트 연결 수락 스레드 시작")
        
        while self.running:
            try:
                # 짧은 타임아웃으로 주기적 확인
                self.server_socket.settimeout(1.0)
                
                try:
                    conn, addr = self.server_socket.accept()
                    logger.info(f"🔌 새 연결 수락: {addr}")
                    
                    # 클라이언트 처리 스레드 시작
                    threading.Thread(
                        target=self.handle_client, 
                        args=(conn, addr), 
                        daemon=True,
                        name=f"Client-{addr[0]}:{addr[1]}"
                    ).start()
                    
                except socket.timeout:
                    continue
                except OSError as e:
                    if self.running:  # 정상적인 종료가 아닌 경우만 로그
                        logger.error(f"소켓 오류: {e}")
                        time.sleep(1)
                    break
                    
            except Exception as e:
                if self.running:
                    logger.error(f"❌ 클라이언트 연결 수락 오류: {e}")
                    time.sleep(1)
                break
        
        logger.info("🔄 클라이언트 연결 수락 스레드 종료")
    
    def handle_client(self, conn, addr):
        """클라이언트 연결 처리 - 강화된 버전"""
        client_id = None
        client_info = None
        
        try:
            # 연결 설정
            conn.settimeout(self.socket_timeout)
            logger.info(f"📞 클라이언트 처리 시작: {addr}")
            
            # 첫 번째 메시지 대기
            first_data = self._receive_message(conn)
            if not first_data:
                logger.warning(f"첫 번째 메시지 없음: {addr}")
                return
                
            first_msg = first_data.strip()
            logger.info(f"📨 첫 번째 메시지: '{first_msg}' from {addr}")
            
            # 클라이언트 타입 결정 및 등록
            client_id, client_info = self._register_client(conn, addr, first_msg)
            
            if not client_id:
                logger.error(f"클라이언트 등록 실패: {addr}")
                return
            
            logger.info(f"✅ 클라이언트 등록 성공: {client_info['name']} ({client_id})")
            
            # 메시지 처리 루프
            self._handle_client_messages(conn, client_id, client_info)
            
        except socket.timeout:
            logger.warning(f"⏰ 클라이언트 타임아웃: {addr}")
        except ConnectionResetError:
            logger.info(f"🔌 클라이언트 연결 재설정: {addr}")
        except Exception as e:
            logger.error(f"❌ 클라이언트 처리 오류 [{client_id or addr}]: {e}")
            logger.error(traceback.format_exc())
        finally:
            self._cleanup_client(client_id, conn)
    
    def _receive_message(self, conn) -> Optional[str]:
        """메시지 수신 - 안전한 버전"""
        try:
            # 버퍼 크기 제한
            data = conn.recv(self.max_message_size)
            if not data:
                return None
            
            # 인코딩 시도 (UTF-8, 실패시 Latin-1)
            try:
                message = data.decode('utf-8')
            except UnicodeDecodeError:
                message = data.decode('latin-1', errors='ignore')
                logger.warning("UTF-8 디코딩 실패, Latin-1로 대체")
            
            return message
            
        except socket.timeout:
            raise
        except Exception as e:
            logger.error(f"메시지 수신 오류: {e}")
            return None
    
    def _register_client(self, conn, addr, first_msg) -> Tuple[Optional[str], Optional[Dict]]:
        """클라이언트 등록"""
        try:
            # 1) ROS2 클라이언트: pinky_1, pinky_2
            if first_msg in ROBOT_NAMES:
                client_id = first_msg
                client_info = {
                    'conn': conn,
                    'name': client_id,
                    'addr': addr,
                    'protocol': 'ros2',
                    'connected_at': time.time()
                }
                
                with self.lock:
                    # 기존 연결이 있다면 정리
                    if client_id in self.clients:
                        logger.warning(f"기존 클라이언트 연결 해제: {client_id}")
                        self._cleanup_client(client_id, None)
                    
                    self.clients[client_id] = client_info
                
                # 등록 확인 전송
                self._send_message(conn, "REGISTERED")
                logger.info(f"🤖 ROS2 클라이언트 등록: {client_id}")
                
                return client_id, client_info
            
            # 2) JSON/일반 클라이언트
            else:
                # 환영 메시지 전송
                self._send_message(conn, "CONNECTED:Please enter your name")
                
                # 이름 등록
                name = first_msg if first_msg else self._receive_message(conn)
                if not name:
                    self._send_message(conn, "ERROR:Name required")
                    return None, None
                
                name = name.strip()
                
                # 고유 ID 생성
                with self.lock:
                    self.client_counter += 1
                    client_id = f"{name}_{self.client_counter}_{int(time.time())}"
                    
                    client_info = {
                        'conn': conn,
                        'name': name,
                        'addr': addr,
                        'protocol': 'json',
                        'connected_at': time.time()
                    }
                    
                    self.clients[client_id] = client_info
                
                # 환영 메시지
                self._send_message(conn, f"WELCOME:{client_id}")
                logger.info(f"👤 JSON 클라이언트 등록: {name} ({client_id})")
                
                return client_id, client_info
                
        except Exception as e:
            logger.error(f"클라이언트 등록 오류: {e}")
            return None, None
    
    def _handle_client_messages(self, conn, client_id, client_info):
        """클라이언트 메시지 처리 루프"""
        protocol = client_info['protocol']
        
        # 무한 타임아웃으로 변경 (Keep-alive로 연결 상태 확인)
        conn.settimeout(None)
        
        while self.running:
            try:
                message = self._receive_message(conn)
                if not message:
                    logger.info(f"클라이언트 연결 종료: {client_info['name']}")
                    break
                
                message = message.strip()
                logger.info(f"📨 메시지 수신 [{client_info['name']}]: '{message}'")
                
                # Keep-alive 응답
                if message.upper() == 'PING':
                    self._send_message(conn, "PONG")
                    continue
                
                # 프로토콜별 처리
                if protocol == 'ros2':
                    # ROS2 프로토콜: 모든 메시지를 command_queue에
                    command_queue.put((client_id, message))
                    self._send_message(conn, "ACK")
                    
                elif protocol == 'json':
                    # JSON 프로토콜 처리
                    try:
                        msg_data = json.loads(message)
                        self._process_json_message(client_id, msg_data)
                    except json.JSONDecodeError:
                        # 일반 텍스트도 허용
                        command_queue.put((client_id, message))
                
            except socket.error as e:
                logger.warning(f"소켓 오류 [{client_info['name']}]: {e}")
                break
            except Exception as e:
                logger.error(f"메시지 처리 오류 [{client_info['name']}]: {e}")
                logger.error(traceback.format_exc())
                break
    
    def _process_json_message(self, client_id, msg_data):
        """JSON 메시지 처리"""
        try:
            msg_type = msg_data.get('type', 'unknown')
            content = msg_data.get('content', '')
            
            if msg_type == 'dobot_command':
                command_queue.put((client_id, content))
            elif msg_type == 'status_request':
                self.send_status_to_client(client_id)
            else:
                logger.warning(f"알 수 없는 JSON 메시지 타입: {msg_type}")
                
        except Exception as e:
            logger.error(f"JSON 메시지 처리 오류: {e}")
    
    def _send_message(self, conn, message):
        """메시지 전송 - 안전한 버전"""
        try:
            if isinstance(message, str):
                message = message.encode('utf-8')
            if not message.endswith(b'\n'):
                message += b'\n'
            
            conn.sendall(message)
            return True
            
        except Exception as e:
            logger.error(f"메시지 전송 오류: {e}")
            return False
    
    def _cleanup_client(self, client_id, conn):
        """클라이언트 정리"""
        if client_id:
            with self.lock:
                client_info = self.clients.pop(client_id, None)
            
            if client_info:
                logger.info(f"🔌 클라이언트 연결 해제: {client_info['name']} ({client_id})")
        
        if conn:
            try:
                conn.close()
            except:
                pass
    
    def keepalive_monitor(self):
        """Keep-alive 모니터링"""
        logger.info("💓 Keep-alive 모니터 시작")
        
        while self.running:
            time.sleep(self.keepalive_interval)
            
            # 모든 클라이언트에게 핑 전송
            with self.lock:
                client_list = list(self.clients.items())
            
            for client_id, client_info in client_list:
                try:
                    conn = client_info['conn']
                    self._send_message(conn, "PING")
                except Exception as e:
                    logger.warning(f"Keep-alive 실패: {client_info['name']} - {e}")
                    # 연결 끊어진 클라이언트 정리
                    self._cleanup_client(client_id, None)
        
        logger.info("💓 Keep-alive 모니터 종료")
    
    def send_to_client(self, target_id, message):
        """특정 클라이언트에게 메시지 전송"""
        with self.lock:
            client_info = self.clients.get(target_id)
        
        if client_info:
            try:
                conn = client_info['conn']
                success = self._send_message(conn, message)
                if success:
                    logger.info(f"📤 메시지 전송 성공 [{client_info['name']}]: {message.strip()}")
                return success
            except Exception as e:
                logger.error(f"❌ 메시지 전송 실패 [{client_info['name']}]: {e}")
                return False
        else:
            logger.error(f"❌ 클라이언트 '{target_id}' 없음")
            return False
    
    def broadcast_message(self, message, exclude_client=None):
        """모든 클라이언트에게 브로드캐스트"""
        with self.lock:
            client_list = list(self.clients.items())
        
        sent_count = 0
        for client_id, client_info in client_list:
            if client_id != exclude_client:
                if self.send_to_client(client_id, message):
                    sent_count += 1
        
        logger.info(f"📡 브로드캐스트: {sent_count}개 클라이언트에게 전송")
        return sent_count
    
    def get_client_count(self) -> int:
        """연결된 클라이언트 수"""
        with self.lock:
            return len(self.clients)
    
    def get_client_list(self) -> list:
        """클라이언트 목록"""
        with self.lock:
            return [
                {
                    'id': cid,
                    'name': info['name'],
                    'protocol': info['protocol'],
                    'addr': info['addr'],
                    'connected_at': info['connected_at']
                }
                for cid, info in self.clients.items()
            ]
    
    def send_status_to_client(self, client_id):
        """클라이언트에게 상태 정보 전송"""
        status = {
            "type": "status_response",
            "connected_clients": self.get_client_count(),
            "client_list": self.get_client_list(),
            "server_running": self.running,
            "timestamp": time.time()
        }
        self.send_to_client(client_id, json.dumps(status))
    
    def stop_server(self):
        """서버 중지"""
        logger.info("🛑 클라이언트 서버 중지 중...")
        self.running = False
        
        # 모든 클라이언트 연결 해제
        with self.lock:
            client_list = list(self.clients.items())
        
        for client_id, client_info in client_list:
            try:
                self._send_message(client_info['conn'], "SERVER_SHUTDOWN")
                client_info['conn'].close()
            except:
                pass
        
        # 서버 소켓 닫기
        if self.server_socket:
            try:
                self.server_socket.close()
            except:
                pass
        
        logger.info("🛑 클라이언트 서버 중지 완료")


# 기존 DobotController, VisionSystem 클래스들은 그대로 유지...
class DobotController:
    """Dobot 제어 클래스"""
    
    def __init__(self, simulation_mode=False):
        self.simulation_mode = simulation_mode
        self.dashboard = None
        self.move = None
        self.feed = None
        
        if simulation_mode:
            self.dashboard = SimulatedDobot()
            self.move = SimulatedDobot()
            self.feed = None
    
    def connect(self, ip='192.168.1.6', dash_port=29999, move_port=30003, feed_port=30004):
        """Dobot에 연결"""
        if self.simulation_mode:
            logger.info("🎮 시뮬레이션 모드로 연결")
            return True
            
        if not DOBOT_API_AVAILABLE:
            logger.error("❌ Dobot API가 없습니다.")
            return False
            
        try:
            self.dashboard = DobotApiDashboard(ip, dash_port)
            self.move = DobotApiMove(ip, move_port)
            self.feed = DobotApi(ip, feed_port)
            
            self.dashboard.EnableRobot()
            logger.info(f"✅ Dobot 연결 성공: {ip}")
            
            if self.feed:
                threading.Thread(target=self.get_feed, daemon=True).start()
                threading.Thread(target=self.clear_robot_error, daemon=True).start()
            
            return True
        except Exception as e:
            logger.error(f"❌ Dobot 연결 실패: {e}")
            return False
    
    def get_feed(self):
        """Dobot 피드백 수신"""
        global current_actual, algorithm_queue, enableStatus_robot, robotErrorState
        
        if not self.feed:
            return
            
        while True:
            try:
                data, received = bytes(), 0
                while received < 1440:
                    chunk = self.feed.socket_dobot.recv(1440 - received)
                    if not chunk: 
                        break
                    data += chunk
                    received += len(chunk)
                    
                if not data: 
                    continue
                    
                info = np.frombuffer(data, dtype=MyType)
                if hex(info['test_value'][0]) == '0x123456789abcdef':
                    with state_lock:
                        current_actual = info['tool_vector_actual'][0]
                        algorithm_queue = info['isRunQueuedCmd'][0]
                        enableStatus_robot = info['EnableStatus'][0]
                        robotErrorState = info['ErrorStatus'][0]
                        
                time.sleep(0.001)
            except Exception as e:
                logger.error(f"피드백 수신 오류: {e}")
                break
    
    def clear_robot_error(self):
        """로봇 에러 처리"""
        if self.simulation_mode or not DOBOT_API_AVAILABLE:
            return
            
        try:
            ctrl_data, servo_data = alarmAlarmJsonFile()
        except:
            ctrl_data, servo_data = [], []
            
        global robotErrorState, algorithm_queue, enableStatus_robot
        
        while True:
            with state_lock:
                err = robotErrorState
                q = algorithm_queue
                en = enableStatus_robot
                
            if err:
                try:
                    codes = [int(e) for e in re.findall(r'-?\d+', self.dashboard.GetErrorID())]
                    if codes and codes[0] == 0:
                        for c in codes[1:]:
                            desc = next(
                                (item['en']['description'] for item in (ctrl_data+servo_data) if item['id']==c),
                                'Unknown'
                            )
                            logger.warning(f"Robot Alarm ID={c}: {desc}")
                        
                        self.dashboard.ClearError()
                        time.sleep(0.01)
                        self.dashboard.Continue()
                        logger.info("자동으로 에러를 클리어했습니다.")
                except Exception as e:
                    logger.error(f"에러 처리 중 오류: {e}")
            else:
                if q and en and int(en[0])==1 and int(q[0])==0:
                    self.dashboard.Continue()
                    
            time.sleep(5)
    
    def wait_arrive(self, target):
        """현재 좌표가 target에 도달할 때까지 대기"""
        if self.simulation_mode:
            time.sleep(1)
            return
            
        while True:
            with state_lock:
                if current_actual is not None and all(
                    abs(current_actual[i] - target[i]) <= 1 for i in range(len(target))
                ):
                    return
            time.sleep(0.001)
    
    def run_point(self, pt):
        """지정된 좌표로 이동"""
        if self.move:
            self.move.MovL(pt[0], pt[1], pt[2], pt[3])
    
    def activate_vacuum_gripper(self, on):
        """진공 그리퍼 제어"""
        if self.dashboard:
            self.dashboard.DO(1, 1 if on else 0)
            logger.info(f"Vacuum gripper {'on' if on else 'off'}")
    
    def blow_off_gripper(self, on):
        """블로우 오프 제어"""
        if self.dashboard:
            self.dashboard.DO(2, 1 if on else 0)
            logger.info(f"Blow-off {'ON' if on else 'OFF'}")


class VisionSystem:
    """YOLO 기반 비전 시스템"""
    
    def __init__(self, model_path='best.pt'):
        self.model = None
        self.cap = None
        self.detector = None
        self.H = None
        
        # 클래스 맵
        self.class_names = {0:"테이블", 1:"쇼파", 2:"의자", 3:"침대"}
        self.class_pick_z = {0:-129.0, 1:-128.0, 2:-115.0, 3:-149.0}
        self.DROP_Z = -10
        
        self.initialize(model_path)
    
    def initialize(self, model_path):
        """비전 시스템 초기화"""
        try:
            # 카메라 초기화
            self.cap = cv2.VideoCapture(0)
            if not self.cap.isOpened():
                logger.warning("⚠️  카메라를 열 수 없습니다. 시뮬레이션 모드로 실행됩니다.")
                self.cap = None
            
            # YOLO 모델 로드
            if YOLO_AVAILABLE and os.path.exists(model_path):
                self.model = YOLO(model_path)
                logger.info(f"✅ YOLO 모델 로드: {model_path}")
            else:
                logger.warning("⚠️  YOLO 모델이 없습니다. 객체 감지가 비활성화됩니다.")
            
            # ARUCO 감지기 초기화
            aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
            self.detector = cv2.aruco.ArucoDetector(aruco_dict, cv2.aruco.DetectorParameters())
            
            # Homography 행렬 설정
            self.setup_homography()
            
        except Exception as e:
            logger.error(f"❌ 비전 시스템 초기화 실패: {e}")
    
    def setup_homography(self):
        """Homography 행렬 설정"""
        pts_aruco = np.array([
            [510.5,358.0],[485.2,365.5],[444.0,372.0],[415.0,174.0],
            [501.5,330.0],[474.5,335.0],[450.2,344.2],[488.8,292.2],
            [467.8,308.0],[448.0,318.2],[476.5,271.5],[459.5,283.5],
            [441.8,295.8],[457.0,247.0],[442.5,263.2],[432.5,278.8],
            [393.8,244.2],[371.0,218.0],[365.5,243.2],[333.0,203.0],
            [329.5,235.5],[296.5,202.5],[297.2,233.2],[265.0,205.2],
            [268.0,236.5],[227.2,209.8],[235.5,242.5],[193.8,231.8],
            [205.5,256.5],[157.8,246.2],[175.5,268.8],[123.2,285.2],
            [152.0,308.2],[107.0,318.2],[134.5,331.5],[166.5,345.5],
            [93.2,358.8],[126.2,369.0]
        ], np.float32)
        
        pts_robot = np.array([
            [101.0,-342.9],[88.75,-299.16],[79.5,-228.6],[335.5,-170.8],
            [148.4,-330.66],[140.1,-281.1],[127.5,-241.1],[203.7,-308.2],
            [184.8,-270.7],[170.9,-239.3],[248.5,-287.7],[227.7,-260.3],
            [208.8,-227.6],[298.7,-261.6],[265.7,-231.7],[235.8,-210.6],
            [299.1,-147.9],[340.4,-110.4],[305.9,-106.1],[368.9,-50.8],
            [315.2,-40.56],[370.17,14.15],[320.4,12.76],[366.4,62.27],
            [312.6,60.36],[357.4,130.25],[304.05,120.1],[332.8,192.25],
            [286.06,165.9],[301.0,244.8],[260.0,218.75],[236.06,303.5],
            [199.4,255.8],[185.26,330.9],[160.0,283.9],[136.3,234.0],
            [122.16,354.1],[95.45,300.1]
        ], np.float32)
        
        try:
            self.H, _ = cv2.findHomography(pts_aruco, pts_robot)
            logger.info("✅ Homography 행렬 설정 완료")
        except Exception as e:
            logger.error(f"❌ Homography 설정 실패: {e}")
            self.H = np.eye(3)
    
    def capture_and_detect(self, furniture_name):
        """프레임 캡처 및 객체 감지"""
        if not self.cap or not self.model:
            logger.info("🎮 [시뮬레이션] 객체 감지")
            return [(300.0, 0.0)]
        
        ret, frame = self.cap.read()
        if not ret:
            logger.error("❌ 카메라 읽기 실패")
            return []
        
        detections = self.model(frame)[0]
        
        cls_id = next((k for k, v in self.class_names.items() if v == furniture_name), None)
        if cls_id is None:
            logger.error(f"❌ 잘못된 가구명: {furniture_name}")
            return []
        
        found = []
        for box in detections.boxes:
            if int(box.cls[0]) != cls_id:
                continue
                
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cx, cy = (x1 + x2)//2, (y1 + y2)//2
            
            uv = np.array([cx, cy, 1.0])
            w = self.H.dot(uv)
            w /= w[2]
            wx, wy = float(w[0]), float(w[1])
            
            if wx <= 380.0:
                found.append((wx, wy))
        
        return found
    
    def detect_aruco_marker(self):
        """ARUCO 마커 감지"""
        if not self.cap or not self.detector:
            logger.info("🎮 [시뮬레이션] ARUCO 마커 감지")
            return (350.0, 0.0)
        
        ret, frame = self.cap.read()
        if not ret:
            logger.error("❌ 카메라 읽기 실패")
            return None
        
        corners, ids, _ = self.detector.detectMarkers(frame)
        if ids is not None and len(ids) > 0:
            center = corners[0][0].mean(axis=0)
            uv = np.array([center[0], center[1], 1.0])
            w = self.H.dot(uv)
            w /= w[2]
            return (float(w[0]), float(w[1]))
        
        return None
    
    def save_annotated_frame(self, filename='arrival_annotated.png'):
        """주석이 달린 프레임 저장"""
        if not self.cap or not self.model:
            logger.info("🎮 [시뮬레이션] 프레임 저장")
            return
        
        ret, frame = self.cap.read()
        if not ret:
            return
        
        detections = self.model(frame)[0]
        for box in detections.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cls = int(box.cls[0])
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, str(cls), (x1, y1-5),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        corners, ids, _ = self.detector.detectMarkers(frame)
        if ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        
        cv2.imwrite(filename, frame)
        logger.info(f"📸 주석 이미지 저장: {filename}")


class ImprovedIntegratedDobotSystem:
    """개선된 통합 Dobot 시스템"""
    
    def __init__(self):
        self.client_manager = ImprovedClientManager(HOST, CLIENT_PORT)
        self.dobot = DobotController(SIMULATION_MODE)
        self.vision = VisionSystem()
        self.running = False
        
        # 시스템 상태
        self.system_status = {
            'started_at': time.time(),
            'commands_processed': 0,
            'errors_count': 0,
            'last_activity': time.time()
        }
        
    def start(self):
        """시스템 시작"""
        logger.info("\n" + "=" * 60)
        logger.info("  🤖 개선된 통합 Dobot 제어 시스템 시작")
        logger.info("=" * 60)
        
        # 네트워크 정보 출력
        self._show_network_info()
        
        # 클라이언트 서버 시작
        if not self.client_manager.start_server():
            logger.error("❌ 클라이언트 서버 시작 실패. 종료합니다.")
            return False
        
        # 서버 준비 확인
        time.sleep(1)
        self._verify_server_status()
        
        # Dobot 연결
        logger.info("\n🤖 Dobot 연결 시도...")
        if not self.dobot.connect():
            logger.warning("⚠️  Dobot 연결 실패. 시뮬레이션 모드로 계속합니다.")
        
        # 기본 위치로 이동
        base = [294.0, 2.0, 0.0, 0.0]
        logger.info(f"📍 기본 위치로 이동: {base}")
        self.dobot.run_point(base)
        self.dobot.wait_arrive(base)
        self.dobot.activate_vacuum_gripper(False)
        
        logger.info("\n✅ 시스템 준비 완료")
        self._show_usage_info()
        
        # 시스템 모니터링 스레드 시작
        threading.Thread(target=self.system_monitor, daemon=True).start()
        
        # 콘솔 명령 처리 스레드 시작
        threading.Thread(target=self.console_command_loop, daemon=True).start()
        
        # 메인 명령 처리 루프 시작
        self.running = True
        try:
            self.command_loop()
        except KeyboardInterrupt:
            logger.info("\n⚠️  사용자에 의해 종료됨")
        except Exception as e:
            logger.error(f"❌ 시스템 오류: {e}")
            logger.error(traceback.format_exc())
        finally:
            self.stop()
        
        return True
    
    def _show_network_info(self):
        """네트워크 정보 표시"""
        try:
            import socket
            hostname = socket.gethostname()
            local_ip = socket.gethostbyname(hostname)
            
            logger.info(f"🌐 네트워크 정보:")
            logger.info(f"   - 호스트명: {hostname}")
            logger.info(f"   - 로컬 IP: {local_ip}")
            logger.info(f"   - 바인드 주소: {HOST}:{CLIENT_PORT}")
            
            # 외부 접근 가능한 주소들
            if HOST == '0.0.0.0':
                logger.info(f"   - 외부 접근 가능: {local_ip}:{CLIENT_PORT}")
                logger.info(f"   - 로컬 접근: 127.0.0.1:{CLIENT_PORT}")
            
        except Exception as e:
            logger.warning(f"네트워크 정보 확인 실패: {e}")
    
    def _verify_server_status(self):
        """서버 상태 확인"""
        logger.info(f"🔍 서버 상태 확인...")
        logger.info(f"   - 실행 상태: {self.client_manager.running}")
        logger.info(f"   - 바인드 포트: {self.client_manager.port}")
        
        # 포트 연결 테스트
        try:
            test_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            test_sock.settimeout(2.0)
            result = test_sock.connect_ex(('127.0.0.1', self.client_manager.port))
            test_sock.close()
            
            if result == 0:
                logger.info(f"   ✅ 포트 {self.client_manager.port} 연결 가능")
            else:
                logger.warning(f"   ❌ 포트 {self.client_manager.port} 연결 불가 (코드: {result})")
        except Exception as e:
            logger.warning(f"   ⚠️  포트 테스트 오류: {e}")
    
    def _show_usage_info(self):
        """사용법 정보 표시"""
        logger.info(f"💡 클라이언트 연결 방법:")
        logger.info(f"   📡 서버 주소: {HOST}:{CLIENT_PORT}")
        logger.info(f"   🤖 ROS2 클라이언트: 'pinky_1' 또는 'pinky_2'로 연결")
        logger.info(f"   👤 일반 클라이언트: 임의 이름으로 연결")
        
        logger.info(f"\n📋 명령어 형식:")
        logger.info(f"   - '<로봇명> <목적지> <가구>': pinky_1 카페 테이블")
        logger.info(f"   - 'ARRIVED': 도착 신호")
        logger.info(f"   - 'status': 시스템 상태")
        logger.info(f"   - 'stop': 시스템 종료")
        
        logger.info(f"\n🔧 콘솔 명령어:")
        logger.info(f"   - list: 접속자 목록")
        logger.info(f"   - send <이름> <메시지>: 메시지 전송")
        logger.info(f"   - status: 시스템 상태")
        logger.info(f"   - test: 연결 테스트")
        logger.info(f"   - stop: 시스템 종료")
    
    def system_monitor(self):
        """시스템 모니터링"""
        logger.info("📊 시스템 모니터 시작")
        
        while self.running:
            try:
                time.sleep(30)  # 30초마다 모니터링
                
                # 상태 업데이트
                uptime = time.time() - self.system_status['started_at']
                client_count = self.client_manager.get_client_count()
                queue_size = command_queue.qsize()
                
                logger.info(f"📊 시스템 상태 - 가동시간: {uptime:.0f}초, "
                          f"클라이언트: {client_count}개, 큐: {queue_size}개")
                
                # 클라이언트 정보
                if client_count > 0:
                    clients = self.client_manager.get_client_list()
                    for client in clients:
                        conn_time = time.time() - client['connected_at']
                        logger.debug(f"   - {client['name']} ({client['protocol']}): {conn_time:.0f}초")
                
            except Exception as e:
                logger.error(f"시스템 모니터링 오류: {e}")
                time.sleep(5)
        
        logger.info("📊 시스템 모니터 종료")
    
    def console_command_loop(self):
        """콘솔 명령 처리"""
        logger.info("\n📋 콘솔 명령어 사용 가능 (help 입력)")
        
        while self.running:
            try:
                sys.stdout.write("시스템> ")
                sys.stdout.flush()
                
                line = sys.stdin.readline().strip()
                if not line:
                    continue
                
                parts = line.split()
                if not parts:
                    continue
                    
                command = parts[0].lower()
                
                if command == 'help':
                    self._show_console_help()
                elif command == 'list':
                    self._show_client_list()
                elif command == 'status':
                    self._show_system_status()
                elif command == 'test':
                    self._run_connection_test()
                elif command == 'stop' or command == 'exit' or command == 'quit':
                    logger.info("시스템을 종료합니다.")
                    self.stop()
                    break
                elif command == 'send' and len(parts) >= 3:
                    target_name = parts[1]
                    message = ' '.join(parts[2:])
                    self._send_message_to_client(target_name, message)
                elif len(parts) == 3 and parts[0] in ROBOT_NAMES and parts[1] in DESTINATIONS:
                    robot_name, dest, furn = parts
                    logger.info(f"📤 직접 명령 추가: {robot_name} {dest} {furn}")
                    command_queue.put((None, line))
                else:
                    logger.warning(f"❌ 알 수 없는 명령어: '{command}' (help 입력)")
                    
            except EOFError:
                break
            except KeyboardInterrupt:
                break
            except Exception as e:
                logger.error(f"❌ 콘솔 명령 처리 오류: {e}")
    
    def _show_console_help(self):
        """콘솔 도움말 표시"""
        logger.info("\n📋 콘솔 명령어:")
        logger.info("  help - 이 도움말")
        logger.info("  list - 접속자 목록")
        logger.info("  status - 시스템 상태")
        logger.info("  test - 연결 테스트")
        logger.info("  send <이름> <메시지> - 메시지 전송")
        logger.info("  <로봇명> <목적지> <가구> - 직접 작업 명령")
        logger.info("  stop/exit/quit - 시스템 종료\n")
    
    def _show_client_list(self):
        """클라이언트 목록 표시"""
        clients = self.client_manager.get_client_list()
        
        logger.info("\n" + "=" * 60)
        logger.info(f"접속자 목록 ({len(clients)}명)")
        logger.info("=" * 60)
        
        if clients:
            for i, client in enumerate(clients, 1):
                conn_time = time.time() - client['connected_at']
                logger.info(f"{i}. {client['name']} ({client['id']})")
                logger.info(f"   프로토콜: {client['protocol']}, 주소: {client['addr']}")
                logger.info(f"   접속 시간: {conn_time:.0f}초")
        else:
            logger.info("접속된 클라이언트가 없습니다.")
        
        logger.info("=" * 60 + "\n")
    
    def _show_system_status(self):
        """시스템 상태 표시"""
        uptime = time.time() - self.system_status['started_at']
        
        logger.info(f"\n📊 시스템 상태:")
        logger.info(f"- 실행 상태: {'실행중' if self.running else '중지'}")
        logger.info(f"- 가동 시간: {uptime:.0f}초")
        logger.info(f"- 시뮬레이션 모드: {'ON' if SIMULATION_MODE else 'OFF'}")
        logger.info(f"- 연결된 클라이언트: {self.client_manager.get_client_count()}개")
        logger.info(f"- 대기 중인 명령: {command_queue.qsize()}개")
        logger.info(f"- 대기 중인 가구: {len(pending_furn)}개")
        logger.info(f"- 처리된 명령 수: {self.system_status['commands_processed']}개")
        logger.info(f"- 오류 발생 수: {self.system_status['errors_count']}개")
        
        if current_actual and not SIMULATION_MODE:
            logger.info(f"- 로봇 현재 위치: {current_actual}")
            logger.info(f"- 로봇 에러 상태: {robotErrorState}")
        logger.info()
    
    def _run_connection_test(self):
        """연결 테스트 실행"""
        logger.info("🧪 연결 테스트 시작")
        
        # 포트 테스트
        try:
            test_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            test_sock.settimeout(2.0)
            result = test_sock.connect_ex(('127.0.0.1', CLIENT_PORT))
            test_sock.close()
            
            if result == 0:
                logger.info("✅ 로컬 포트 연결 테스트 성공")
            else:
                logger.error(f"❌ 로컬 포트 연결 테스트 실패 (코드: {result})")
        except Exception as e:
            logger.error(f"❌ 포트 테스트 오류: {e}")
        
        # 브로드캐스트 테스트
        if self.client_manager.get_client_count() > 0:
            count = self.client_manager.broadcast_message("TEST_MESSAGE")
            logger.info(f"📡 브로드캐스트 테스트: {count}개 클라이언트에게 전송")
        else:
            logger.info("📡 연결된 클라이언트가 없어 브로드캐스트 테스트 생략")
    
    def _send_message_to_client(self, target_name, message):
        """이름으로 클라이언트 찾아서 메시지 전송"""
        clients = self.client_manager.get_client_list()
        target_client = None
        
        for client in clients:
            if client['name'] == target_name:
                target_client = client
                break
        
        if target_client:
            success = self.client_manager.send_to_client(target_client['id'], message)
            if success:
                logger.info(f"✅ 메시지 전송 성공: '{message}' → {target_name}")
            else:
                logger.error(f"❌ 메시지 전송 실패: {target_name}")
        else:
            logger.error(f"❌ '{target_name}' 클라이언트를 찾을 수 없습니다.")
            if clients:
                logger.info("현재 연결된 클라이언트:")
                for client in clients:
                    logger.info(f"   - {client['name']}")
    
    def pick_and_place(self, furniture_name, marker_pos):
        """Pick & Place 작업 실행"""
        logger.info(f"🤖 Pick & Place 시작: {furniture_name}")
        
        try:
            # 1. 객체 감지
            found_objects = self.vision.capture_and_detect(furniture_name)
            if not found_objects:
                logger.error(f"❌ {furniture_name}을(를) 찾을 수 없습니다.")
                return False
            
            # 2. 첫 번째 객체 선택
            px, py = found_objects[0]
            cls_id = next((k for k, v in self.vision.class_names.items() if v == furniture_name), 0)
            pz = self.vision.class_pick_z[cls_id]
            
            logger.info(f"📍 감지된 위치: ({px}, {py}, {pz})")
            
            # 3. Pick 단계
            logger.info("🔽 Pick 단계 시작")
            self.dobot.run_point([px, py, 0, 0])
            self.dobot.wait_arrive([px, py, 0, 0])
            
            self.dobot.run_point([px, py, pz, 0])
            self.dobot.wait_arrive([px, py, pz, 0])
            
            self.dobot.activate_vacuum_gripper(True)
            time.sleep(1)
            
            self.dobot.run_point([px, py, 0, 0])
            self.dobot.wait_arrive([px, py, 0, 0])
            
            # 4. Place 단계
            logger.info("🔼 Place 단계 시작")
            mx, my = marker_pos
            self.dobot.run_point([mx, my, 0, 0])
            self.dobot.wait_arrive([mx, my, 0, 0])
            
            self.dobot.run_point([mx, my, self.vision.DROP_Z, 0])
            self.dobot.wait_arrive([mx, my, self.vision.DROP_Z, 0])
            
            self.dobot.activate_vacuum_gripper(False)
            time.sleep(1)
            
            self.dobot.blow_off_gripper(True)
            time.sleep(0.3)
            self.dobot.blow_off_gripper(False)
            
            logger.info(f"✅ {furniture_name} Pick & Place 완료")
            return True
            
        except Exception as e:
            logger.error(f"❌ Pick & Place 실행 오류: {e}")
            logger.error(traceback.format_exc())
            return False
    
    def command_loop(self):
        """메인 명령 처리 루프"""
        logger.info("🎯 명령 처리 루프 시작...")
        
        while self.running:
            try:
                client_id, msg = command_queue.get(timeout=0.5)
                
                # 통계 업데이트
                self.system_status['commands_processed'] += 1
                self.system_status['last_activity'] = time.time()
                
                logger.info(f"\n📨 명령 수신: '{msg}' from {client_id or 'Console'}")
                
                # 명령 처리
                success = self._process_command(client_id, msg)
                
                if not success:
                    self.system_status['errors_count'] += 1
                    
            except queue.Empty:
                continue
            except Exception as e:
                logger.error(f"❌ 명령 처리 오류: {e}")
                logger.error(traceback.format_exc())
                self.system_status['errors_count'] += 1
        
        logger.info("📝 명령 처리 루프 종료")
    
    def _process_command(self, client_id, msg):
        """명령 처리"""
        try:
            parts = msg.split()
            
            # 1) 로봇명+목적지+가구 명령
            if len(parts) == 3 and parts[0] in ROBOT_NAMES and parts[1] in DESTINATIONS:
                return self._handle_robot_command(client_id, parts)
            
            # 2) 레거시 목적지+가구 명령
            elif len(parts) == 2 and parts[0] in DESTINATIONS:
                return self._handle_legacy_command(client_id, parts)
            
            # 3) ARRIVED 신호
            elif msg == 'ARRIVED' and client_id in pending_furn:
                return self._handle_arrival(client_id)
            
            # 4) 시스템 제어 명령
            elif msg.lower() in ['stop', 'shutdown', 'exit']:
                logger.info("🛑 원격 종료 명령 수신")
                self.stop()
                return True
            
            # 5) 상태 요청
            elif msg.lower() == 'status':
                self._show_system_status()
                return True
            
            # 6) 기타
            else:
                logger.warning(f"⚠️  처리되지 않은 메시지: '{msg}' from {client_id}")
                self._show_command_help()
                return False
                
        except Exception as e:
            logger.error(f"❌ 명령 처리 실행 오류: {e}")
            return False
    
    def _handle_robot_command(self, client_id, parts):
        """로봇 명령 처리: <로봇명> <목적지> <가구>"""
        robot_name, dest, furn = parts
        target = robot_name
        
        logger.info(f"🤖 로봇 명령: {robot_name} → {dest} (가구: {furn})")
        
        # 대상 로봇 확인
        clients = self.client_manager.get_client_list()
        target_exists = any(client['name'] == target for client in clients)
        
        if target_exists:
            success = self.client_manager.send_to_client(target, dest)
            if success:
                pending_furn[target] = furn
                logger.info(f"📤 목적지 전송 성공: '{dest}' → {target}, 대기 가구: '{furn}'")
                return True
            else:
                logger.error(f"❌ 메시지 전송 실패: {target}")
                return False
        else:
            logger.error(f"❌ {target} 클라이언트가 연결되지 않았습니다.")
            logger.info("💡 현재 연결된 클라이언트:")
            for client in clients:
                logger.info(f"   - {client['name']} ({client['id']})")
            return False
    
    def _handle_legacy_command(self, client_id, parts):
        """레거시 명령 처리: <목적지> <가구>"""
        dest, furn = parts
        target = 'pinky_1' if dest in ['카페', '레스토랑'] else 'pinky_2'
        
        logger.warning(f"⚠️  레거시 명령어 사용: '{dest} {furn}' → 자동 선택: {target}")
        logger.info(f"💡 권장 형식: '{target} {dest} {furn}'")
        
        clients = self.client_manager.get_client_list()
        target_exists = any(client['name'] == target for client in clients)
        
        if target_exists:
            success = self.client_manager.send_to_client(target, dest)
            if success:
                pending_furn[target] = furn
                logger.info(f"📤 목적지 전송 성공: '{dest}' → {target}, 대기 가구: '{furn}'")
                return True
            else:
                logger.error(f"❌ 메시지 전송 실패: {target}")
                return False
        else:
            logger.error(f"❌ {target} 클라이언트가 연결되지 않았습니다.")
            return False
    
    def _handle_arrival(self, client_id):
        """도착 신호 처리"""
        furn = pending_furn.pop(client_id)
        logger.info(f"🚗 {client_id} 도착 → {furn} 작업 시작")
        
        try:
            # 도착 후 안정화 대기
            logger.info("⏳ 안정화 대기 중...")
            time.sleep(3.0)
            
            # 현재 상태 캡처
            logger.info("📸 현재 상태 캡처 중...")
            self.vision.save_annotated_frame()
            
            # ARUCO 마커 감지
            logger.info("🎯 ARUCO 마커 감지 중...")
            marker_pos = self.vision.detect_aruco_marker()
            
            if marker_pos:
                logger.info(f"📍 마커 위치: {marker_pos}")
                
                # Pick & Place 실행
                success = self.pick_and_place(furn, marker_pos)
                
                if success:
                    self.client_manager.send_to_client(client_id, "DONE")
                    logger.info(f"✅ 작업 완료 신호 전송 → {client_id}")
                    return True
                else:
                    self.client_manager.send_to_client(client_id, "ERROR")
                    logger.error(f"❌ 작업 실패 신호 전송 → {client_id}")
                    return False
            else:
                logger.error("❌ ARUCO 마커를 찾을 수 없습니다.")
                self.client_manager.send_to_client(client_id, "ERROR")
                return False
                
        except Exception as e:
            logger.error(f"❌ 도착 처리 오류: {e}")
            logger.error(traceback.format_exc())
            self.client_manager.send_to_client(client_id, "ERROR")
            return False
    
    def _show_command_help(self):
        """명령어 도움말 표시"""
        logger.info(f"💡 지원되는 명령:")
        logger.info(f"   - <로봇명> <목적지> <가구>: {ROBOT_NAMES} + {DESTINATIONS} + [가구명]")
        logger.info(f"   - ARRIVED: 도착 신호")
        logger.info(f"   - status: 시스템 상태")
        logger.info(f"   - stop: 시스템 종료")
        logger.info(f"   - 레거시: <목적지> <가구> (자동 로봇 선택)")
    
    def stop(self):
        """시스템 종료"""
        logger.info("🛑 시스템 종료 중...")
        self.running = False
        
        # 클라이언트들에게 종료 알림
        try:
            self.client_manager.broadcast_message("SYSTEM_SHUTDOWN")
            time.sleep(1)  # 메시지 전송 대기
        except:
            pass
        
        # 클라이언트 서버 중지
        self.client_manager.stop_server()
        
        # 카메라 해제
        if self.vision.cap:
            self.vision.cap.release()
        
        # 시스템 통계 출력
        uptime = time.time() - self.system_status['started_at']
        logger.info(f"📊 시스템 통계:")
        logger.info(f"   - 가동 시간: {uptime:.0f}초")
        logger.info(f"   - 처리된 명령: {self.system_status['commands_processed']}개")
        logger.info(f"   - 발생한 오류: {self.system_status['errors_count']}개")
        
        logger.info("👋 시스템이 종료되었습니다.")


# HTTP API 서버 (개선된 버전)
class ImprovedHTTPServer:
    """개선된 HTTP API 서버"""
    
    def __init__(self, dobot_system, port=8080):
        self.dobot_system = dobot_system
        self.port = port
        self.running = False
    
    def run(self):
        """HTTP 서버 실행"""
        if not FLASK_AVAILABLE:
            logger.warning("⚠️  Flask가 없어 HTTP 서버를 건너뜁니다.")
            return
        
        from flask import Flask, jsonify, request, render_template_string
        
        app = Flask(__name__)
        app.config['JSON_AS_ASCII'] = False  # 한글 지원
        
        # 기본 웹 페이지
        @app.route('/')
        def index():
            return render_template_string('''
            <!DOCTYPE html>
            <html>
            <head>
                <title>Dobot 제어 시스템</title>
                <meta charset="utf-8">
                <style>
                    body { font-family: Arial, sans-serif; margin: 40px; }
                    .container { max-width: 800px; margin: 0 auto; }
                    .status { background: #f0f0f0; padding: 20px; border-radius: 5px; margin: 20px 0; }
                    .clients { background: #e8f4f8; padding: 15px; border-radius: 5px; margin: 10px 0; }
                    button { padding: 10px 20px; margin: 5px; cursor: pointer; }
                    input[type="text"] { padding: 8px; margin: 5px; width: 200px; }
                </style>
            </head>
            <body>
                <div class="container">
                    <h1>🤖 Dobot 통합 제어 시스템</h1>
                    
                    <div class="status">
                        <h3>시스템 상태</h3>
                        <p id="status">로딩 중...</p>
                        <button onclick="updateStatus()">새로고침</button>
                    </div>
                    
                    <div class="clients">
                        <h3>연결된 클라이언트</h3>
                        <div id="clients">로딩 중...</div>
                    </div>
                    
                    <div>
                        <h3>명령 전송</h3>
                        <input type="text" id="command" placeholder="예: pinky_1 카페 테이블">
                        <button onclick="sendCommand()">전송</button>
                        <div id="result"></div>
                    </div>
                </div>
                
                <script>
                    function updateStatus() {
                        fetch('/api/status')
                            .then(response => response.json())
                            .then(data => {
                                document.getElementById('status').innerHTML = 
                                    `실행중: ${data.running}<br>` +
                                    `클라이언트: ${data.clients}개<br>` +
                                    `시뮬레이션: ${data.simulation_mode}<br>` +
                                    `마지막 업데이트: ${new Date(data.timestamp * 1000).toLocaleString()}`;
                            });
                        
                        fetch('/api/clients')
                            .then(response => response.json())
                            .then(data => {
                                let html = '';
                                data.forEach(client => {
                                    html += `<p>${client.name} (${client.protocol}) - ${client.addr}</p>`;
                                });
                                document.getElementById('clients').innerHTML = html || '연결된 클라이언트가 없습니다.';
                            });
                    }
                    
                    function sendCommand() {
                        const command = document.getElementById('command').value;
                        fetch('/api/command', {
                            method: 'POST',
                            headers: {'Content-Type': 'application/json'},
                            body: JSON.stringify({command: command})
                        })
                        .then(response => response.json())
                        .then(data => {
                            document.getElementById('result').innerHTML = 
                                `<p style="color: ${data.error ? 'red' : 'green'}">` +
                                `${data.error || '명령이 큐에 추가되었습니다: ' + data.command}</p>`;
                        });
                    }
                    
                    // 자동 업데이트
                    updateStatus();
                    setInterval(updateStatus, 5000);
                </script>
            </body>
            </html>
            ''')
        
        @app.route('/api/status')
        def status():
            return jsonify({
                'running': self.dobot_system.running,
                'clients': self.dobot_system.client_manager.get_client_count(),
                'simulation_mode': SIMULATION_MODE,
                'available_robots': ROBOT_NAMES,
                'destinations': DESTINATIONS,
                'commands_processed': self.dobot_system.system_status['commands_processed'],
                'errors_count': self.dobot_system.system_status['errors_count'],
                'uptime': time.time() - self.dobot_system.system_status['started_at'],
                'timestamp': time.time()
            })
        
        @app.route('/api/clients')
        def clients():
            return jsonify(self.dobot_system.client_manager.get_client_list())
        
        @app.route('/api/command', methods=['POST'])
        def command():
            try:
                data = request.get_json()
                if data and 'command' in data:
                    command_queue.put((None, data['command']))
                    return jsonify({'status': 'queued', 'command': data['command']})
                return jsonify({'error': 'Invalid command'}), 400
            except Exception as e:
                return jsonify({'error': str(e)}), 500
        
        @app.route('/api/broadcast', methods=['POST'])
        def broadcast():
            try:
                data = request.get_json()
                if data and 'message' in data:
                    count = self.dobot_system.client_manager.broadcast_message(data['message'])
                    return jsonify({'status': 'sent', 'count': count, 'message': data['message']})
                return jsonify({'error': 'Invalid message'}), 400
            except Exception as e:
                return jsonify({'error': str(e)}), 500
        
        try:
            logger.info(f"🌐 HTTP API 서버 시작: http://0.0.0.0:{self.port}")
            logger.info(f"📋 웹 인터페이스: http://localhost:{self.port}")
            logger.info(f"📋 API 엔드포인트:")
            logger.info(f"   - GET / - 웹 인터페이스")
            logger.info(f"   - GET /api/status - 시스템 상태")
            logger.info(f"   - GET /api/clients - 클라이언트 목록")
            logger.info(f"   - POST /api/command - 명령 전송")
            logger.info(f"   - POST /api/broadcast - 브로드캐스트")
            
            app.run(host='0.0.0.0', port=self.port, debug=False, threaded=True)
        except Exception as e:
            logger.error(f"❌ HTTP 서버 오류: {e}")


def test_client_connection():
    """클라이언트 연결 테스트 함수"""
    import socket
    
    logger.info("🧪 클라이언트 연결 테스트 시작")
    
    try:
        # ROS2 클라이언트 시뮬레이션
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect(('127.0.0.1', CLIENT_PORT))
        
        # 등록
        sock.sendall(b'pinky_1\n')
        response = sock.recv(1024).decode().strip()
        logger.info(f"등록 응답: {response}")
        
        # 테스트 메시지 전송
        test_commands = [
            'pinky_1 카페 테이블',
            'ARRIVED',
            'status'
        ]
        
        for cmd in test_commands:
            logger.info(f"테스트 명령 전송: {cmd}")
            sock.sendall(f'{cmd}\n'.encode())
            response = sock.recv(1024).decode().strip()
            logger.info(f"응답: {response}")
            time.sleep(1)
        
        sock.close()
        logger.info("✅ 클라이언트 테스트 완료")
        
    except Exception as e:
        logger.error(f"❌ 클라이언트 테스트 실패: {e}")


def main():
    """메인 함수"""
    try:
        # 시스템 생성
        system = ImprovedIntegratedDobotSystem()
        
        # HTTP 서버 백그라운드 실행
        http_server = ImprovedHTTPServer(system, PORT)
        threading.Thread(target=http_server.run, daemon=True).start()
        
        # 약간의 지연 후 테스트 실행 (옵션)
        # threading.Timer(5.0, test_client_connection).start()
        
        # 메인 시스템 시작
        system.start()
        
    except KeyboardInterrupt:
        logger.info("\n⚠️  사용자에 의해 종료됨")
        if 'system' in locals():
            system.stop()
    except Exception as e:
        logger.error(f"❌ 시스템 오류: {e}")
        logger.error(traceback.format_exc())
        if 'system' in locals():
            system.stop()


if __name__ == "__main__":
    # 명령행 인수 처리
    if len(sys.argv) > 1:
        if sys.argv[1] == '--test-client':
            test_client_connection()
        elif sys.argv[1] == '--help':
            print("사용법:")
            print("  python improved_dobot_server.py          # 서버 실행")
            print("  python improved_dobot_server.py --test-client  # 클라이언트 테스트")
            print("  python improved_dobot_server.py --help        # 도움말")
        else:
            main()
    else:
        main()
