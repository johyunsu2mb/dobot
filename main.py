#!/usr/bin/env python3
"""
통합 Dobot 제어 시스템
- 로봇 제어 및 YOLO 객체 감지 기반 pick & place
- 클라이언트 관리 및 메시지 전송
- TCP 서버 및 HTTP API 서버
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

# 로깅 설정
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# --- 설정 상수 ---
HOST = '127.0.0.1'
PORT = 8080
CLIENT_PORT = 9999  # ROS2 클라이언트가 연결하는 포트
DESTINATIONS = ['카페', '레스토랑', '건담베이스', '실내골프장']
ROBOT_NAMES = ['pinky_1', 'pinky_2']  # 사용 가능한 로봇 이름 목록

# --- 전역 변수 ---
command_queue = queue.Queue()   # (client_id, msg)
clients = {}                    # client_id -> socket
pending_furn = {}              # client_id -> furniture

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
        print("🎮 [시뮬레이션] 로봇 활성화")
        return "EnableRobot() 시뮬레이션"
        
    def MovL(self, x, y, z, r):
        print(f"🎮 [시뮬레이션] 이동: ({x}, {y}, {z}, {r})")
        self.position = [x, y, z, r]
        time.sleep(0.5)  # 이동 시간 시뮬레이션
        
    def DO(self, port, value):
        if port == 1:  # 진공 그리퍼
            self.vacuum_state = bool(value)
            print(f"🎮 [시뮬레이션] 진공 그리퍼: {'ON' if value else 'OFF'}")
        elif port == 2:  # 블로우 오프
            self.blow_state = bool(value)
            print(f"🎮 [시뮬레이션] 블로우 오프: {'ON' if value else 'OFF'}")
            
    def GetErrorID(self):
        return "[0]"  # 에러 없음
        
    def ClearError(self):
        print("🎮 [시뮬레이션] 에러 클리어")
        
    def Continue(self):
        print("🎮 [시뮬레이션] 계속")


class ClientManager:
    """클라이언트 연결 관리 및 메시지 전송 클래스"""
    
    def __init__(self, host='0.0.0.0', port=9999):  # 모든 인터페이스에서 수신
        self.host = host
        self.port = port
        self.clients = {}  # { client_id: {'conn': socket, 'name': str, 'addr': tuple} }
        self.lock = threading.Lock()
        self.server_socket = None
        self.running = False
        self.client_counter = 0
        
    def start_server(self):
        """TCP 서버 시작"""
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            
            # 소켓 재사용 옵션 추가
            if hasattr(socket, 'SO_REUSEPORT'):
                self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
            
            self.server_socket.bind((self.host, self.port))
            self.server_socket.listen(10)  # backlog 증가
            self.running = True
            
            print(f"📡 클라이언트 서버 시작: {self.host}:{self.port}")
            print(f"🔗 연결 대기 중... (backlog: 10)")
            
            # 클라이언트 연결 대기 스레드 시작
            threading.Thread(target=self.accept_clients, daemon=True).start()
            
            return True
            
        except Exception as e:
            print(f"❌ 클라이언트 서버 시작 실패: {e}")
            print(f"🔍 포트 {self.port} 사용 중인지 확인: lsof -i :{self.port}")
            self.running = False
            return False
    
    def accept_clients(self):
        """클라이언트 연결 수락"""
        print("🔄 클라이언트 연결 수락 스레드 시작")
        
        while self.running:
            try:
                print(f"⏳ 클라이언트 연결 대기 중... (포트: {self.port})")
                self.server_socket.settimeout(1.0)  # 1초 타임아웃으로 주기적 확인
                
                try:
                    conn, addr = self.server_socket.accept()
                    print(f"🔌 새 연결 수락: {addr}")
                    threading.Thread(target=self.handle_client, args=(conn, addr), daemon=True).start()
                except socket.timeout:
                    continue  # 타임아웃은 정상, 다시 대기
                    
            except Exception as e:
                if self.running:
                    print(f"❌ 클라이언트 연결 수락 오류: {e}")
                    time.sleep(1)  # 오류 시 잠시 대기
                break
        
        print("🔄 클라이언트 연결 수락 스레드 종료")
    
    def handle_client(self, conn, addr):
        """클라이언트 연결 처리 - ROS2 클라이언트와 호환성 강화"""
        client_id = None
        try:
            print(f"New connection: {addr}")
            
            # 첫 번째 메시지 대기 (더 긴 타임아웃)
            conn.settimeout(30.0)
            first_data = conn.recv(1024)
            if not first_data:
                return
                
            first_msg = first_data.decode().strip()
            print(f"First message received: '{first_msg}'")
            
            # 1) ROS2 클라이언트 프로토콜: pinky_1 또는 pinky_2
            if first_msg in ['pinky_1', 'pinky_2']:
                client_id = first_msg
                with self.lock:
                    self.clients[client_id] = {
                        'conn': conn,
                        'name': client_id,
                        'addr': addr,
                        'protocol': 'ros2',
                        'connected_at': time.time()
                    }
                print(f"✅ ROS2 클라이언트 등록: {client_id}")
                
                # ROS2 클라이언트에게 등록 확인 전송 (개행 포함)
                conn.sendall(b"REGISTERED\n")
                
                # ROS2 프로토콜로 메시지 처리
                conn.settimeout(None)
                while True:
                    try:
                        data = conn.recv(1024)
                        if not data:
                            break
                        msg = data.decode().strip()
                        print(f"📨 ROS2 메시지 수신 [{client_id}]: '{msg}'")
                        
                        # 명령 큐에 추가
                        command_queue.put((client_id, msg))
                        
                        # ACK 응답 전송
                        conn.sendall(b"ACK\n")
                        
                    except socket.timeout:
                        continue
                    except Exception as e:
                        print(f"ROS2 메시지 수신 오류: {e}")
                        break
            
            # 2) 새 프로토콜: JSON 기반
            else:
                # 환영 메시지 전송
                conn.sendall(b"CONNECTED:Please enter your name\n")
                
                # 클라이언트 이름 등록
                name_data = first_msg if first_msg else conn.recv(1024).decode('utf-8').strip()
                
                if not name_data:
                    conn.sendall(b"ERROR:Name required\n")
                    return
                
                # 고유 클라이언트 ID 생성
                with self.lock:
                    self.client_counter += 1
                    client_id = f"{name_data}_{self.client_counter}_{int(time.time())}"
                    self.clients[client_id] = {
                        'conn': conn,
                        'name': name_data,
                        'addr': addr,
                        'connected_at': time.time(),
                        'protocol': 'json'
                    }
                
                # 환영 메시지 전송
                welcome_msg = f"WELCOME:{client_id}\n"
                conn.sendall(welcome_msg.encode('utf-8'))
                
                print(f"✅ JSON 클라이언트 등록: {name_data} (ID: {client_id})")
                
                # JSON 프로토콜로 메시지 처리
                conn.settimeout(None)
                while True:
                    data = conn.recv(1024)
                    if not data:
                        break
                    message = data.decode('utf-8').strip()
                    print(f"📨 JSON 메시지 수신 [{name_data}]: {message}")
                    
                    # JSON 또는 일반 텍스트 처리
                    try:
                        msg_data = json.loads(message)
                        self.process_client_message(client_id, msg_data)
                    except json.JSONDecodeError:
                        # 일반 텍스트는 command_queue에 추가
                        command_queue.put((client_id, message))
                        
        except socket.timeout:
            print(f"⏰ 클라이언트 연결 타임아웃: {addr}")
        except Exception as e:
            print(f"❌ 클라이언트 처리 오류 [{client_id or addr}]: {e}")
            import traceback
            traceback.print_exc()
        finally:
            # 연결 종료 처리
            if client_id:
                with self.lock:
                    client_info = self.clients.pop(client_id, None)
                if client_info:
                    print(f"🔌 클라이언트 연결 해제: {client_info['name']} ({client_id})")
            try:
                conn.close()
            except:
                pass
    
    def process_client_message(self, client_id, msg_data):
        """클라이언트로부터 받은 JSON 메시지 처리"""
        try:
            if isinstance(msg_data, dict):
                msg_type = msg_data.get('type', 'unknown')
                content = msg_data.get('content', '')
                
                client_info = self.clients.get(client_id, {})
                client_name = client_info.get('name', 'Unknown')
                
                print(f"📨 {client_name}로부터 메시지: {msg_type} - {content}")
                
                if msg_type == 'dobot_command':
                    # Dobot 명령을 command_queue에 추가
                    command_queue.put((client_id, content))
                elif msg_type == 'status_request':
                    self.send_status_to_client(client_id)
                    
        except Exception as e:
            print(f"❌ 메시지 처리 오류: {e}")
    
    def send_to_client(self, target_id, message):
        """특정 클라이언트에게 메시지 전송"""
        with self.lock:
            client_info = self.clients.get(target_id)
        
        if client_info:
            try:
                conn = client_info['conn']
                if isinstance(message, str):
                    message = message.encode('utf-8')
                if not message.endswith(b'\n'):
                    message += b'\n'
                conn.sendall(message)
                print(f"📤 [발신 {client_info['name']}] {message.decode().strip()}")
                return True
            except Exception as e:
                print(f"❌ [오류] '{client_info['name']}' 전송 실패: {e}")
                return False
        else:
            print(f"❌ [오류] ID '{target_id}'를 찾을 수 없습니다.")
            return False
    
    def send_status_to_client(self, client_id):
        """클라이언트에게 상태 정보 전송"""
        with self.lock:
            client_list = [
                {
                    "id": cid,
                    "name": info['name'],
                    "connected_at": info.get('connected_at', time.time())
                }
                for cid, info in self.clients.items()
            ]
        
        status = {
            "type": "status_response",
            "connected_clients": len(self.clients),
            "client_list": client_list,
            "server_running": self.running,
            "timestamp": time.time()
        }
        self.send_to_client(client_id, json.dumps(status))
    
    def stop_server(self):
        """서버 중지"""
        self.running = False
        if self.server_socket:
            try:
                self.server_socket.close()
            except:
                pass
        print("클라이언트 서버가 중지되었습니다.")


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
            print("🎮 시뮬레이션 모드로 연결")
            return True
            
        if not DOBOT_API_AVAILABLE:
            print("❌ Dobot API가 없습니다.")
            return False
            
        try:
            self.dashboard = DobotApiDashboard(ip, dash_port)
            self.move = DobotApiMove(ip, move_port)
            self.feed = DobotApi(ip, feed_port)
            
            self.dashboard.EnableRobot()
            print(f"✅ Dobot 연결 성공: {ip}")
            
            if self.feed:
                threading.Thread(target=self.get_feed, daemon=True).start()
                threading.Thread(target=self.clear_robot_error, daemon=True).start()
            
            return True
        except Exception as e:
            print(f"❌ Dobot 연결 실패: {e}")
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
                print(f"피드백 수신 오류: {e}")
                break
    
    def wait_arrive(self, target):
        """현재 좌표가 target에 도달할 때까지 대기"""
        if self.simulation_mode:
            time.sleep(1)  # 시뮬레이션에서는 1초 대기
            return
            
        while True:
            with state_lock:
                if current_actual is not None and all(
                    abs(current_actual[i] - target[i]) <= 1 for i in range(len(target))
                ):
                    return
            time.sleep(0.001)
    
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
                            print(f"Robot Alarm ID={c}: {desc}")
                        
                        # 자동으로 에러 클리어 (입력 대기 제거)
                        self.dashboard.ClearError()
                        time.sleep(0.01)
                        self.dashboard.Continue()
                        print("자동으로 에러를 클리어했습니다.")
                except Exception as e:
                    print(f"에러 처리 중 오류: {e}")
            else:
                if q and en and int(en[0])==1 and int(q[0])==0:
                    self.dashboard.Continue()
                    
            time.sleep(5)
    
    def run_point(self, pt):
        """지정된 좌표로 이동"""
        if self.move:
            self.move.MovL(pt[0], pt[1], pt[2], pt[3])
    
    def activate_vacuum_gripper(self, on):
        """진공 그리퍼 제어"""
        if self.dashboard:
            self.dashboard.DO(1, 1 if on else 0)
            print(f"Vacuum gripper {'on' if on else 'off'}")
    
    def blow_off_gripper(self, on):
        """블로우 오프 제어"""
        if self.dashboard:
            self.dashboard.DO(2, 1 if on else 0)
            print(f"Blow-off {'ON' if on else 'OFF'}")


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
                print("⚠️  카메라를 열 수 없습니다. 시뮬레이션 모드로 실행됩니다.")
                self.cap = None
            
            # YOLO 모델 로드
            if YOLO_AVAILABLE and os.path.exists(model_path):
                self.model = YOLO(model_path)
                print(f"✅ YOLO 모델 로드: {model_path}")
            else:
                print("⚠️  YOLO 모델이 없습니다. 객체 감지가 비활성화됩니다.")
            
            # ARUCO 감지기 초기화
            aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
            self.detector = cv2.aruco.ArucoDetector(aruco_dict, cv2.aruco.DetectorParameters())
            
            # Homography 행렬 설정
            self.setup_homography()
            
        except Exception as e:
            print(f"❌ 비전 시스템 초기화 실패: {e}")
    
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
            print("✅ Homography 행렬 설정 완료")
        except Exception as e:
            print(f"❌ Homography 설정 실패: {e}")
            self.H = np.eye(3)  # 단위 행렬로 설정
    
    def capture_and_detect(self, furniture_name):
        """프레임 캡처 및 객체 감지"""
        if not self.cap or not self.model:
            print("🎮 [시뮬레이션] 객체 감지")
            return [(300.0, 0.0)]  # 시뮬레이션 좌표 반환
        
        ret, frame = self.cap.read()
        if not ret:
            print("❌ 카메라 읽기 실패")
            return []
        
        # YOLO 감지
        detections = self.model(frame)[0]
        
        # 클래스 필터링
        cls_id = next((k for k, v in self.class_names.items() if v == furniture_name), None)
        if cls_id is None:
            print(f"❌ 잘못된 가구명: {furniture_name}")
            return []
        
        found = []
        for box in detections.boxes:
            if int(box.cls[0]) != cls_id:
                continue
                
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cx, cy = (x1 + x2)//2, (y1 + y2)//2
            
            # 좌표 변환
            uv = np.array([cx, cy, 1.0])
            w = self.H.dot(uv)
            w /= w[2]
            wx, wy = float(w[0]), float(w[1])
            
            if wx <= 380.0:  # 작업 영역 내부만
                found.append((wx, wy))
        
        return found
    
    def detect_aruco_marker(self):
        """ARUCO 마커 감지"""
        if not self.cap or not self.detector:
            print("🎮 [시뮬레이션] ARUCO 마커 감지")
            return (350.0, 0.0)  # 시뮬레이션 좌표
        
        ret, frame = self.cap.read()
        if not ret:
            print("❌ 카메라 읽기 실패")
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
            print("🎮 [시뮬레이션] 프레임 저장")
            return
        
        ret, frame = self.cap.read()
        if not ret:
            return
        
        # YOLO 박스 그리기
        detections = self.model(frame)[0]
        for box in detections.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cls = int(box.cls[0])
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, str(cls), (x1, y1-5),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        # ARUCO 마커 그리기
        corners, ids, _ = self.detector.detectMarkers(frame)
        if ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        
        cv2.imwrite(filename, frame)
        print(f"📸 주석 이미지 저장: {filename}")


class IntegratedDobotSystem:
    """통합 Dobot 시스템"""
    
    def __init__(self):
        self.client_manager = ClientManager(HOST, CLIENT_PORT)
        self.dobot = DobotController(SIMULATION_MODE)
        self.vision = VisionSystem()
        self.running = False
        
    def start(self):
        """시스템 시작"""
        print("\n" + "=" * 60)
        print("  🤖 통합 Dobot 제어 시스템 시작")
        print("=" * 60)
        
        # 클라이언트 서버 시작
        if not self.client_manager.start_server():
            print("❌ 클라이언트 서버 시작 실패. 종료합니다.")
            return
        
        # 서버 준비 확인
        time.sleep(2)
        print(f"🌐 서버 상태 확인...")
        print(f"  - 바인드 주소: {self.client_manager.host}:{self.client_manager.port}")
        print(f"  - 실행 상태: {self.client_manager.running}")
        
        # 포트 확인
        try:
            test_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            result = test_sock.connect_ex(('127.0.0.1', self.client_manager.port))
            test_sock.close()
            if result == 0:
                print(f"✅ 포트 {self.client_manager.port} 연결 가능")
            else:
                print(f"❌ 포트 {self.client_manager.port} 연결 불가")
        except Exception as e:
            print(f"⚠️  포트 테스트 오류: {e}")
        
        # Dobot 연결
        print("\n🤖 Dobot 연결 시도...")
        if not self.dobot.connect():
            print("⚠️  Dobot 연결 실패. 시뮬레이션 모드로 계속합니다.")
        
        # 기본 위치로 이동
        base = [294.0, 2.0, 0.0, 0.0]
        print(f"📍 기본 위치로 이동: {base}")
        self.dobot.run_point(base)
        self.dobot.wait_arrive(base)
        self.dobot.activate_vacuum_gripper(False)
        
        print("\n✅ 시스템 준비 완료")
        print(f"📡 클라이언트 서버: {self.client_manager.host}:{self.client_manager.port}")
        print("💡 사용법:")
        print("- 'pinky_1' 또는 'pinky_2'로 레거시 클라이언트 연결")
        print("- 일반 이름으로 JSON 클라이언트 연결")
        print("- '<로봇명> <목적지> <가구>' 형식으로 명령 전송")
        print("- 'ARRIVED' 메시지로 도착 신호")
        print(f"\n🔗 ROS2 클라이언트 연결 명령:")
        print(f"ros2 run dobot_navigation pinky_nav_client --ros-args -p server_host:={self.client_manager.host} -p server_port:={self.client_manager.port}")
        
        # 콘솔 명령 처리 스레드 시작
        threading.Thread(target=self.console_command_loop, daemon=True).start()
        
        # 메인 명령 처리 루프 시작
        self.running = True
        self.command_loop()
    
    def console_command_loop(self):
        """콘솔 명령 처리"""
        print("\n📋 콘솔 명령어:")
        print("  list - 접속자 목록")
        print("  send <이름> <메시지> - 메시지 전송")
        print("  status - 시스템 상태")
        print("  <로봇명> <목적지> <가구> - pick & place 명령")
        print("  stop - 시스템 종료")
        print("  help - 도움말 다시 보기\n")
        
        import sys
        
        while self.running:
            try:
                # 프롬프트 출력 후 flush
                sys.stdout.write("시스템> ")
                sys.stdout.flush()
                
                # 입력 받기
                line = sys.stdin.readline()
                if not line:  # EOF
                    break
                    
                line = line.strip()
                if not line:
                    continue
                
                # 명령 파싱
                parts = line.split()
                if not parts:
                    continue
                    
                command = parts[0].lower()
                
                # 명령 처리
                if command == 'list':
                    self.show_client_list()
                    
                elif command == 'status':
                    self.show_system_status()
                    
                elif command == 'help':
                    print("\n📋 콘솔 명령어:")
                    print("  list - 접속자 목록")
                    print("  send <이름> <메시지> - 메시지 전송")
                    print("  status - 시스템 상태")
                    print("  <로봇명> <목적지> <가구> - pick & place 명령")
                    print("  stop - 시스템 종료")
                    print("  help - 도움말 다시 보기\n")
                    
                elif command == 'stop' or command == 'exit' or command == 'quit':
                    print("시스템을 종료합니다.")
                    self.stop()
                    break
                    
                elif command == 'send' and len(parts) >= 3:
                    target_name = parts[1]
                    message = ' '.join(parts[2:])  # 공백 포함 메시지
                    self.send_message_to_client(target_name, message)
                    
                elif len(parts) == 3 and parts[0] in ROBOT_NAMES and parts[1] in DESTINATIONS:
                    # 직접 pick & place 명령: <로봇명> <목적지> <가구>
                    robot_name, dest, furn = parts
                    print(f"📤 Pick & Place 명령 추가: {robot_name} {dest} {furn}")
                    command_queue.put((None, line))
                    
                elif command == 'test':
                    # 테스트 명령
                    print("🧪 시스템 테스트")
                    print(f"- 큐 크기: {command_queue.qsize()}")
                    print(f"- 클라이언트 수: {len(self.client_manager.clients)}")
                    print(f"- 실행 상태: {self.running}")
                    
                else:
                    print(f"❌ 알 수 없는 명령어: '{command}'")
                    print("💡 'help' 명령으로 사용법을 확인하세요.")
                    
            except EOFError:
                print("\n입력 종료 (EOF)")
                break
            except KeyboardInterrupt:
                print("\n키보드 중단 (Ctrl+C)")
                break
            except Exception as e:
                print(f"❌ 명령 처리 오류: {e}")
                import traceback
                traceback.print_exc()
    
    def show_client_list(self):
        """클라이언트 목록 표시"""
        with self.client_manager.lock:
            client_list = list(self.client_manager.clients.items())
        
        print("\n" + "=" * 50)
        print(f"접속자 목록 ({len(client_list)}명)")
        print("=" * 50)
        if client_list:
            for i, (client_id, info) in enumerate(client_list, 1):
                protocol = info.get('protocol', 'unknown')
                connected_time = time.time() - info.get('connected_at', time.time())
                print(f"{i}. {info['name']} (ID: {client_id})")
                print(f"   프로토콜: {protocol}, 접속 시간: {int(connected_time)}초")
        else:
            print("접속된 클라이언트가 없습니다.")
        print("=" * 50 + "\n")
    
    def show_system_status(self):
        """시스템 상태 표시"""
        print(f"\n📊 시스템 상태:")
        print(f"- 실행 상태: {'실행중' if self.running else '중지'}")
        print(f"- 시뮬레이션 모드: {'ON' if SIMULATION_MODE else 'OFF'}")
        print(f"- 연결된 클라이언트: {len(self.client_manager.clients)}개")
        print(f"- 대기 중인 명령: {command_queue.qsize()}개")
        print(f"- 대기 중인 가구: {len(pending_furn)}개")
        print(f"- 사용 가능한 로봇: {', '.join(ROBOT_NAMES)}")
        print(f"- 지원 목적지: {', '.join(DESTINATIONS)}")
        
        if current_actual and not SIMULATION_MODE:
            print(f"- 로봇 현재 위치: {current_actual}")
            print(f"- 로봇 에러 상태: {robotErrorState}")
        print()
    
    def send_message_to_client(self, target_name, message):
        """이름으로 클라이언트 찾아서 메시지 전송"""
        target_id = None
        with self.client_manager.lock:
            for cid, info in self.client_manager.clients.items():
                if info['name'] == target_name:
                    target_id = cid
                    break
        
        if target_id:
            # 프로토콜에 따라 메시지 형식 결정
            client_info = self.client_manager.clients.get(target_id, {})
            protocol = client_info.get('protocol', 'legacy')
            
            if protocol == 'json':
                msg = json.dumps({
                    "type": "server_message",
                    "from": "System",
                    "content": message,
                    "timestamp": time.time()
                })
            else:
                msg = message
                
            self.client_manager.send_to_client(target_id, msg)
        else:
            print(f"❌ '{target_name}' 클라이언트를 찾을 수 없습니다.")
    
    def pick_and_place(self, furniture_name, marker_pos):
        """Pick & Place 작업 실행"""
        print(f"🤖 Pick & Place 시작: {furniture_name}")
        
        # 1. 객체 감지
        found_objects = self.vision.capture_and_detect(furniture_name)
        if not found_objects:
            print(f"❌ {furniture_name}을(를) 찾을 수 없습니다.")
            return False
        
        # 2. 첫 번째 객체 선택
        px, py = found_objects[0]
        cls_id = next((k for k, v in self.vision.class_names.items() if v == furniture_name), 0)
        pz = self.vision.class_pick_z[cls_id]
        
        print(f"📍 감지된 위치: ({px}, {py}, {pz})")
        
        # 3. Pick 단계
        print("🔽 Pick 단계 시작")
        self.dobot.run_point([px, py, 0, 0])
        self.dobot.wait_arrive([px, py, 0, 0])
        
        self.dobot.run_point([px, py, pz, 0])
        self.dobot.wait_arrive([px, py, pz, 0])
        
        self.dobot.activate_vacuum_gripper(True)
        time.sleep(1)
        
        self.dobot.run_point([px, py, 0, 0])
        self.dobot.wait_arrive([px, py, 0, 0])
        
        # 4. Place 단계
        print("🔼 Place 단계 시작")
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
        
        print(f"✅ {furniture_name} Pick & Place 완료")
        return True
    
    def command_loop(self):
        """메인 명령 처리 루프"""
        print("🎯 명령 처리 루프 시작...")
        
        while self.running:
            try:
                # 타임아웃을 짧게 설정하여 빠른 종료 가능
                client_id, msg = command_queue.get(timeout=0.5)
                
                print(f"\n📨 명령 수신: '{msg}' from {client_id or 'Console'}")
                
                # 1) 로봇명+목적지+가구 명령: <로봇명> <목적지> <가구>
                parts = msg.split()
                if len(parts) == 3 and parts[0] in ROBOT_NAMES and parts[1] in DESTINATIONS:
                    robot_name, dest, furn = parts
                    target = robot_name  # 명시적으로 지정된 로봇 사용
                    
                    # 대상 클라이언트에게 목적지 전송
                    with self.client_manager.lock:
                        target_exists = target in self.client_manager.clients
                    
                    if target_exists:
                        success = self.client_manager.send_to_client(target, dest)
                        if success:
                            pending_furn[target] = furn
                            print(f"📤 '{dest}' 전송 → {target}, 대기 가구: '{furn}'")
                        else:
                            print(f"❌ {target}에게 메시지 전송 실패")
                    else:
                        print(f"❌ {target} 클라이언트가 연결되지 않았습니다.")
                        print("💡 현재 연결된 클라이언트:")
                        with self.client_manager.lock:
                            for cid, info in self.client_manager.clients.items():
                                print(f"   - {info['name']} ({cid})")
                
                # 2) 레거시 목적지+가구 명령 (하위 호환성)
                elif len(parts) == 2 and parts[0] in DESTINATIONS:
                    dest, furn = parts
                    # 기존 로직: 목적지에 따라 자동 선택
                    target = 'pinky_1' if dest in ['카페', '레스토랑'] else 'pinky_2'
                    
                    print(f"⚠️  레거시 명령어 사용: '{dest} {furn}' → 자동 선택: {target}")
                    print(f"💡 권장 형식: '{target} {dest} {furn}'")
                    
                    # 대상 클라이언트에게 목적지 전송
                    with self.client_manager.lock:
                        target_exists = target in self.client_manager.clients
                    
                    if target_exists:
                        success = self.client_manager.send_to_client(target, dest)
                        if success:
                            pending_furn[target] = furn
                            print(f"📤 '{dest}' 전송 → {target}, 대기 가구: '{furn}'")
                        else:
                            print(f"❌ {target}에게 메시지 전송 실패")
                    else:
                        print(f"❌ {target} 클라이언트가 연결되지 않았습니다.")
                        print("💡 현재 연결된 클라이언트:")
                        with self.client_manager.lock:
                            for cid, info in self.client_manager.clients.items():
                                print(f"   - {info['name']} ({cid})")
                
                # 3) ARRIVED 신호 처리
                elif msg == 'ARRIVED' and client_id in pending_furn:
                    furn = pending_furn.pop(client_id)
                    print(f"🚗 {client_id} 도착 → {furn} 작업 시작")
                    
                    # 도착 후 잠시 대기
                    print("⏳ 안정화 대기 중...")
                    time.sleep(3.0)
                    
                    # 주석 이미지 저장
                    print("📸 현재 상태 캡처 중...")
                    self.vision.save_annotated_frame()
                    
                    # ARUCO 마커 감지
                    print("🎯 ARUCO 마커 감지 중...")
                    marker_pos = self.vision.detect_aruco_marker()
                    
                    if marker_pos:
                        print(f"📍 마커 위치: {marker_pos}")
                        # Pick & Place 실행
                        success = self.pick_and_place(furn, marker_pos)
                        
                        if success:
                            # 완료 신호 전송
                            self.client_manager.send_to_client(client_id, "DONE")
                            print(f"✅ 작업 완료 신호 전송 → {client_id}")
                        else:
                            self.client_manager.send_to_client(client_id, "ERROR")
                            print(f"❌ 작업 실패 신호 전송 → {client_id}")
                    else:
                        print("❌ ARUCO 마커를 찾을 수 없습니다.")
                        self.client_manager.send_to_client(client_id, "ERROR")
                
                # 4) 시스템 제어 명령
                elif msg.lower() in ['stop', 'shutdown', 'exit']:
                    print("🛑 원격 종료 명령 수신")
                    self.stop()
                    break
                
                # 5) 상태 요청
                elif msg.lower() == 'status':
                    self.show_system_status()
                
                # 6) 기타 메시지
                else:
                    print(f"⚠️  처리되지 않은 메시지: '{msg}' from {client_id}")
                    print(f"💡 지원되는 명령:")
                    print(f"   - <로봇명> <목적지> <가구>: {ROBOT_NAMES} + {DESTINATIONS} + [가구명]")
                    print(f"   - ARRIVED, status, stop")
                    print(f"   - 레거시: <목적지> <가구> (자동 로봇 선택)")
                    
            except queue.Empty:
                # 타임아웃은 정상적인 상황 - 로그 출력 안 함
                continue
            except Exception as e:
                print(f"❌ 명령 처리 오류: {e}")
                import traceback
                traceback.print_exc()
        
        print("📝 명령 처리 루프 종료")
    
    def stop(self):
        """시스템 종료"""
        print("🛑 시스템 종료 중...")
        self.running = False
        
        # 클라이언트 서버 중지
        self.client_manager.stop_server()
        
        # 카메라 해제
        if self.vision.cap:
            self.vision.cap.release()
        
        print("👋 시스템이 종료되었습니다.")


# HTTP API 서버 (선택사항)
class SimpleHTTPServer:
    """간단한 HTTP API 서버"""
    
    def __init__(self, dobot_system, port=8080):
        self.dobot_system = dobot_system
        self.port = port
        self.running = False
    
    def run(self):
        """HTTP 서버 실행"""
        if not FLASK_AVAILABLE:
            print("⚠️  Flask가 없어 HTTP 서버를 건너뜁니다.")
            return
        
        from flask import Flask, jsonify, request
        
        app = Flask(__name__)
        
        @app.route('/api/status')
        def status():
            return jsonify({
                'running': self.dobot_system.running,
                'clients': len(self.dobot_system.client_manager.clients),
                'simulation_mode': SIMULATION_MODE,
                'available_robots': ROBOT_NAMES,
                'destinations': DESTINATIONS,
                'timestamp': time.time()
            })
        
        @app.route('/api/clients')
        def clients():
            with self.dobot_system.client_manager.lock:
                client_list = [
                    {
                        'id': cid,
                        'name': info['name'],
                        'protocol': info.get('protocol', 'unknown')
                    }
                    for cid, info in self.dobot_system.client_manager.clients.items()
                ]
            return jsonify(client_list)
        
        @app.route('/api/command', methods=['POST'])
        def command():
            data = request.get_json()
            if data and 'command' in data:
                command_queue.put((None, data['command']))
                return jsonify({'status': 'queued', 'command': data['command']})
            return jsonify({'error': 'Invalid command'}), 400
        
        try:
            print(f"🌐 HTTP API 서버 시작: http://localhost:{self.port}")
            print(f"📋 API 엔드포인트:")
            print(f"   - GET /api/status - 시스템 상태")
            print(f"   - GET /api/clients - 클라이언트 목록")
            print(f"   - POST /api/command - 명령 전송")
            app.run(host='0.0.0.0', port=self.port, debug=False)
        except Exception as e:
            print(f"❌ HTTP 서버 오류: {e}")


def main():
    """메인 함수"""
    try:
        # 시스템 생성 및 시작
        system = IntegratedDobotSystem()
        
        # HTTP 서버 백그라운드 실행
        http_server = SimpleHTTPServer(system, PORT)
        threading.Thread(target=http_server.run, daemon=True).start()
        
        # 메인 시스템 시작
        system.start()
        
    except KeyboardInterrupt:
        print("\n⚠️  사용자에 의해 종료됨")
        if 'system' in locals():
            system.stop()
    except Exception as e:
        print(f"❌ 시스템 오류: {e}")
        import traceback
        traceback.print_exc()
        if 'system' in locals():
            system.stop()


if __name__ == "__main__":
    main()