#!/usr/bin/env python3
"""
Enhanced Dobot ROS2 Client Node
TCP와 ROS2 통신을 모두 지원하는 하이브리드 클라이언트
"""

import socket
import threading
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Pose, Point
import json
import time
import sys
import os
from enum import Enum
from typing import Optional, Dict, Any

PYTHON_PROJECT_PATH = "/home/kiro06 /dobot_workspace/src/python_project_file"
if os.path.exists(PYTHON_PROJECT_PATH):
    sys.path.append(PYTHON_PROJECT_PATH)

# Dobot 프로젝트 모듈 임포트 - ROS2 패키지 구조에 맞게 수정
# 방법 1: 상대 경로로 python_project_file 찾기
current_dir = os.path.dirname(os.path.abspath(__file__))
workspace_src = os.path.join(current_dir, '..', '..', '..')  # workspace/src까지
python_project_path = os.path.join(workspace_src, 'python_project_file')

# 경로가 존재하지 않으면 다른 위치들 시도
if not os.path.exists(python_project_path):
    # 방법 2: dobot_robot_manager 패키지 내부
    python_project_path = os.path.join(current_dir, '..', 'python_project_file')
    
if not os.path.exists(python_project_path):
    # 방법 3: 같은 레벨
    python_project_path = os.path.join(current_dir, '..', '..', 'python_project_file')

if os.path.exists(python_project_path):
    sys.path.append(python_project_path)
else:
    print(f"Warning: python_project_file not found. Searched paths:")
    print(f"  - {os.path.join(workspace_src, 'python_project_file')}")
    print(f"  - {os.path.join(current_dir, '..', 'python_project_file')}")
    print(f"  - {os.path.join(current_dir, '..', '..', 'python_project_file')}")

try:
    from robot_controller import RobotController
    from yolo_detector import YOLODetector
    from config import *
    DOBOT_AVAILABLE = True
except ImportError as e:
    print(f"Dobot 모듈을 불러올 수 없습니다: {e}")
    print("시뮬레이션 모드로 실행됩니다.")
    DOBOT_AVAILABLE = False


class CommunicationMode(Enum):
    TCP_ONLY = "tcp_only"
    ROS2_ONLY = "ros2_only"
    HYBRID = "hybrid"


class DobotROS2ClientNode(Node):
    """
    TCP와 ROS2 통신을 모두 지원하는 향상된 Dobot 클라이언트 노드
    """
    
    def __init__(self):
        super().__init__('dobot_ros2_client')
        
        # 파라미터 선언
        self.declare_parameters(
            namespace='',
            parameters=[
                ('robot_id', ''),
                ('server_host', '127.0.1.1'),
                ('server_port', 9988),
                ('communication_mode', 'hybrid'),  # tcp_only, ros2_only, hybrid
                ('enable_tcp', True),
                ('enable_ros2', True),
                ('heartbeat_interval', 10.0),
                ('reconnect_interval', 5.0),
                ('max_reconnect_attempts', 5),
            ]
        )
        
        # 파라미터 로드
        self.robot_id = self.get_parameter('robot_id').value
        self.server_host = self.get_parameter('server_host').value
        self.server_port = self.get_parameter('server_port').value
        self.communication_mode = CommunicationMode(self.get_parameter('communication_mode').value)
        self.enable_tcp = self.get_parameter('enable_tcp').value
        self.enable_ros2 = self.get_parameter('enable_ros2').value
        self.heartbeat_interval = self.get_parameter('heartbeat_interval').value
        self.reconnect_interval = self.get_parameter('reconnect_interval').value
        self.max_reconnect_attempts = self.get_parameter('max_reconnect_attempts').value
        
        # Robot ID 입력 (파라미터로 제공되지 않은 경우)
        if not self.robot_id:
            self.robot_id = input('Enter robot ID: ').strip()
            if not self.robot_id:
                self.get_logger().error('Robot ID를 입력해야 합니다.')
                rclpy.shutdown()
                return
        
        # 통신 모드 조정
        if self.communication_mode == CommunicationMode.TCP_ONLY:
            self.enable_ros2 = False
        elif self.communication_mode == CommunicationMode.ROS2_ONLY:
            self.enable_tcp = False
        
        # Dobot 컨트롤러 초기화
        self.robot_controller = None
        self.yolo_detector = None
        self.init_dobot_components()
        
        # 로봇 상태 관리
        self.is_busy = False
        self.current_task = None
        self.task_history = []
        self.last_position = None
        self.gripper_state = False
        self.connection_status = {
            'tcp': False,
            'ros2': True  # ROS2는 노드 생성 시 자동 연결
        }
        
        # TCP 연결 관리
        self.sock = None
        self.tcp_connected = False
        self.reconnect_attempts = 0
        
        # TCP 연결 초기화
        if self.enable_tcp:
            self.init_tcp_connection()
        
        # ROS2 인터페이스 설정
        if self.enable_ros2:
            self.setup_ros2_interface()
        
        # 백그라운드 스레드 시작
        self.start_background_threads()
        
        self.get_logger().info(f'=== Dobot ROS2 Client [{self.robot_id}] Started ===')
        self.get_logger().info(f'Communication Mode: {self.communication_mode.value}')
        self.get_logger().info(f'TCP: {"Enabled" if self.enable_tcp else "Disabled"}')
        self.get_logger().info(f'ROS2: {"Enabled" if self.enable_ros2 else "Disabled"}')
        self.get_logger().info(f'Dobot Available: {DOBOT_AVAILABLE}')
    
    def init_dobot_components(self):
        """Dobot 컴포넌트 초기화"""
        if DOBOT_AVAILABLE:
            try:
                self.robot_controller = RobotController()
                self.yolo_detector = YOLODetector()
                self.get_logger().info('Dobot 컴포넌트가 성공적으로 초기화되었습니다.')
            except Exception as e:
                self.get_logger().error(f'Dobot 초기화 실패: {e}')
                self.get_logger().info('시뮬레이션 모드로 전환합니다.')
                self.robot_controller = None
                self.yolo_detector = None
    
    def init_tcp_connection(self):
        """TCP 연결 초기화"""
        if not self.enable_tcp:
            return
        
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(10.0)  # 10초 타임아웃
            self.sock.connect((self.server_host, self.server_port))
            
            # 최초 ID 전송
            self.sock.sendall(self.robot_id.encode('utf-8'))
            
            self.tcp_connected = True
            self.connection_status['tcp'] = True
            self.reconnect_attempts = 0
            
            self.get_logger().info(f'TCP 연결 성공: {self.server_host}:{self.server_port}')
            
            # 초기 상태 전송
            self.send_status_update()
            
        except Exception as e:
            self.get_logger().error(f'TCP 연결 실패: {e}')
            self.tcp_connected = False
            self.connection_status['tcp'] = False
            if self.sock:
                self.sock.close()
                self.sock = None
    
    def setup_ros2_interface(self):
        """ROS2 인터페이스 설정"""
        if not self.enable_ros2:
            return
        
        # 개별 로봇 토픽들
        robot_prefix = f'/robot_{self.robot_id}'
        
        # 퍼블리셔들
        self.status_pub = self.create_publisher(
            String, f'{robot_prefix}/status', 10
        )
        self.result_pub = self.create_publisher(
            String, f'{robot_prefix}/result', 10
        )
        self.position_pub = self.create_publisher(
            Pose, f'{robot_prefix}/position', 10
        )
        
        # 공통 시스템 토픽들에도 발행
        self.system_status_pub = self.create_publisher(
            String, '/robot_status', 10
        )
        self.system_result_pub = self.create_publisher(
            String, '/robot_result', 10
        )
        
        # 구독자들
        self.command_sub = self.create_subscription(
            String, f'{robot_prefix}/command', self.ros2_command_callback, 10
        )
        
        # 공통 명령 토픽도 구독 (브로드캐스트용)
        self.broadcast_command_sub = self.create_subscription(
            String, '/robot_command', self.ros2_broadcast_command_callback, 10
        )
        
        # 긴급정지 구독
        self.emergency_stop_sub = self.create_subscription(
            Bool, '/emergency_stop', self.emergency_stop_callback, 10
        )
        
        # 주기적 상태 발행 타이머
        self.create_timer(self.heartbeat_interval, self.periodic_status_update)
        
        self.get_logger().info('ROS2 인터페이스가 설정되었습니다.')
    
    def start_background_threads(self):
        """백그라운드 스레드들 시작"""
        if self.enable_tcp:
            # TCP 수신 스레드
            threading.Thread(target=self.tcp_receive_loop, daemon=True).start()
            # TCP 재연결 스레드
            threading.Thread(target=self.tcp_reconnect_loop, daemon=True).start()
        
        # 연결 상태 모니터링 스레드
        threading.Thread(target=self.connection_monitor, daemon=True).start()
    
    def ros2_command_callback(self, msg: String):
        """개별 로봇 명령 수신"""
        try:
            command = json.loads(msg.data)
            self.get_logger().info(f'ROS2 개별 명령 수신: {command.get("task_type", "unknown")}')
            self.execute_command(command)
        except json.JSONDecodeError as e:
            self.get_logger().error(f'잘못된 ROS2 명령 JSON: {e}')
    
    def ros2_broadcast_command_callback(self, msg: String):
        """브로드캐스트 명령 수신"""
        try:
            command = json.loads(msg.data)
            target_robot_id = command.get('target_robot_id')
            
            # 이 로봇에게 보내진 명령인지 확인
            if target_robot_id == self.robot_id or target_robot_id == 'all':
                self.get_logger().info(f'ROS2 브로드캐스트 명령 수신: {command.get("task_type", "unknown")}')
                self.execute_command(command)
        except json.JSONDecodeError as e:
            self.get_logger().error(f'잘못된 브로드캐스트 명령 JSON: {e}')
    
    def emergency_stop_callback(self, msg: Bool):
        """긴급정지 처리"""
        if msg.data:
            self.get_logger().warning('긴급정지 신호 수신!')
            self.emergency_stop()
    
    def execute_command(self, command):
        """명령 실행"""
        if self.is_busy:
            error_result = {
                'type': 'error',
                'message': f'로봇 {self.robot_id}이 현재 작업 중입니다.',
                'command': command
            }
            self.send_result(error_result)
            return
        
        self.is_busy = True
        self.current_task = command.get('task_type', 'unknown')
        
        # 작업 실행을 별도 스레드에서 수행
        threading.Thread(target=self._execute_task_thread, args=(command,), daemon=True).start()
    
    def _execute_task_thread(self, command):
        """작업 실행 스레드"""
        try:
            task_type = command.get('task_type')
            self.get_logger().info(f'작업 실행 시작: {task_type}')
            
            if task_type == 'move_to_position':
                result = self.move_to_position(command)
            elif task_type == 'pickup_furniture':
                result = self.pickup_furniture(command)
            elif task_type == 'detect_objects':
                result = self.detect_objects(command)
            elif task_type == 'gripper_control':
                result = self.control_gripper(command)
            elif task_type == 'home_position':
                result = self.move_to_home()
            elif task_type == 'get_position':
                result = self.get_current_position()
            elif task_type == 'emergency_stop':
                result = self.emergency_stop()
            else:
                result = {
                    'type': 'error',
                    'message': f'알 수 없는 작업 타입: {task_type}',
                    'task_type': task_type
                }
            
            self.send_result(result)
            self.get_logger().info(f'작업 완료: {task_type} - {result.get("type", "unknown")}')
            
        except Exception as e:
            self.get_logger().error(f'작업 실행 중 오류: {e}')
            error_result = {
                'type': 'error',
                'message': f'작업 실행 중 오류 발생: {str(e)}',
                'task_type': command.get('task_type', 'unknown')
            }
            self.send_result(error_result)
        finally:
            self.is_busy = False
            self.current_task = None
            self.send_status_update()
    
    def move_to_position(self, command):
        """지정된 위치로 이동"""
        if not self.robot_controller:
            return {
                'type': 'simulation',
                'message': f'시뮬레이션: {self.robot_id} 위치 이동 완료',
                'robot_id': self.robot_id
            }
        
        try:
            x = float(command.get('x', 0))
            y = float(command.get('y', 0))
            z = float(command.get('z', 0))
            r = float(command.get('r', 0))
            
            self.robot_controller.move_to_position(x, y, z, r)
            
            # 위치 정보 업데이트
            self.last_position = {'x': x, 'y': y, 'z': z, 'r': r}
            
            # ROS2 위치 발행
            if self.enable_ros2:
                self.publish_position_update()
            
            return {
                'type': 'success',
                'message': f'위치 이동 완료: ({x}, {y}, {z}, {r})',
                'position': self.last_position,
                'robot_id': self.robot_id
            }
        except Exception as e:
            return {
                'type': 'error',
                'message': f'위치 이동 실패: {str(e)}',
                'robot_id': self.robot_id
            }
    
    def pickup_furniture(self, command):
        """가구 픽업 작업"""
        if not self.robot_controller:
            furniture_type = command.get('furniture_type', 'unknown')
            return {
                'type': 'simulation',
                'message': f'시뮬레이션: {self.robot_id} {furniture_type} 픽업 완료',
                'furniture_type': furniture_type,
                'robot_id': self.robot_id
            }
        
        try:
            furniture_type = command.get('furniture_type', 'chair')
            
            # Dobot 프로젝트의 가구 픽업 로직 사용
            if hasattr(self.robot_controller, 'pickup_furniture'):
                success = self.robot_controller.pickup_furniture(furniture_type)
            else:
                # 기본 픽업 시퀀스
                success = self._execute_pickup_sequence(furniture_type)
            
            if success:
                return {
                    'type': 'success',
                    'message': f'{furniture_type} 픽업 완료',
                    'furniture_type': furniture_type,
                    'robot_id': self.robot_id
                }
            else:
                return {
                    'type': 'error',
                    'message': f'{furniture_type} 픽업 실패',
                    'furniture_type': furniture_type,
                    'robot_id': self.robot_id
                }
        except Exception as e:
            return {
                'type': 'error',
                'message': f'픽업 작업 실패: {str(e)}',
                'robot_id': self.robot_id
            }
    
    def _execute_pickup_sequence(self, furniture_type):
        """기본 픽업 시퀀스"""
        try:
            # 가구별 기본 위치
            positions = {
                'chair': [200, 100, -50, 0],
                'sofa': [250, 0, -30, 0],
                'desk': [300, -100, -40, 0],
                'bed': [350, 50, -20, 0]
            }
            
            target_pos = positions.get(furniture_type, [200, 0, -50, 0])
            
            # 1. 안전 위치로 이동
            self.robot_controller.move_to_position(
                target_pos[0], target_pos[1], target_pos[2] + 50, target_pos[3]
            )
            time.sleep(1)
            
            # 2. 그리퍼 열기
            self.robot_controller.set_gripper(False)
            self.gripper_state = False
            time.sleep(1)
            
            # 3. 목표 위치로 하강
            self.robot_controller.move_to_position(
                target_pos[0], target_pos[1], target_pos[2], target_pos[3]
            )
            time.sleep(1)
            
            # 4. 그리퍼 닫기
            self.robot_controller.set_gripper(True)
            self.gripper_state = True
            time.sleep(1.5)
            
            # 5. 안전 위치로 상승
            self.robot_controller.move_to_position(
                target_pos[0], target_pos[1], target_pos[2] + 50, target_pos[3]
            )
            time.sleep(1)
            
            # 6. 홈 위치로 이동
            self.robot_controller.move_to_home()
            
            # 위치 정보 업데이트
            self.last_position = {'x': 0, 'y': 0, 'z': 0, 'r': 0}  # 홈 위치
            
            return True
        except Exception as e:
            self.get_logger().error(f'Pickup sequence failed: {e}')
            return False
    
    def detect_objects(self, command):
        """YOLO를 사용한 객체 인식"""
        if not self.yolo_detector:
            return {
                'type': 'simulation',
                'message': f'시뮬레이션: {self.robot_id} 객체 인식 완료',
                'detected_objects': ['chair', 'table'],
                'robot_id': self.robot_id
            }
        
        try:
            # YOLO 객체 인식 실행
            results = self.yolo_detector.detect_objects()
            
            return {
                'type': 'success',
                'message': '객체 인식 완료',
                'detected_objects': results,
                'robot_id': self.robot_id
            }
        except Exception as e:
            return {
                'type': 'error',
                'message': f'객체 인식 실패: {str(e)}',
                'robot_id': self.robot_id
            }
    
    def control_gripper(self, command):
        """그리퍼 제어"""
        if not self.robot_controller:
            state = command.get('state', True)
            self.gripper_state = state
            return {
                'type': 'simulation',
                'message': f'시뮬레이션: {self.robot_id} 그리퍼 {"닫기" if state else "열기"} 완료',
                'gripper_state': state,
                'robot_id': self.robot_id
            }
        
        try:
            state = command.get('state', True)  # True: 닫기, False: 열기
            self.robot_controller.set_gripper(state)
            self.gripper_state = state
            
            return {
                'type': 'success',
                'message': f'그리퍼 {"닫기" if state else "열기"} 완료',
                'gripper_state': state,
                'robot_id': self.robot_id
            }
        except Exception as e:
            return {
                'type': 'error',
                'message': f'그리퍼 제어 실패: {str(e)}',
                'robot_id': self.robot_id
            }
    
    def move_to_home(self):
        """홈 위치로 이동"""
        if not self.robot_controller:
            self.last_position = {'x': 0, 'y': 0, 'z': 0, 'r': 0}
            return {
                'type': 'simulation',
                'message': f'시뮬레이션: {self.robot_id} 홈 위치 이동 완료',
                'robot_id': self.robot_id
            }
        
        try:
            self.robot_controller.move_to_home()
            self.last_position = {'x': 0, 'y': 0, 'z': 0, 'r': 0}  # 홈 위치
            
            # ROS2 위치 발행
            if self.enable_ros2:
                self.publish_position_update()
            
            return {
                'type': 'success',
                'message': '홈 위치 이동 완료',
                'position': self.last_position,
                'robot_id': self.robot_id
            }
        except Exception as e:
            return {
                'type': 'error',
                'message': f'홈 위치 이동 실패: {str(e)}',
                'robot_id': self.robot_id
            }
    
    def get_current_position(self):
        """현재 위치 조회"""
        if not self.robot_controller:
            return {
                'type': 'simulation',
                'message': f'시뮬레이션: {self.robot_id} 현재 위치',
                'position': self.last_position or {'x': 0, 'y': 0, 'z': 0, 'r': 0},
                'robot_id': self.robot_id
            }
        
        try:
            # 실제 로봇에서 위치 정보 가져오기
            if hasattr(self.robot_controller, 'get_current_position'):
                position = self.robot_controller.get_current_position()
                self.last_position = position
            else:
                position = self.last_position or {'x': 0, 'y': 0, 'z': 0, 'r': 0}
            
            return {
                'type': 'success',
                'message': '현재 위치 조회 완료',
                'position': position,
                'robot_id': self.robot_id
            }
        except Exception as e:
            return {
                'type': 'error',
                'message': f'위치 조회 실패: {str(e)}',
                'robot_id': self.robot_id
            }
    
    def emergency_stop(self):
        """긴급 정지"""
        try:
            if self.robot_controller and hasattr(self.robot_controller, 'emergency_stop'):
                self.robot_controller.emergency_stop()
            
            # 현재 작업 중단
            self.is_busy = False
            self.current_task = None
            
            self.get_logger().warning(f'긴급 정지 실행: {self.robot_id}')
            
            return {
                'type': 'success',
                'message': '긴급 정지 완료',
                'robot_id': self.robot_id
            }
        except Exception as e:
            return {
                'type': 'error',
                'message': f'긴급 정지 실패: {str(e)}',
                'robot_id': self.robot_id
            }
    
    def send_status_update(self):
        """상태 업데이트 전송"""
        status = {
            'robot_id': self.robot_id,
            'type': 'status',
            'is_busy': self.is_busy,
            'current_task': self.current_task,
            'dobot_available': DOBOT_AVAILABLE and self.robot_controller is not None,
            'yolo_available': DOBOT_AVAILABLE and self.yolo_detector is not None,
            'position': self.last_position,
            'gripper_state': self.gripper_state,
            'connection_status': self.connection_status,
            'timestamp': time.time()
        }
        
        # TCP로 전송
        if self.enable_tcp and self.tcp_connected:
            try:
                self.sock.sendall(json.dumps(status).encode('utf-8'))
            except Exception as e:
                self.get_logger().error(f'TCP 상태 전송 실패: {e}')
                self.tcp_connected = False
        
        # ROS2로 전송
        if self.enable_ros2:
            try:
                status_msg = String()
                status_msg.data = json.dumps(status)
                
                # 개별 로봇 토픽
                self.status_pub.publish(status_msg)
                
                # 시스템 공통 토픽
                self.system_status_pub.publish(status_msg)
                
            except Exception as e:
                self.get_logger().error(f'ROS2 상태 발행 실패: {e}')
    
    def send_result(self, result):
        """작업 결과 전송"""
        result_data = {
            'robot_id': self.robot_id,
            'type': 'result',
            'result': result,
            'timestamp': time.time()
        }
        
        # TCP로 전송
        if self.enable_tcp and self.tcp_connected:
            try:
                self.sock.sendall(json.dumps(result_data).encode('utf-8'))
            except Exception as e:
                self.get_logger().error(f'TCP 결과 전송 실패: {e}')
                self.tcp_connected = False
        
        # ROS2로 전송
        if self.enable_ros2:
            try:
                result_msg = String()
                result_msg.data = json.dumps(result_data)
                
                # 개별 로봇 토픽
                self.result_pub.publish(result_msg)
                
                # 시스템 공통 토픽
                self.system_result_pub.publish(result_msg)
                
            except Exception as e:
                self.get_logger().error(f'ROS2 결과 발행 실패: {e}')
        
        # 작업 이력에 추가
        self.task_history.append({
            'timestamp': time.time(),
            'result': result
        })
        
        # 이력 크기 제한 (최근 100개만 유지)
        if len(self.task_history) > 100:
            self.task_history = self.task_history[-100:]
    
    def publish_position_update(self):
        """위치 정보 ROS2 발행"""
        if not self.enable_ros2 or not self.last_position:
            return
        
        try:
            pose_msg = Pose()
            pose_msg.position.x = float(self.last_position.get('x', 0))
            pose_msg.position.y = float(self.last_position.get('y', 0))
            pose_msg.position.z = float(self.last_position.get('z', 0))
            # 회전값은 쿼터니언으로 변환해야 하지만 간단히 처리
            pose_msg.orientation.w = 1.0
            
            self.position_pub.publish(pose_msg)
        except Exception as e:
            self.get_logger().error(f'위치 발행 실패: {e}')
    
    def periodic_status_update(self):
        """주기적 상태 업데이트"""
        self.send_status_update()
    
    def tcp_receive_loop(self):
        """TCP 메시지 수신 루프"""
        while rclpy.ok():
            if not self.tcp_connected or not self.sock:
                time.sleep(1)
                continue
            
            try:
                data = self.sock.recv(4096)
                if not data:
                    self.get_logger().info('TCP 서버 연결 종료')
                    self.tcp_connected = False
                    break
                
                text = data.decode('utf-8').strip()
                self.get_logger().info(f'TCP 수신: {text}')
                
                try:
                    # JSON 명령 파싱 시도
                    command = json.loads(text)
                    if isinstance(command, dict):
                        self.execute_command(command)
                    else:
                        self.get_logger().warning(f'잘못된 명령 형식: {text}')
                except json.JSONDecodeError:
                    # JSON이 아닌 경우 일반 텍스트 메시지로 처리
                    self.get_logger().info(f'텍스트 메시지: {text}')
                
            except socket.timeout:
                continue
            except Exception as e:
                self.get_logger().error(f'TCP 수신 실패: {e}')
                self.tcp_connected = False
                break
    
    def tcp_reconnect_loop(self):
        """TCP 재연결 루프"""
        while rclpy.ok():
            if not self.enable_tcp:
                time.sleep(10)
                continue
            
            if not self.tcp_connected and self.reconnect_attempts < self.max_reconnect_attempts:
                self.get_logger().info(f'TCP 재연결 시도 {self.reconnect_attempts + 1}/{self.max_reconnect_attempts}')
                self.reconnect_attempts += 1
                self.init_tcp_connection()
                
                if not self.tcp_connected:
                    time.sleep(self.reconnect_interval)
            else:
                time.sleep(10)
    
    def connection_monitor(self):
        """연결 상태 모니터링"""
        while rclpy.ok():
            try:
                # TCP 연결 상태 확인
                if self.enable_tcp:
                    self.connection_status['tcp'] = self.tcp_connected
                
                # ROS2 연결 상태는 항상 True (노드가 실행 중이면)
                self.connection_status['ros2'] = self.enable_ros2
                
                # 전체 연결 상태 로그
                if self.enable_tcp and self.enable_ros2:
                    status = f"TCP: {'✓' if self.tcp_connected else '✗'}, ROS2: {'✓' if self.enable_ros2 else '✗'}"
                elif self.enable_tcp:
                    status = f"TCP: {'✓' if self.tcp_connected else '✗'}"
                elif self.enable_ros2:
                    status = f"ROS2: ✓"
                else:
                    status = "연결 없음"
                
                self.get_logger().debug(f'연결 상태: {status}')
                
                time.sleep(30)  # 30초마다 확인
                
            except Exception as e:
                self.get_logger().error(f'연결 모니터링 오류: {e}')
                time.sleep(60)
    
    def destroy_node(self):
        """노드 종료 시 정리"""
        try:
            # 로봇을 안전 위치로 이동
            if self.robot_controller and not self.is_busy:
                try:
                    self.robot_controller.move_to_home()
                    if hasattr(self.robot_controller, 'disconnect'):
                        self.robot_controller.disconnect()
                except:
                    pass
            
            # TCP 연결 종료
            if self.sock:
                try:
                    self.sock.close()
                except:
                    pass
        except:
            pass
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = DobotROS2ClientNode()
        
        print("\n" + "="*70)
        print(f"Dobot ROS2 클라이언트 [{node.robot_id}]가 시작되었습니다!")
        print(f"통신 모드: {node.communication_mode.value}")
        print("지원되는 작업:")
        print("- move_to_position: 특정 위치로 이동")
        print("- pickup_furniture: 가구 픽업")
        print("- detect_objects: 객체 인식")
        print("- gripper_control: 그리퍼 제어")
        print("- home_position: 홈 위치로 이동")
        print("- get_position: 현재 위치 조회")
        print("- emergency_stop: 긴급 정지")
        print("="*70)
        
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print("\n키보드 인터럽트로 종료합니다.")
    except Exception as e:
        print(f"오류 발생: {e}")
    finally:
        try:
            node.destroy_node()
        except:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()