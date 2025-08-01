#!/usr/bin/env python3
"""
Dobot Multi-Robot Management Server Node
TCP와 ROS2 통신을 모두 지원하는 하이브리드 서버
"""

import socket
import threading
import json
import time
from datetime import datetime
from enum import Enum
from dataclasses import dataclass, field
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String, Bool, Int32
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import JointState

# 커스텀 메시지 (간단한 버전)
from std_msgs.msg import String as RobotCommand
from std_msgs.msg import String as RobotStatus
from std_msgs.msg import String as RobotResult


class TaskStatus(Enum):
    PENDING = "pending"
    IN_PROGRESS = "in_progress"
    COMPLETED = "completed"
    FAILED = "failed"
    CANCELLED = "cancelled"


class CommunicationType(Enum):
    TCP = "tcp"
    ROS2 = "ros2"
    HYBRID = "hybrid"


@dataclass
class Robot:
    id: str
    conn: Optional[socket.socket] = None
    comm_type: CommunicationType = CommunicationType.TCP
    is_busy: bool = False
    current_task: Optional[str] = None
    dobot_available: bool = False
    yolo_available: bool = False
    last_seen: float = field(default_factory=time.time)
    task_history: List[dict] = field(default_factory=list)
    position: Optional[dict] = None
    gripper_state: bool = False
    
    def to_dict(self):
        """ROS2 메시지 전송을 위한 딕셔너리 변환"""
        return {
            'id': self.id,
            'comm_type': self.comm_type.value,
            'is_busy': self.is_busy,
            'current_task': self.current_task,
            'dobot_available': self.dobot_available,
            'yolo_available': self.yolo_available,
            'last_seen': self.last_seen,
            'position': self.position,
            'gripper_state': self.gripper_state
        }


@dataclass
class Task:
    id: str
    task_type: str
    parameters: dict
    robot_id: Optional[str] = None
    status: TaskStatus = TaskStatus.PENDING
    created_at: float = field(default_factory=time.time)
    completed_at: Optional[float] = None
    result: Optional[dict] = None
    priority: int = 0
    
    def to_dict(self):
        """ROS2 메시지 전송을 위한 딕셔너리 변환"""
        return {
            'id': self.id,
            'task_type': self.task_type,
            'parameters': self.parameters,
            'robot_id': self.robot_id,
            'status': self.status.value,
            'created_at': self.created_at,
            'completed_at': self.completed_at,
            'result': self.result,
            'priority': self.priority
        }


class DobotMultiRobotServerNode(Node):
    """
    TCP와 ROS2 통신을 모두 지원하는 다중 로봇 관리 서버 노드
    """
    
    def __init__(self):
        super().__init__('dobot_multi_robot_server')
        
        # 파라미터 선언
        self.declare_parameters(
            namespace='',
            parameters=[
                ('tcp_host', '127.0.1.1'),
                ('tcp_port', 9980),
                ('enable_tcp', True),
                ('enable_ros2', True),
                ('max_robots', 10),
                ('task_timeout', 300.0),  # 5분
                ('heartbeat_interval', 10.0),  # 10초
                ('cleanup_interval', 60.0),  # 1분
            ]
        )
        
        # 파라미터 로드
        self.tcp_host = self.get_parameter('tcp_host').value
        self.tcp_port = self.get_parameter('tcp_port').value
        self.enable_tcp = self.get_parameter('enable_tcp').value
        self.enable_ros2 = self.get_parameter('enable_ros2').value
        self.max_robots = self.get_parameter('max_robots').value
        self.task_timeout = self.get_parameter('task_timeout').value
        self.heartbeat_interval = self.get_parameter('heartbeat_interval').value
        self.cleanup_interval = self.get_parameter('cleanup_interval').value
        
        # 데이터 구조 초기화
        self.robots: Dict[str, Robot] = {}
        self.tasks: Dict[str, Task] = {}
        self.task_queue: List[str] = []
        self.lock = threading.Lock()
        self.task_counter = 0
        
        # TCP 서버 초기화
        self.tcp_server_socket = None
        if self.enable_tcp:
            self.setup_tcp_server()
        
        # ROS2 퍼블리셔 및 구독자 설정
        if self.enable_ros2:
            self.setup_ros2_interface()
        
        # 백그라운드 스레드 시작
        self.start_background_threads()
        
        self.get_logger().info('=== Dobot Multi-Robot Server Node Started ===')
        self.get_logger().info(f'TCP Server: {"Enabled" if self.enable_tcp else "Disabled"}')
        self.get_logger().info(f'ROS2 Interface: {"Enabled" if self.enable_ros2 else "Disabled"}')
        if self.enable_tcp:
            self.get_logger().info(f'TCP Server listening on {self.tcp_host}:{self.tcp_port}')
    
    def setup_tcp_server(self):
        """TCP 서버 설정"""
        try:
            self.tcp_server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.tcp_server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.tcp_server_socket.bind((self.tcp_host, self.tcp_port))
            self.tcp_server_socket.listen(self.max_robots)
            self.get_logger().info(f'TCP server initialized on {self.tcp_host}:{self.tcp_port}')
        except Exception as e:
            self.get_logger().error(f'TCP server setup failed: {e}')
            self.enable_tcp = False
    
    def setup_ros2_interface(self):
        """ROS2 인터페이스 설정"""
        # 퍼블리셔
        self.robot_status_pub = self.create_publisher(
            String, '/robot_status', 10
        )
        self.task_status_pub = self.create_publisher(
            String, '/task_status', 10
        )
        self.system_status_pub = self.create_publisher(
            String, '/system_status', 10
        )
        
        # 구독자
        self.robot_command_sub = self.create_subscription(
            String, '/robot_command', self.ros2_command_callback, 10
        )
        self.task_request_sub = self.create_subscription(
            String, '/task_request', self.ros2_task_request_callback, 10
        )
        self.emergency_stop_sub = self.create_subscription(
            Bool, '/emergency_stop', self.emergency_stop_callback, 10
        )
        
        # 서비스 (나중에 추가 예정)
        # self.assign_task_service = self.create_service(...)
        
        # 주기적 상태 발행 타이머
        self.create_timer(self.heartbeat_interval, self.publish_system_status)
    
    def start_background_threads(self):
        """백그라운드 스레드들 시작"""
        if self.enable_tcp:
            # TCP 클라이언트 수락 스레드
            threading.Thread(target=self.tcp_accept_loop, daemon=True).start()
        
        # 작업 분배 스레드
        threading.Thread(target=self.task_distributor, daemon=True).start()
        
        # 로봇 모니터링 스레드
        threading.Thread(target=self.robot_monitor, daemon=True).start()
        
        # 정리 스레드
        threading.Thread(target=self.cleanup_manager, daemon=True).start()
    
    def tcp_accept_loop(self):
        """TCP 클라이언트 연결 수락 루프"""
        while rclpy.ok() and self.tcp_server_socket:
            try:
                conn, addr = self.tcp_server_socket.accept()
                threading.Thread(
                    target=self.handle_tcp_client, 
                    args=(conn, addr), 
                    daemon=True
                ).start()
            except Exception as e:
                self.get_logger().error(f'TCP accept error: {e}')
                if not rclpy.ok():
                    break
                time.sleep(1)
    
    def handle_tcp_client(self, conn, addr):
        """TCP 클라이언트 연결 처리"""
        robot_id = None
        try:
            # 최초 메시지로 클라이언트 ID 수신
            robot_id = conn.recv(1024).decode('utf-8').strip()
            if not robot_id:
                return
            
            # 로봇 등록
            with self.lock:
                if len(self.robots) >= self.max_robots:
                    self.get_logger().warning(f'Max robots ({self.max_robots}) reached. Rejecting {robot_id}')
                    conn.close()
                    return
                
                self.robots[robot_id] = Robot(
                    id=robot_id,
                    conn=conn,
                    comm_type=CommunicationType.TCP,
                    last_seen=time.time()
                )
            
            self.get_logger().info(f'[TCP 접속] 로봇 {robot_id} @ {addr}')
            self.publish_robot_status_update(robot_id)
            
            # 메시지 수신 루프
            while rclpy.ok():
                data = conn.recv(4096)
                if not data:
                    break
                
                try:
                    message = json.loads(data.decode('utf-8'))
                    self.process_robot_message(robot_id, message, CommunicationType.TCP)
                except json.JSONDecodeError:
                    # JSON이 아닌 일반 텍스트
                    text = data.decode('utf-8').strip()
                    self.get_logger().info(f'[TCP 수신 {robot_id}] {text}')
                
        except Exception as e:
            self.get_logger().error(f'[TCP 에러] {robot_id}: {e}')
        finally:
            # 연결 종료 시 로봇 제거
            if robot_id:
                with self.lock:
                    self.robots.pop(robot_id, None)
                self.get_logger().info(f'[TCP 해제] {robot_id}')
                self.publish_robot_status_update(robot_id, disconnected=True)
            conn.close()
    
    def ros2_command_callback(self, msg: String):
        """ROS2 명령 메시지 처리"""
        try:
            command = json.loads(msg.data)
            robot_id = command.get('robot_id')
            
            if robot_id and robot_id in self.robots:
                self.process_robot_command(robot_id, command, CommunicationType.ROS2)
            else:
                self.get_logger().warning(f'Unknown robot_id in ROS2 command: {robot_id}')
                
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Invalid ROS2 command JSON: {e}')
    
    def ros2_task_request_callback(self, msg: String):
        """ROS2 작업 요청 처리"""
        try:
            task_request = json.loads(msg.data)
            task_type = task_request.get('task_type')
            parameters = task_request.get('parameters', {})
            priority = task_request.get('priority', 0)
            
            task_id = self.add_task(task_type, parameters, priority)
            self.get_logger().info(f'ROS2 task added: {task_id}')
            
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Invalid ROS2 task request JSON: {e}')
    
    def emergency_stop_callback(self, msg: Bool):
        """긴급 정지 처리"""
        if msg.data:
            self.get_logger().warning('EMERGENCY STOP ACTIVATED!')
            self.emergency_stop_all_robots()
    
    def process_robot_message(self, robot_id: str, message: dict, comm_type: CommunicationType):
        """로봇으로부터 받은 메시지 처리"""
        msg_type = message.get('type')
        
        if msg_type == 'status':
            self.update_robot_status(robot_id, message, comm_type)
        elif msg_type == 'result':
            self.process_task_result(robot_id, message, comm_type)
        else:
            self.get_logger().warning(f'Unknown message type from {robot_id}: {msg_type}')
    
    def process_robot_command(self, robot_id: str, command: dict, comm_type: CommunicationType):
        """로봇 명령 처리"""
        task_type = command.get('task_type')
        parameters = command.get('parameters', {})
        
        if task_type:
            task_id = self.add_task(task_type, parameters, target_robot_id=robot_id)
            self.get_logger().info(f'Direct command to {robot_id}: {task_type} (Task ID: {task_id})')
    
    def update_robot_status(self, robot_id: str, status_msg: dict, comm_type: CommunicationType):
        """로봇 상태 업데이트"""
        with self.lock:
            if robot_id in self.robots:
                robot = self.robots[robot_id]
                robot.is_busy = status_msg.get('is_busy', False)
                robot.current_task = status_msg.get('current_task')
                robot.dobot_available = status_msg.get('dobot_available', False)
                robot.yolo_available = status_msg.get('yolo_available', False)
                robot.last_seen = time.time()
                robot.comm_type = comm_type
                
                # 위치 정보 업데이트 (있는 경우)
                if 'position' in status_msg:
                    robot.position = status_msg['position']
                
                # 그리퍼 상태 업데이트 (있는 경우)
                if 'gripper_state' in status_msg:
                    robot.gripper_state = status_msg['gripper_state']
        
        self.get_logger().debug(f'[상태 업데이트] {robot_id}: {"작업중" if status_msg.get("is_busy") else "대기중"}')
        
        # ROS2로 상태 발행
        if self.enable_ros2:
            self.publish_robot_status_update(robot_id)
    
    def process_task_result(self, robot_id: str, result_msg: dict, comm_type: CommunicationType):
        """작업 결과 처리"""
        result = result_msg.get('result', {})
        
        with self.lock:
            robot = self.robots.get(robot_id)
            if robot and robot.current_task:
                # 작업 완료 처리
                for task_id, task in self.tasks.items():
                    if task.robot_id == robot_id and task.status == TaskStatus.IN_PROGRESS:
                        task.status = TaskStatus.COMPLETED if result.get('type') == 'success' else TaskStatus.FAILED
                        task.completed_at = time.time()
                        task.result = result
                        
                        # ROS2로 작업 결과 발행
                        if self.enable_ros2:
                            self.publish_task_status_update(task_id)
                        break
                
                # 로봇 상태 업데이트
                robot.is_busy = False
                robot.current_task = None
                robot.task_history.append({
                    'timestamp': time.time(),
                    'result': result
                })
        
        self.get_logger().info(f'[작업 완료] {robot_id}: {result.get("message", "알 수 없는 결과")}')
    
    def send_task_to_robot(self, robot_id: str, task: Task) -> bool:
        """로봇에게 작업 전송"""
        with self.lock:
            robot = self.robots.get(robot_id)
        
        if not robot:
            self.get_logger().error(f'Robot "{robot_id}" not found')
            return False
        
        command = {
            'task_id': task.id,
            'task_type': task.task_type,
            **task.parameters
        }
        
        try:
            if robot.comm_type == CommunicationType.TCP and robot.conn:
                # TCP로 전송
                robot.conn.sendall(json.dumps(command).encode('utf-8'))
            elif robot.comm_type == CommunicationType.ROS2:
                # ROS2 토픽으로 전송
                command_msg = String()
                command_msg.data = json.dumps(command)
                # 로봇별 개별 토픽으로 전송
                robot_topic = f'/robot_{robot_id}/command'
                # 동적으로 퍼블리셔 생성은 복잡하므로, 통합 토픽 사용
                # 실제로는 robot_id를 포함한 메시지로 전송
                command['target_robot_id'] = robot_id
                command_msg.data = json.dumps(command)
                if hasattr(self, 'robot_command_pub'):
                    self.robot_command_pub.publish(command_msg)
            
            # 작업 상태 업데이트
            task.robot_id = robot_id
            task.status = TaskStatus.IN_PROGRESS
            
            with self.lock:
                robot.is_busy = True
                robot.current_task = task.id
            
            self.get_logger().info(f'[작업 전송] {robot_id} → {task.task_type}')
            
            # ROS2로 작업 상태 발행
            if self.enable_ros2:
                self.publish_task_status_update(task.id)
            
            return True
            
        except Exception as e:
            self.get_logger().error(f'Task send failed to {robot_id}: {e}')
            return False
    
    def add_task(self, task_type: str, parameters: dict, priority: int = 0, target_robot_id: Optional[str] = None) -> str:
        """새 작업 추가"""
        self.task_counter += 1
        task_id = f"task_{self.task_counter:06d}"
        
        task = Task(
            id=task_id,
            task_type=task_type,
            parameters=parameters,
            priority=priority
        )
        
        with self.lock:
            self.tasks[task_id] = task
            if target_robot_id and target_robot_id in self.robots:
                # 특정 로봇에게 직접 할당
                task.robot_id = target_robot_id
                if self.send_task_to_robot(target_robot_id, task):
                    self.get_logger().info(f'Task {task_id} assigned directly to {target_robot_id}')
                else:
                    # 직접 할당 실패시 큐에 추가
                    task.robot_id = None
                    self.task_queue.append(task_id)
            else:
                # 큐에 추가하여 자동 분배
                self.task_queue.append(task_id)
                self.task_queue.sort(key=lambda tid: self.tasks[tid].priority, reverse=True)
        
        self.get_logger().info(f'[작업 추가] {task_type} (ID: {task_id}, Priority: {priority})')
        
        # ROS2로 작업 상태 발행
        if self.enable_ros2:
            self.publish_task_status_update(task_id)
        
        return task_id
    
    def task_distributor(self):
        """작업 분배 스레드"""
        while rclpy.ok():
            try:
                if self.task_queue:
                    with self.lock:
                        # 사용 가능한 로봇 찾기
                        available_robots = [
                            robot for robot in self.robots.values()
                            if not robot.is_busy and (time.time() - robot.last_seen) < 30
                        ]
                        
                        if available_robots and self.task_queue:
                            # 우선순위가 높은 작업 먼저
                            task_id = self.task_queue.pop(0)
                            task = self.tasks.get(task_id)
                            
                            if task:
                                # 가장 적합한 로봇 선택 (현재는 단순히 첫 번째)
                                selected_robot = available_robots[0]
                                
                                # 작업 전송
                                if self.send_task_to_robot(selected_robot.id, task):
                                    self.get_logger().info(f'[작업 분배] {task.task_type} → {selected_robot.id}')
                
                time.sleep(2)  # 2초마다 확인
                
            except Exception as e:
                self.get_logger().error(f'Task distributor error: {e}')
                time.sleep(5)
    
    def robot_monitor(self):
        """로봇 상태 모니터링"""
        while rclpy.ok():
            try:
                current_time = time.time()
                with self.lock:
                    disconnected_robots = []
                    for robot_id, robot in self.robots.items():
                        if current_time - robot.last_seen > 60:  # 60초 무응답
                            disconnected_robots.append(robot_id)
                    
                    # 연결이 끊긴 로봇 제거
                    for robot_id in disconnected_robots:
                        self.get_logger().warning(f'[타임아웃] {robot_id} 연결 끊김')
                        self.robots.pop(robot_id, None)
                        # ROS2로 연결 해제 알림
                        if self.enable_ros2:
                            self.publish_robot_status_update(robot_id, disconnected=True)
                
                time.sleep(30)  # 30초마다 확인
                
            except Exception as e:
                self.get_logger().error(f'Robot monitor error: {e}')
                time.sleep(60)
    
    def cleanup_manager(self):
        """정리 작업 관리"""
        while rclpy.ok():
            try:
                current_time = time.time()
                with self.lock:
                    # 완료된 오래된 작업 정리 (1시간 이상 된 것)
                    old_tasks = []
                    for task_id, task in self.tasks.items():
                        if (task.status in [TaskStatus.COMPLETED, TaskStatus.FAILED, TaskStatus.CANCELLED] 
                            and task.completed_at 
                            and current_time - task.completed_at > 3600):
                            old_tasks.append(task_id)
                    
                    for task_id in old_tasks:
                        self.tasks.pop(task_id, None)
                        self.get_logger().debug(f'Cleaned up old task: {task_id}')
                
                time.sleep(self.cleanup_interval)
                
            except Exception as e:
                self.get_logger().error(f'Cleanup manager error: {e}')
                time.sleep(300)  # 5분 후 재시도
    
    def publish_robot_status_update(self, robot_id: str, disconnected: bool = False):
        """로봇 상태 업데이트 발행"""
        if not self.enable_ros2:
            return
        
        try:
            if disconnected:
                status_data = {
                    'robot_id': robot_id,
                    'status': 'disconnected',
                    'timestamp': time.time()
                }
            else:
                with self.lock:
                    robot = self.robots.get(robot_id)
                    if robot:
                        status_data = robot.to_dict()
                        status_data['timestamp'] = time.time()
                    else:
                        return
            
            status_msg = String()
            status_msg.data = json.dumps(status_data)
            self.robot_status_pub.publish(status_msg)
            
        except Exception as e:
            self.get_logger().error(f'Failed to publish robot status: {e}')
    
    def publish_task_status_update(self, task_id: str):
        """작업 상태 업데이트 발행"""
        if not self.enable_ros2:
            return
        
        try:
            with self.lock:
                task = self.tasks.get(task_id)
                if task:
                    task_data = task.to_dict()
                    task_data['timestamp'] = time.time()
                    
                    task_msg = String()
                    task_msg.data = json.dumps(task_data)
                    self.task_status_pub.publish(task_msg)
        
        except Exception as e:
            self.get_logger().error(f'Failed to publish task status: {e}')
    
    def publish_system_status(self):
        """시스템 전체 상태 발행"""
        if not self.enable_ros2:
            return
        
        try:
            with self.lock:
                system_data = {
                    'timestamp': time.time(),
                    'total_robots': len(self.robots),
                    'active_robots': len([r for r in self.robots.values() if not r.is_busy]),
                    'busy_robots': len([r for r in self.robots.values() if r.is_busy]),
                    'total_tasks': len(self.tasks),
                    'pending_tasks': len(self.task_queue),
                    'active_tasks': len([t for t in self.tasks.values() if t.status == TaskStatus.IN_PROGRESS]),
                    'completed_tasks': len([t for t in self.tasks.values() if t.status == TaskStatus.COMPLETED]),
                    'tcp_enabled': self.enable_tcp,
                    'ros2_enabled': self.enable_ros2
                }
            
            system_msg = String()
            system_msg.data = json.dumps(system_data)
            self.system_status_pub.publish(system_msg)
            
        except Exception as e:
            self.get_logger().error(f'Failed to publish system status: {e}')
    
    def emergency_stop_all_robots(self):
        """모든 로봇 긴급 정지"""
        emergency_command = {
            'task_type': 'emergency_stop',
            'parameters': {}
        }
        
        with self.lock:
            for robot in self.robots.values():
                try:
                    if robot.comm_type == CommunicationType.TCP and robot.conn:
                        robot.conn.sendall(json.dumps(emergency_command).encode('utf-8'))
                    # ROS2 긴급정지는 별도 토픽으로 처리
                    
                    # 현재 작업 취소
                    if robot.current_task:
                        task = self.tasks.get(robot.current_task)
                        if task:
                            task.status = TaskStatus.CANCELLED
                            task.completed_at = time.time()
                    
                    robot.is_busy = False
                    robot.current_task = None
                    
                except Exception as e:
                    self.get_logger().error(f'Emergency stop failed for {robot.id}: {e}')
        
        self.get_logger().warning('Emergency stop sent to all robots')
    
    def get_robot_status_summary(self) -> dict:
        """로봇 상태 요약 반환"""
        with self.lock:
            return {
                'total_robots': len(self.robots),
                'active_robots': len([r for r in self.robots.values() if not r.is_busy]),
                'busy_robots': len([r for r in self.robots.values() if r.is_busy]),
                'robots': [robot.to_dict() for robot in self.robots.values()]
            }
    
    def get_task_status_summary(self) -> dict:
        """작업 상태 요약 반환"""
        with self.lock:
            return {
                'total_tasks': len(self.tasks),
                'pending_tasks': len(self.task_queue),
                'active_tasks': len([t for t in self.tasks.values() if t.status == TaskStatus.IN_PROGRESS]),
                'completed_tasks': len([t for t in self.tasks.values() if t.status == TaskStatus.COMPLETED]),
                'failed_tasks': len([t for t in self.tasks.values() if t.status == TaskStatus.FAILED]),
                'tasks': [task.to_dict() for task in list(self.tasks.values())[-20:]]  # 최근 20개
            }
    
    def destroy_node(self):
        """노드 종료 시 정리"""
        try:
            # TCP 서버 종료
            if self.tcp_server_socket:
                self.tcp_server_socket.close()
            
            # 모든 로봇에게 종료 알림
            with self.lock:
                for robot in self.robots.values():
                    if robot.conn:
                        try:
                            robot.conn.close()
                        except:
                            pass
        except:
            pass
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = DobotMultiRobotServerNode()
        
        # 인터랙티브 모드를 위한 별도 스레드 (선택사항)
        import sys
        if len(sys.argv) > 1 and sys.argv[1] == '--interactive':
            threading.Thread(target=interactive_mode, args=(node,), daemon=True).start()
        
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print("\n서버를 종료합니다.")
    except Exception as e:
        print(f"오류 발생: {e}")
    finally:
        try:
            node.destroy_node()
        except:
            pass
        rclpy.shutdown()


def interactive_mode(node):
    """인터랙티브 명령 모드 (선택사항)"""
    print("\n=== Dobot Multi-Robot Server Interactive Mode ===")
    print("명령어:")
    print("  status - 로봇 상태 표시")
    print("  tasks - 작업 상태 표시")
    print("  add_task <type> <params> - 작업 추가")
    print("  emergency - 긴급 정지")
    print("  quit - 종료\n")
    
    while rclpy.ok():
        try:
            line = input("서버 명령> ").strip()
            if not line:
                continue
            
            parts = line.split()
            command = parts[0].lower()
            
            if command == 'status':
                status = node.get_robot_status_summary()
                print(f"로봇 상태: {status}")
            
            elif command == 'tasks':
                status = node.get_task_status_summary()
                print(f"작업 상태: {status}")
            
            elif command == 'add_task' and len(parts) >= 2:
                task_type = parts[1]
                params = {}
                if len(parts) > 2:
                    try:
                        params = json.loads(' '.join(parts[2:]))
                    except:
                        params = {'data': ' '.join(parts[2:])}
                
                task_id = node.add_task(task_type, params)
                print(f"작업 추가됨: {task_id}")
            
            elif command == 'emergency':
                node.emergency_stop_all_robots()
                print("긴급 정지 실행됨")
            
            elif command == 'quit':
                break
            
            else:
                print("알 수 없는 명령어입니다.")
                
        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"명령 처리 오류: {e}")


if __name__ == '__main__':
    main()