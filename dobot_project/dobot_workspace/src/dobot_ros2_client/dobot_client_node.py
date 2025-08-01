#!/usr/bin/env python3
"""
ROS2 클라이언트 노드 - 브로드캐스트 메시지 수신 기능 포함
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point, Quaternion
from std_srvs.srv import SetBool, Trigger
from sensor_msgs.msg import JointState

import math
import requests
import json
import time
import threading
import socket

class DobotROS2Client(Node):
    """ROS2 클라이언트 노드 - TCP 클라이언트 기능 포함"""
    global i
    def __init__(self):
        super().__init__('dobot_ros2_client')
        i+=1
        # 파라미터 선언
        self.declare_parameter('server_url', 'http://localhost:8080')
        self.declare_parameter('tcp_server_host', '127.0.0.1')
        self.declare_parameter('tcp_server_port', 9988)
        self.declare_parameter('update_rate', 10.0)
        self.declare_parameter('client_name', 'ROS2_Client{i}')
        
        # 파라미터 가져오기
        self.server_url = self.get_parameter('server_url').value
        self.tcp_host = self.get_parameter('tcp_server_host').value
        self.tcp_port = self.get_parameter('tcp_server_port').value
        self.update_rate = self.get_parameter('update_rate').value
        self.client_name = self.get_parameter('client_name').value
        
        # TCP 클라이언트 설정
        self.tcp_socket = None
        self.tcp_connected = False
        self.tcp_running = False
        
        # Publishers
        self.pose_publisher = self.create_publisher(Pose, 'dobot/current_pose', 10)
        self.status_publisher = self.create_publisher(String, 'dobot/status', 10)
        self.joint_state_publisher = self.create_publisher(JointState, 'joint_states', 10)
        
        # 브로드캐스트 메시지 publisher
        self.broadcast_publisher = self.create_publisher(String, 'dobot/broadcast_messages', 10)
        self.server_message_publisher = self.create_publisher(String, 'dobot/server_messages', 10)
        
        # Subscribers
        self.target_pose_sub = self.create_subscription(
            Pose, 'dobot/target_pose', self.target_pose_callback, 10)
        
        # ROS2로 메시지 전송 subscriber
        self.send_message_sub = self.create_subscription(
            String, 'dobot/send_message', self.send_message_callback, 10)
        
        # Services
        self.setup_services()
        
        # 타이머
        self.status_timer = self.create_timer(1.0 / self.update_rate, self.status_timer_callback)
        
        self.get_logger().info(f'🤖 Dobot ROS2 클라이언트 시작됨')
        self.get_logger().info(f'📡 HTTP 서버: {self.server_url}')
        self.get_logger().info(f'🔌 TCP 서버: {self.tcp_host}:{self.tcp_port}')
        self.get_logger().info(f'👤 클라이언트 이름: {self.client_name}')
        
        # TCP 클라이언트 연결 시작
        self.connect_tcp_client()
    
    def connect_tcp_client(self):
        """Python 서버의 TCP 서버에 연결"""
        try:
            self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.tcp_socket.connect((self.tcp_host, self.tcp_port))
            
            # 서버로부터 연결 확인 메시지 수신
            welcome_data = self.tcp_socket.recv(1024).decode('utf-8').strip()
            self.get_logger().info(f"TCP 서버: {welcome_data}")
            
            # 이름 전송
            self.tcp_socket.sendall(self.client_name.encode('utf-8'))
            
            # 환영 메시지 수신
            response = self.tcp_socket.recv(1024).decode('utf-8').strip()
            self.get_logger().info(f"TCP 서버: {response}")
            
            if response.startswith("WELCOME:"):
                self.client_id = response.split(":", 1)[1]
                self.tcp_connected = True
                self.tcp_running = True
                
                # 메시지 수신 스레드 시작
                threading.Thread(target=self.receive_tcp_messages, daemon=True).start()
                
                self.get_logger().info(f'✅ TCP 서버에 연결됨 (ID: {self.client_id})')
                
                # 연결 성공 메시지 발행
                msg = String()
                msg.data = f"Connected to TCP server as {self.client_name}"
                self.server_message_publisher.publish(msg)
                
                return True
            else:
                self.get_logger().error(f'❌ TCP 연결 실패: {response}')
                return False
                
        except Exception as e:
            self.get_logger().error(f'❌ TCP 연결 실패: {e}')
            self.get_logger().info('💡 Python 서버의 클라이언트 포트(9988)가 활성화되어 있는지 확인하세요')
            return False
    
    def receive_tcp_messages(self):
        """TCP 서버로부터 메시지 수신"""
        while self.tcp_running:
            try:
                data = self.tcp_socket.recv(1024)
                if not data:
                    break
                
                messages = data.decode('utf-8').strip().split('\n')
                for message in messages:
                    if message:
                        try:
                            msg_data = json.loads(message)
                            self.handle_tcp_message(msg_data)
                        except json.JSONDecodeError:
                            # 일반 텍스트 메시지
                            self.get_logger().info(f"📨 TCP 서버: {message}")
                            msg = String()
                            msg.data = f"Server: {message}"
                            self.server_message_publisher.publish(msg)
                            
            except Exception as e:
                if self.tcp_running:
                    self.get_logger().error(f'❌ TCP 수신 오류: {e}')
                break
        
        self.tcp_connected = False
        self.get_logger().warn('📡 TCP 서버와의 연결이 끊어졌습니다.')
    
    def handle_tcp_message(self, msg_data):
        """TCP 서버로부터 받은 메시지 처리"""
        msg_type = msg_data.get('type', 'unknown')
        
        # ROS2 토픽으로 메시지 발행
        ros_msg = String()
        
        if msg_type == 'broadcast':
            # 브로드캐스트 메시지
            content = msg_data.get('content', '')
            from_user = msg_data.get('from', 'Unknown')
            
            self.get_logger().info(f'📢 [브로드캐스트] {from_user}: {content}')
            
            ros_msg.data = json.dumps({
                'type': 'broadcast',
                'from': from_user,
                'content': content,
                'timestamp': msg_data.get('timestamp', time.time())
            })
            self.broadcast_publisher.publish(ros_msg)
            
        elif msg_type == 'private_message':
            # 개인 메시지
            content = msg_data.get('content', '')
            from_user = msg_data.get('from', 'Unknown')
            
            self.get_logger().info(f'💬 [개인 메시지] {from_user}: {content}')
            
            ros_msg.data = json.dumps({
                'type': 'private',
                'from': from_user,
                'content': content,
                'timestamp': msg_data.get('timestamp', time.time())
            })
            self.server_message_publisher.publish(ros_msg)
            
        elif msg_type == 'new_connection':
            # 새 클라이언트 접속
            client_name = msg_data.get('client_name', 'Unknown')
            self.get_logger().info(f'👋 [{client_name}]님이 입장했습니다.')
            
            ros_msg.data = f"New client connected: {client_name}"
            self.server_message_publisher.publish(ros_msg)
            
        elif msg_type == 'disconnection':
            # 클라이언트 퇴장
            client_name = msg_data.get('client_name', 'Unknown')
            self.get_logger().info(f'👋 [{client_name}]님이 퇴장했습니다.')
            
            ros_msg.data = f"Client disconnected: {client_name}"
            self.server_message_publisher.publish(ros_msg)
            
        elif msg_type == 'dobot_response':
            # Dobot 명령 응답
            self.get_logger().info(f'🤖 Dobot 응답: {msg_data}')
            
            ros_msg.data = json.dumps(msg_data)
            self.server_message_publisher.publish(ros_msg)
            
        else:
            # 기타 메시지
            self.get_logger().info(f'📨 {msg_type}: {msg_data}')
            
            ros_msg.data = json.dumps(msg_data)
            self.server_message_publisher.publish(ros_msg)
    
    def send_message_callback(self, msg):
        """ROS2에서 TCP 서버로 메시지 전송"""
        if self.tcp_connected and self.tcp_socket:
            try:
                # 메시지 형식 확인
                try:
                    # JSON 형식인 경우
                    msg_data = json.loads(msg.data)
                except:
                    # 일반 텍스트인 경우
                    msg_data = {
                        "type": "text",
                        "content": msg.data,
                        "timestamp": time.time()
                    }
                
                message = json.dumps(msg_data)
                self.tcp_socket.sendall(message.encode('utf-8'))
                self.get_logger().info(f'📤 메시지 전송: {msg.data}')
                
            except Exception as e:
                self.get_logger().error(f'❌ 메시지 전송 실패: {e}')
        else:
            self.get_logger().warn('⚠️ TCP 서버에 연결되어 있지 않습니다.')
    
    def send_dobot_command_via_tcp(self, command):
        """TCP를 통해 Dobot 명령 전송"""
        if self.tcp_connected:
            msg_data = {
                "type": "dobot_command",
                "content": command,
                "timestamp": time.time()
            }
            
            try:
                self.tcp_socket.sendall(json.dumps(msg_data).encode('utf-8'))
                self.get_logger().info(f'🤖 Dobot 명령 전송 (TCP): {command}')
                return True
            except Exception as e:
                self.get_logger().error(f'❌ Dobot 명령 전송 실패: {e}')
                return False
        return False
    
    def setup_services(self):
        """서비스 설정"""
        self.move_service = self.create_service(
            SetBool, 'dobot/move_to_pose', self.move_service_callback)
        
        self.suction_service = self.create_service(
            SetBool, 'dobot/set_suction', self.suction_service_callback)
        
        self.gripper_service = self.create_service(
            SetBool, 'dobot/set_gripper', self.gripper_service_callback)
        
        self.emergency_stop_service = self.create_service(
            Trigger, 'dobot/emergency_stop', self.emergency_stop_callback)
        
        # TCP 연결 관련 서비스
        self.reconnect_tcp_service = self.create_service(
            Trigger, 'dobot/reconnect_tcp', self.reconnect_tcp_callback)
    
    def reconnect_tcp_callback(self, request, response):
        """TCP 재연결 서비스"""
        if self.tcp_connected:
            response.success = True
            response.message = "Already connected to TCP server"
        else:
            if self.connect_tcp_client():
                response.success = True
                response.message = f"Reconnected to TCP server as {self.client_name}"
            else:
                response.success = False
                response.message = "Failed to reconnect to TCP server"
        return response
    
    def target_pose_callback(self, msg: Pose):
        """목표 위치 명령 수신"""
        try:
            z_rotation = self.quaternion_to_euler_z(msg.orientation)
            command = f"MovJ({msg.position.x},{msg.position.y},{msg.position.z},{z_rotation})"
            
            # TCP와 HTTP 둘 다 시도
            tcp_sent = self.send_dobot_command_via_tcp(command)
            
            if not tcp_sent:
                # TCP 실패 시 HTTP로 전송
                data = {'command': command}
                response = requests.post(f"{self.server_url}/command", 
                                       json=data, timeout=5.0)
                
                if response.status_code == 200:
                    self.get_logger().info(f'🎯 이동 명령 전송 (HTTP): {command}')
                else:
                    self.get_logger().error(f'❌ 이동 명령 실패: {response.status_code}')
                    
        except Exception as e:
            self.get_logger().error(f'❌ 통신 오류: {e}')
    
    def quaternion_to_euler_z(self, q):
        """Quaternion을 Z축 오일러 각도로 변환"""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp) * 180.0 / math.pi
    
    def move_service_callback(self, request, response):
        """이동 서비스 콜백"""
        response.success = True
        response.message = "Move service called"
        return response
    
    def suction_service_callback(self, request, response):
        """석션 제어 서비스"""
        command = f"Suction({1 if request.data else 0})"
        
        if self.send_dobot_command_via_tcp(command):
            response.success = True
            response.message = f"석션 {'활성화' if request.data else '비활성화'} (TCP)"
        else:
            # HTTP 시도
            try:
                data = {'command': command}
                http_response = requests.post(f"{self.server_url}/command", 
                                            json=data, timeout=5.0)
                
                if http_response.status_code == 200:
                    response.success = True
                    response.message = f"석션 {'활성화' if request.data else '비활성화'} (HTTP)"
                else:
                    response.success = False
                    response.message = "석션 제어 실패"
            except Exception as e:
                response.success = False
                response.message = f"오류: {e}"
        
        return response
    
    def gripper_service_callback(self, request, response):
        """그리퍼 제어 서비스"""
        command = f"Gripper({1 if request.data else 0})"
        
        if self.send_dobot_command_via_tcp(command):
            response.success = True
            response.message = f"그리퍼 {'열림' if request.data else '닫힘'} (TCP)"
        else:
            # HTTP 시도
            try:
                data = {'command': command}
                http_response = requests.post(f"{self.server_url}/command", 
                                            json=data, timeout=5.0)
                
                if http_response.status_code == 200:
                    response.success = True
                    response.message = f"그리퍼 {'열림' if request.data else '닫힘'} (HTTP)"
                else:
                    response.success = False
                    response.message = "그리퍼 제어 실패"
            except Exception as e:
                response.success = False
                response.message = f"오류: {e}"
        
        return response
    
    def emergency_stop_callback(self, request, response):
        """비상 정지 서비스"""
        command = "EmergencyStop()"
        
        if self.send_dobot_command_via_tcp(command):
            response.success = True
            response.message = "비상 정지 실행됨 (TCP)"
        else:
            # HTTP 시도
            try:
                data = {'command': command}
                http_response = requests.post(f"{self.server_url}/command", 
                                            json=data, timeout=5.0)
                
                if http_response.status_code == 200:
                    response.success = True
                    response.message = "비상 정지 실행됨 (HTTP)"
                else:
                    response.success = False
                    response.message = "비상 정지 실패"
            except Exception as e:
                response.success = False
                response.message = f"오류: {e}"
        
        return response
    
    def status_timer_callback(self):
        """주기적 상태 확인"""
        try:
            # HTTP 상태 확인
            response = requests.get(f"{self.server_url}/status", timeout=2.0)
            
            if response.status_code == 200:
                status_data = response.json()
                
                # TCP 연결 상태 추가
                status_data['tcp_connected'] = self.tcp_connected
                status_data['tcp_client_id'] = getattr(self, 'client_id', 'Not connected')
                
                # 상태 메시지 발행
                status_msg = String()
                status_msg.data = json.dumps(status_data)
                self.status_publisher.publish(status_msg)
                
                # 현재 위치 발행
                position_response = requests.get(f"{self.server_url}/position", timeout=2.0)
                if position_response.status_code == 200:
                    position_data = position_response.json()
                    if position_data.get('status') == 'success' and position_data.get('position'):
                        pose = position_data['position']
                        if len(pose) >= 4:
                            pose_msg = Pose()
                            pose_msg.position = Point(x=float(pose[0]), y=float(pose[1]), z=float(pose[2]))
                            r_rad = float(pose[3]) * 3.14159 / 180.0
                            pose_msg.orientation = Quaternion(x=0.0, y=0.0, z=math.sin(r_rad/2), w=math.cos(r_rad/2))
                            self.pose_publisher.publish(pose_msg)
                
                # Joint State 발행
                joint_state = JointState()
                joint_state.header.stamp = self.get_clock().now().to_msg()
                joint_state.name = ['joint1', 'joint2', 'joint3', 'joint4']
                joint_state.position = [0.0, 0.0, 0.0, 0.0]
                self.joint_state_publisher.publish(joint_state)
                
        except requests.exceptions.RequestException:
            pass
        except Exception as e:
            self.get_logger().error(f'상태 확인 중 오류: {e}')
    
    def __del__(self):
        """소멸자 - TCP 연결 정리"""
        self.tcp_running = False
        if self.tcp_socket:
            try:
                self.tcp_socket.close()
            except:
                pass


def main(args=None):
    """메인 함수"""
    rclpy.init(args=args)
    
    try:
        dobot_client = DobotROS2Client()
        
        dobot_client.get_logger().info('🚀 ROS2 클라이언트가 실행 중입니다!')
        dobot_client.get_logger().info('📖 사용법:')
        dobot_client.get_logger().info('  📢 브로드캐스트 메시지 확인: ros2 topic echo /dobot/broadcast_messages')
        dobot_client.get_logger().info('  💬 서버 메시지 확인: ros2 topic echo /dobot/server_messages')
        dobot_client.get_logger().info('  📤 메시지 전송: ros2 topic pub /dobot/send_message std_msgs/String "{data: \'Hello from ROS2\'}"')
        dobot_client.get_logger().info('  🎯 이동 명령: ros2 topic pub /dobot/target_pose geometry_msgs/Pose "{position: {x: 200.0, y: 0.0, z: 100.0}}"')
        dobot_client.get_logger().info('  🔄 TCP 재연결: ros2 service call /dobot/reconnect_tcp std_srvs/Trigger')
        
        rclpy.spin(dobot_client)
        
    except KeyboardInterrupt:
        print('\n👋 사용자에 의해 종료됨')
    except Exception as e:
        print(f"❌ ROS2 클라이언트 오류: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if 'dobot_client' in locals():
            dobot_client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()