import socket
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
import sys
import os

# Dobot 프로젝트의 주요 모듈들을 임포트하기 위한 경로 추가
sys.path.append(os.path.join(os.path.dirname(__file__), 'python_project_file'))

try:
    from robot_controller import RobotController
    from yolo_detector import YOLODetector
    from config import *
    DOBOT_AVAILABLE = True
except ImportError as e:
    print(f"Dobot 모듈을 불러올 수 없습니다: {e}")
    print("시뮬레이션 모드로 실행됩니다.")
    DOBOT_AVAILABLE = False

class DobotROS2ClientNode(Node):
    """
    ROS2 노드로 동작하며, TCP 소켓으로 중앙 서버와 통신합니다.
    - Dobot 로봇팔 제어
    - YOLO 객체 인식
    - 서버로부터 작업 명령 수신 및 실행
    - 작업 결과를 서버로 전송
    """
    
    def __init__(self):
        super().__init__('dobot_ros2_client')
        
        # 런타임에 robot_id 입력
        robot_id = input('Enter robot ID: ').strip()
        if not robot_id:
            self.get_logger().error('Robot ID를 입력해야 합니다.')
            rclpy.shutdown()
            return
        
        self.robot_id = robot_id
        
        # 파라미터 선언 및 로드
        self.declare_parameter('server_host', '127.0.1.1')
        self.declare_parameter('server_port', 9988)
        host = self.get_parameter('server_host').get_parameter_value().string_value
        port = self.get_parameter('server_port').get_parameter_value().integer_value
        
        # Dobot 로봇 컨트롤러 초기화
        self.robot_controller = None
        self.yolo_detector = None
        
        if DOBOT_AVAILABLE:
            try:
                self.robot_controller = RobotController()
                self.yolo_detector = YOLODetector()
                self.get_logger().info('Dobot 로봇 컨트롤러가 초기화되었습니다.')
            except Exception as e:
                self.get_logger().error(f'Dobot 초기화 실패: {e}')
                self.get_logger().info('시뮬레이션 모드로 전환합니다.')
        
        # 로봇 상태 관리
        self.is_busy = False
        self.current_task = None
        self.task_history = []
        
        # 소켓 연결
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.sock.connect((host, port))
            self.get_logger().info(f'Socket connected to server at {host}:{port}')
            # 최초 ID 전송
            self.sock.sendall(robot_id.encode('utf-8'))
            
            # 로봇 상태 정보 전송
            self.send_status_update()
            
        except Exception as e:
            self.get_logger().error(f'Connection failed: {e}')
            rclpy.shutdown()
            return
        
        # ROS2 퍼블리셔 및 구독자 설정
        self.status_pub = self.create_publisher(String, f'robot_{robot_id}/status', 10)
        self.result_pub = self.create_publisher(String, f'robot_{robot_id}/result', 10)
        self.command_sub = self.create_subscription(
            String, f'robot_{robot_id}/command', self.command_callback, 10
        )
        
        # 백그라운드 수신 스레드 시작
        threading.Thread(target=self.receive_loop, daemon=True).start()
        
        # 상태 업데이트 타이머 (10초마다)
        self.create_timer(10.0, self.periodic_status_update)
        
        self.get_logger().info(f'=== Dobot ROS2 Client [{robot_id}] Started ===')
        self.get_logger().info('서버로부터 작업 명령을 대기 중입니다.')
    
    def send_status_update(self):
        """현재 로봇 상태를 서버로 전송"""
        status = {
            'robot_id': self.robot_id,
            'type': 'status',
            'is_busy': self.is_busy,
            'current_task': self.current_task,
            'dobot_available': DOBOT_AVAILABLE and self.robot_controller is not None,
            'yolo_available': DOBOT_AVAILABLE and self.yolo_detector is not None,
            'timestamp': time.time()
        }
        
        try:
            self.sock.sendall(json.dumps(status).encode('utf-8'))
            self.get_logger().debug(f'Status sent: {status}')
        except Exception as e:
            self.get_logger().error(f'Status send failed: {e}')
    
    def periodic_status_update(self):
        """주기적 상태 업데이트"""
        self.send_status_update()
    
    def command_callback(self, msg: String):
        """ROS2 토픽으로부터 명령 수신"""
        try:
            command = json.loads(msg.data)
            self.execute_command(command)
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Invalid command JSON: {e}')
    
    def execute_command(self, command):
        """받은 명령을 실행"""
        if self.is_busy:
            self.send_result({
                'type': 'error',
                'message': '로봇이 현재 작업 중입니다.',
                'command': command
            })
            return
        
        self.is_busy = True
        self.current_task = command.get('task_type', 'unknown')
        
        # 작업 실행을 별도 스레드에서 수행
        threading.Thread(target=self._execute_task_thread, args=(command,), daemon=True).start()
    
    def _execute_task_thread(self, command):
        """작업 실행 스레드"""
        try:
            task_type = command.get('task_type')
            
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
            else:
                result = {
                    'type': 'error',
                    'message': f'알 수 없는 작업 타입: {task_type}'
                }
            
            self.send_result(result)
            
        except Exception as e:
            self.get_logger().error(f'Task execution failed: {e}')
            self.send_result({
                'type': 'error',
                'message': f'작업 실행 중 오류 발생: {str(e)}'
            })
        finally:
            self.is_busy = False
            self.current_task = None
            self.send_status_update()
    
    def move_to_position(self, command):
        """지정된 위치로 이동"""
        if not self.robot_controller:
            return {'type': 'simulation', 'message': '시뮬레이션: 위치 이동 완료'}
        
        try:
            x = command.get('x', 0)
            y = command.get('y', 0)
            z = command.get('z', 0)
            r = command.get('r', 0)
            
            self.robot_controller.move_to_position(x, y, z, r)
            
            return {
                'type': 'success',
                'message': f'위치 이동 완료: ({x}, {y}, {z}, {r})',
                'position': {'x': x, 'y': y, 'z': z, 'r': r}
            }
        except Exception as e:
            return {'type': 'error', 'message': f'위치 이동 실패: {str(e)}'}
    
    def pickup_furniture(self, command):
        """가구 픽업 작업"""
        if not self.robot_controller:
            return {'type': 'simulation', 'message': '시뮬레이션: 가구 픽업 완료'}
        
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
                    'furniture_type': furniture_type
                }
            else:
                return {
                    'type': 'error',
                    'message': f'{furniture_type} 픽업 실패'
                }
        except Exception as e:
            return {'type': 'error', 'message': f'픽업 작업 실패: {str(e)}'}
    
    def _execute_pickup_sequence(self, furniture_type):
        """기본 픽업 시퀀스"""
        try:
            # 가구별 기본 위치 (Dobot 프로젝트의 config에서 가져올 수 있음)
            positions = {
                'chair': [200, 100, -50, 0],
                'sofa': [250, 0, -30, 0],
                'desk': [300, -100, -40, 0],
                'bed': [350, 50, -20, 0]
            }
            
            target_pos = positions.get(furniture_type, [200, 0, -50, 0])
            
            # 1. 안전 위치로 이동
            self.robot_controller.move_to_position(target_pos[0], target_pos[1], target_pos[2] + 50, target_pos[3])
            time.sleep(1)
            
            # 2. 그리퍼 열기
            self.robot_controller.set_gripper(False)
            time.sleep(1)
            
            # 3. 목표 위치로 하강
            self.robot_controller.move_to_position(target_pos[0], target_pos[1], target_pos[2], target_pos[3])
            time.sleep(1)
            
            # 4. 그리퍼 닫기
            self.robot_controller.set_gripper(True)
            time.sleep(1.5)
            
            # 5. 안전 위치로 상승
            self.robot_controller.move_to_position(target_pos[0], target_pos[1], target_pos[2] + 50, target_pos[3])
            time.sleep(1)
            
            # 6. 홈 위치로 이동
            self.robot_controller.move_to_home()
            
            return True
        except Exception as e:
            self.get_logger().error(f'Pickup sequence failed: {e}')
            return False
    
    def detect_objects(self, command):
        """YOLO를 사용한 객체 인식"""
        if not self.yolo_detector:
            return {'type': 'simulation', 'message': '시뮬레이션: 객체 인식 완료'}
        
        try:
            # YOLO 객체 인식 실행
            results = self.yolo_detector.detect_objects()
            
            return {
                'type': 'success',
                'message': '객체 인식 완료',
                'detected_objects': results
            }
        except Exception as e:
            return {'type': 'error', 'message': f'객체 인식 실패: {str(e)}'}
    
    def control_gripper(self, command):
        """그리퍼 제어"""
        if not self.robot_controller:
            return {'type': 'simulation', 'message': '시뮬레이션: 그리퍼 제어 완료'}
        
        try:
            state = command.get('state', True)  # True: 닫기, False: 열기
            self.robot_controller.set_gripper(state)
            
            return {
                'type': 'success',
                'message': f'그리퍼 {"닫기" if state else "열기"} 완료',
                'gripper_state': state
            }
        except Exception as e:
            return {'type': 'error', 'message': f'그리퍼 제어 실패: {str(e)}'}
    
    def move_to_home(self):
        """홈 위치로 이동"""
        if not self.robot_controller:
            return {'type': 'simulation', 'message': '시뮬레이션: 홈 위치 이동 완료'}
        
        try:
            self.robot_controller.move_to_home()
            return {
                'type': 'success',
                'message': '홈 위치 이동 완료'
            }
        except Exception as e:
            return {'type': 'error', 'message': f'홈 위치 이동 실패: {str(e)}'}
    
    def send_result(self, result):
        """작업 결과를 서버로 전송"""
        result_data = {
            'robot_id': self.robot_id,
            'type': 'result',
            'result': result,
            'timestamp': time.time()
        }
        
        try:
            # 소켓으로 전송
            self.sock.sendall(json.dumps(result_data).encode('utf-8'))
            
            # ROS2 토픽으로도 퍼블리시
            self.result_pub.publish(String(data=json.dumps(result_data)))
            
            self.get_logger().info(f'Result sent: {result}')
            
            # 작업 이력에 추가
            self.task_history.append({
                'timestamp': time.time(),
                'result': result
            })
            
        except Exception as e:
            self.get_logger().error(f'Result send failed: {e}')
    
    def receive_loop(self):
        """서버로부터 오는 메시지를 수신"""
        while rclpy.ok():
            try:
                data = self.sock.recv(4096)  # 더 큰 버퍼 사용
                if not data:
                    self.get_logger().info('Server closed connection')
                    break
                
                text = data.decode('utf-8').strip()
                self.get_logger().info(f'Received from server: {text}')
                
                try:
                    # JSON 명령 파싱 시도
                    command = json.loads(text)
                    if isinstance(command, dict):
                        self.execute_command(command)
                    else:
                        self.get_logger().warning(f'Invalid command format: {text}')
                except json.JSONDecodeError:
                    # JSON이 아닌 경우 일반 텍스트 메시지로 처리
                    self.get_logger().info(f'Text message from server: {text}')
                
            except Exception as e:
                self.get_logger().error(f'Receive failed: {e}')
                break
    
    def destroy_node(self):
        """노드 종료 시 정리"""
        try:
            # 로봇을 안전 위치로 이동
            if self.robot_controller:
                try:
                    self.robot_controller.move_to_home()
                    self.robot_controller.disconnect()
                except:
                    pass
            
            # 소켓 연결 종료
            self.sock.close()
        except:
            pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = DobotROS2ClientNode()
        
        print("\n" + "="*60)
        print("Dobot ROS2 클라이언트가 시작되었습니다!")
        print("서버로부터 작업 명령을 대기 중입니다.")
        print("지원되는 작업:")
        print("- move_to_position: 특정 위치로 이동")
        print("- pickup_furniture: 가구 픽업")
        print("- detect_objects: 객체 인식")
        print("- gripper_control: 그리퍼 제어")
        print("- home_position: 홈 위치로 이동")
        print("="*60)
        
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
