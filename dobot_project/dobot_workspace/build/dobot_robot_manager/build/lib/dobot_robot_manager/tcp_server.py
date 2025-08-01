#!/usr/bin/env python3
"""
독립 실행 가능한 TCP 서버
ROS2 없이도 동작하는 순수 TCP 기반 다중 로봇 관리 서버
"""

import socket
import threading
import json
import time
import argparse
import signal
import sys
from datetime import datetime
from typing import Dict, List, Optional
from dataclasses import dataclass, field

# 내부 모듈들 (ROS2 의존성 없음)
try:
    from . import utils
    from .utils import Robot, Task, TaskStatus, CommunicationType, LogLevel, setup_logger
except ImportError:
    # 독립 실행 시 상대 임포트 실패하면 절대 임포트 시도
    import utils
    from utils import Robot, Task, TaskStatus, CommunicationType, LogLevel, setup_logger


class StandaloneTCPServer:
    """
    ROS2 없이 동작하는 독립 TCP 서버
    """
    
    def __init__(self, host: str = '127.0.1.1', port: int = 9988, max_robots: int = 10):
        self.host = host
        self.port = port
        self.max_robots = max_robots
        
        # 데이터 구조
        self.robots: Dict[str, Robot] = {}
        self.tasks: Dict[str, Task] = {}
        self.task_queue: List[str] = []
        self.lock = threading.Lock()
        self.task_counter = 0
        
        # 서버 소켓
        self.server_socket = None
        self.running = False
        
        # 로거 설정
        self.logger = setup_logger('TCPServer', LogLevel.INFO, 'tcp_server.log')
        
        # 통계
        self.stats = {
            'start_time': time.time(),
            'total_connections': 0,
            'total_tasks': 0,
            'completed_tasks': 0,
            'failed_tasks': 0
        }
        
        # 시그널 핸들러 설정
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
    
    def signal_handler(self, signum, frame):
        """시그널 핸들러 (Ctrl+C 등)"""
        self.logger.info(f'시그널 {signum} 수신. 서버를 종료합니다.')
        self.shutdown()
        sys.exit(0)
    
    def start(self):
        """서버 시작"""
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.bind((self.host, self.port))
            self.server_socket.listen(self.max_robots)
            
            self.running = True
            self.logger.info(f'TCP 서버 시작: {self.host}:{self.port}')
            self.logger.info(f'최대 로봇 수: {self.max_robots}')
            
            # 백그라운드 스레드들 시작
            self.start_background_threads()
            
            # 클라이언트 연결 대기 루프
            self.accept_connections()
            
        except Exception as e:
            self.logger.error(f'서버 시작 실패: {e}')
            raise
    
    def start_background_threads(self):
        """백그라운드 스레드들 시작"""
        threads = [
            ('TaskDistributor', self.task_distributor),
            ('RobotMonitor', self.robot_monitor),
            ('StatsCollector', self.stats_collector),
            ('CommandInterface', self.command_interface)
        ]
        
        for name, target in threads:
            thread = threading.Thread(target=target, name=name, daemon=True)
            thread.start()
            self.logger.debug(f'{name} 스레드 시작됨')
    
    def accept_connections(self):
        """클라이언트 연결 수락"""
        while self.running:
            try:
                if self.server_socket:
                    conn, addr = self.server_socket.accept()
                    
                    with self.lock:
                        if len(self.robots) >= self.max_robots:
                            self.logger.warning(f'최대 로봇 수 초과. 연결 거부: {addr}')
                            conn.close()
                            continue
                    
                    # 클라이언트 처리 스레드 시작
                    client_thread = threading.Thread(
                        target=self.handle_client,
                        args=(conn, addr),
                        daemon=True
                    )
                    client_thread.start()
                    
                    self.stats['total_connections'] += 1
                    
            except OSError:
                if self.running:
                    self.logger.error('연결 수락 중 오류 발생')
                break
            except Exception as e:
                self.logger.error(f'연결 처리 오류: {e}')
    
    def handle_client(self, conn: socket.socket, addr):
        """클라이언트 연결 처리"""
        robot_id = None
        try:
            # 클라이언트로부터 로봇 ID 수신
            data = conn.recv(1024)
            if not data:
                return
            
            robot_id = data.decode('utf-8').strip()
            if not robot_id:
                self.logger.warning(f'빈 로봇 ID: {addr}')
                return
            
            # 로봇 등록
            with self.lock:
                if robot_id in self.robots:
                    self.logger.warning(f'로봇 ID 중복: {robot_id}')
                    # 기존 연결 해제
                    old_robot = self.robots[robot_id]
                    if old_robot.conn:
                        old_robot.conn.close()
                
                self.robots[robot_id] = Robot(
                    id=robot_id,
                    conn=conn,
                    comm_type=CommunicationType.TCP,
                    last_seen=time.time()
                )
            
            self.logger.info(f'로봇 연결: {robot_id} @ {addr}')
            self.show_robot_status()
            
            # 메시지 수신 루프
            self.client_message_loop(robot_id, conn)
            
        except Exception as e:
            self.logger.error(f'클라이언트 처리 오류 {robot_id}: {e}')
        finally:
            # 연결 정리
            if robot_id:
                with self.lock:
                    self.robots.pop(robot_id, None)
                self.logger.info(f'로봇 연결 해제: {robot_id}')
                self.show_robot_status()
            conn.close()
    
    def client_message_loop(self, robot_id: str, conn: socket.socket):
        """클라이언트 메시지 수신 루프"""
        while self.running:
            try:
                data = conn.recv(4096)
                if not data:
                    break
                
                message_str = data.decode('utf-8').strip()
                self.logger.debug(f'메시지 수신 {robot_id}: {message_str}')
                
                try:
                    message = json.loads(message_str)
                    self.process_robot_message(robot_id, message)
                except json.JSONDecodeError:
                    self.logger.warning(f'잘못된 JSON {robot_id}: {message_str}')
                
            except socket.timeout:
                continue
            except ConnectionResetError:
                self.logger.info(f'클라이언트 연결 해제: {robot_id}')
                break
            except Exception as e:
                self.logger.error(f'메시지 수신 오류 {robot_id}: {e}')
                break
    
    def process_robot_message(self, robot_id: str, message: dict):
        """로봇 메시지 처리"""
        msg_type = message.get('type')
        
        if msg_type == 'status':
            self.update_robot_status(robot_id, message)
        elif msg_type == 'result':
            self.process_task_result(robot_id, message)
        else:
            self.logger.warning(f'알 수 없는 메시지 타입 {robot_id}: {msg_type}')
    
    def update_robot_status(self, robot_id: str, status_msg: dict):
        """로봇 상태 업데이트"""
        with self.lock:
            robot = self.robots.get(robot_id)
            if robot:
                robot.is_busy = status_msg.get('is_busy', False)
                robot.current_task = status_msg.get('current_task')
                robot.dobot_available = status_msg.get('dobot_available', False)
                robot.yolo_available = status_msg.get('yolo_available', False)
                robot.last_seen = time.time()
                
                # 위치 정보 업데이트
                if 'position' in status_msg:
                    robot.position = status_msg['position']
                
                # 그리퍼 상태 업데이트  
                if 'gripper_state' in status_msg:
                    robot.gripper_state = status_msg['gripper_state']
        
        self.logger.debug(f'상태 업데이트 {robot_id}: {"작업중" if status_msg.get("is_busy") else "대기중"}')
    
    def process_task_result(self, robot_id: str, result_msg: dict):
        """작업 결과 처리"""
        result = result_msg.get('result', {})
        
        with self.lock:
            robot = self.robots.get(robot_id)
            if robot and robot.current_task:
                # 작업 완료 처리
                task = self.tasks.get(robot.current_task)
                if task:
                    if result.get('type') == 'success':
                        task.complete_successfully(result)
                        self.stats['completed_tasks'] += 1
                    else:
                        task.fail_with_error(result.get('message', '알 수 없는 오류'))
                        self.stats['failed_tasks'] += 1
                
                # 로봇 상태 업데이트
                robot.is_busy = False
                robot.current_task = None
                robot.add_task_to_history({
                    'task_id': robot.current_task,
                    'result': result
                })
        
        self.logger.info(f'작업 완료 {robot_id}: {result.get("message", "결과 없음")}')
    
    def send_task_to_robot(self, robot_id: str, task: Task) -> bool:
        """로봇에게 작업 전송"""
        with self.lock:
            robot = self.robots.get(robot_id)
        
        if not robot or not robot.conn:
            self.logger.error(f'로봇을 찾을 수 없음: {robot_id}')
            return False
        
        command = {
            'task_id': task.id,
            'task_type': task.task_type,
            **task.parameters
        }
        
        try:
            robot.conn.sendall(json.dumps(command).encode('utf-8'))
            
            # 상태 업데이트
            task.start_execution()
            task.robot_id = robot_id
            
            with self.lock:
                robot.is_busy = True
                robot.current_task = task.id
            
            self.logger.info(f'작업 전송 {robot_id}: {task.task_type}')
            return True
            
        except Exception as e:
            self.logger.error(f'작업 전송 실패 {robot_id}: {e}')
            return False
    
    def add_task(self, task_type: str, parameters: dict, priority: int = 0, 
                 target_robot_id: Optional[str] = None) -> str:
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
                if self.send_task_to_robot(target_robot_id, task):
                    self.logger.info(f'작업 직접 할당: {task_id} → {target_robot_id}')
                else:
                    # 직접 할당 실패시 큐에 추가
                    task.robot_id = None
                    self.task_queue.append(task_id)
            else:
                # 큐에 추가하여 자동 분배
                self.task_queue.append(task_id)
                self.task_queue.sort(key=lambda tid: self.tasks[tid].priority, reverse=True)
        
        self.stats['total_tasks'] += 1
        self.logger.info(f'작업 추가: {task_type} (ID: {task_id}, Priority: {priority})')
        
        return task_id
    
    def task_distributor(self):
        """작업 분배 스레드"""
        while self.running:
            try:
                if self.task_queue:
                    with self.lock:
                        # 사용 가능한 로봇 찾기
                        available_robots = [
                            robot for robot in self.robots.values()
                            if not robot.is_busy and robot.is_connected()
                        ]
                        
                        if available_robots and self.task_queue:
                            # 우선순위 높은 작업 먼저
                            task_id = self.task_queue.pop(0)
                            task = self.tasks.get(task_id)
                            
                            if task:
                                # 첫 번째 사용 가능한 로봇 선택 (나중에 더 스마트한 로직 추가 가능)
                                selected_robot = available_robots[0]
                                
                                if self.send_task_to_robot(selected_robot.id, task):
                                    self.logger.info(f'작업 자동 분배: {task.task_type} → {selected_robot.id}')
                
                time.sleep(2)  # 2초마다 확인
                
            except Exception as e:
                self.logger.error(f'작업 분배 오류: {e}')
                time.sleep(5)
    
    def robot_monitor(self):
        """로봇 모니터링 스레드"""
        while self.running:
            try:
                current_time = time.time()
                disconnected_robots = []
                
                with self.lock:
                    for robot_id, robot in list(self.robots.items()):
                        if not robot.is_connected(60.0):  # 60초 타임아웃
                            disconnected_robots.append(robot_id)
                            # 현재 작업 실패 처리
                            if robot.current_task:
                                task = self.tasks.get(robot.current_task)
                                if task:
                                    task.fail_with_error('Robot disconnected')
                                    self.stats['failed_tasks'] += 1
                    
                    # 연결 끊긴 로봇 제거
                    for robot_id in disconnected_robots:
                        self.robots.pop(robot_id, None)
                        self.logger.warning(f'로봇 연결 타임아웃: {robot_id}')
                
                time.sleep(30)  # 30초마다 확인
                
            except Exception as e:
                self.logger.error(f'로봇 모니터링 오류: {e}')
                time.sleep(60)
    
    def stats_collector(self):
        """통계 수집 스레드"""
        while self.running:
            try:
                with self.lock:
                    uptime = time.time() - self.stats['start_time']
                    active_robots = len([r for r in self.robots.values() if not r.is_busy])
                    busy_robots = len([r for r in self.robots.values() if r.is_busy])
                    pending_tasks = len(self.task_queue)
                    active_tasks = len([t for t in self.tasks.values() if t.status == TaskStatus.IN_PROGRESS])
                
                # 통계 로그 (1분마다)
                if int(uptime) % 60 == 0:
                    self.logger.info(
                        f'통계 - 업타임: {uptime/3600:.1f}h, '
                        f'로봇: {len(self.robots)}대 (활성: {active_robots}, 작업중: {busy_robots}), '
                        f'작업: 대기 {pending_tasks}, 진행중 {active_tasks}, '
                        f'완료 {self.stats["completed_tasks"]}, 실패 {self.stats["failed_tasks"]}'
                    )
                
                time.sleep(60)  # 1분마다
                
            except Exception as e:
                self.logger.error(f'통계 수집 오류: {e}')
                time.sleep(60)
    
    def command_interface(self):
        """명령어 인터페이스 스레드"""
        self.print_help()
        
        while self.running:
            try:
                line = input("TCP서버> ").strip()
                if not line:
                    continue
                
                self.process_command(line)
                
            except EOFError:
                break
            except KeyboardInterrupt:
                break
            except Exception as e:
                print(f"명령 처리 오류: {e}")
    
    def process_command(self, command_line: str):
        """명령어 처리"""
        parts = command_line.split()
        if not parts:
            return
        
        command = parts[0].lower()
        
        if command == 'help' or command == 'h':
            self.print_help()
        
        elif command == 'status' or command == 's':
            self.show_robot_status()
        
        elif command == 'tasks' or command == 't':
            self.show_task_status()
        
        elif command == 'stats':
            self.show_statistics()
        
        elif command == 'move' and len(parts) >= 6:
            robot_id = parts[1]
            try:
                x, y, z, r = map(float, parts[2:6])
                task_id = self.add_task('move_to_position', 
                                       {'x': x, 'y': y, 'z': z, 'r': r}, 
                                       target_robot_id=robot_id)
                print(f"이동 작업 추가: {task_id}")
            except ValueError:
                print("잘못된 좌표 형식")
        
        elif command == 'pickup' and len(parts) >= 3:
            robot_id = parts[1]
            furniture_type = parts[2]
            task_id = self.add_task('pickup_furniture', 
                                   {'furniture_type': furniture_type}, 
                                   target_robot_id=robot_id)
            print(f"픽업 작업 추가: {task_id}")
        
        elif command == 'detect' and len(parts) >= 2:
            robot_id = parts[1]
            task_id = self.add_task('detect_objects', {}, target_robot_id=robot_id)
            print(f"객체 인식 작업 추가: {task_id}")
        
        elif command == 'gripper' and len(parts) >= 3:
            robot_id = parts[1]
            state = parts[2].lower() == 'close'
            task_id = self.add_task('gripper_control', {'state': state}, target_robot_id=robot_id)
            print(f"그리퍼 제어 작업 추가: {task_id}")
        
        elif command == 'home' and len(parts) >= 2:
            robot_id = parts[1]
            task_id = self.add_task('home_position', {}, target_robot_id=robot_id)
            print(f"홈 위치 작업 추가: {task_id}")
        
        elif command == 'emergency' or command == 'stop':
            self.emergency_stop_all()
            print("모든 로봇 긴급 정지")
        
        elif command == 'quit' or command == 'q':
            self.shutdown()
        
        else:
            print("알 수 없는 명령어. 'help'를 입력하세요.")
    
    def print_help(self):
        """도움말 출력"""
        help_text = """
=== TCP 서버 명령어 ===
help, h          - 이 도움말 표시
status, s        - 로봇 상태 표시
tasks, t         - 작업 상태 표시  
stats            - 서버 통계 표시
move <robot_id> <x> <y> <z> <r> - 로봇 이동
pickup <robot_id> <furniture>    - 가구 픽업
detect <robot_id>               - 객체 인식
gripper <robot_id> <open|close> - 그리퍼 제어
home <robot_id>                 - 홈 위치로 이동
emergency, stop                 - 모든 로봇 긴급 정지
quit, q                         - 서버 종료
========================
        """
        print(help_text)
    
    def show_robot_status(self):
        """로봇 상태 표시"""
        with self.lock:
            robots = list(self.robots.values())
        
        print("\n" + "="*80)
        print(f"로봇 상태 ({len(robots)}대)")
        print("="*80)
        
        if robots:
            print(f"{'ID':<15} {'상태':<10} {'Dobot':<8} {'YOLO':<8} {'현재 작업':<15} {'마지막 연결'}")
            print("-"*80)
            
            for robot in robots:
                status = "작업중" if robot.is_busy else "대기중"
                dobot = "✓" if robot.dobot_available else "✗"
                yolo = "✓" if robot.yolo_available else "✗"
                current_task = robot.current_task or "-"
                last_seen = datetime.fromtimestamp(robot.last_seen).strftime("%H:%M:%S")
                
                print(f"{robot.id:<15} {status:<10} {dobot:<8} {yolo:<8} {current_task:<15} {last_seen}")
        else:
            print("연결된 로봇이 없습니다.")
        
        print("="*80 + "\n")
    
    def show_task_status(self):
        """작업 상태 표시"""
        with self.lock:
            tasks = list(self.tasks.values())
            queue_tasks = [self.tasks[tid] for tid in self.task_queue if tid in self.tasks]
        
        print("\n" + "="*90)
        print(f"작업 상태 (전체: {len(tasks)}, 대기: {len(queue_tasks)})")
        print("="*90)
        
        if tasks:
            print(f"{'ID':<12} {'타입':<20} {'상태':<12} {'로봇':<15} {'생성 시간':<10} {'소요 시간'}")
            print("-"*90)
            
            # 최근 작업 20개만 표시
            recent_tasks = sorted(tasks, key=lambda t: t.created_at, reverse=True)[:20]
            
            for task in recent_tasks:
                created_time = datetime.fromtimestamp(task.created_at).strftime("%H:%M:%S")
                robot_id = task.robot_id or "-"
                
                # 소요 시간 계산
                if task.completed_at:
                    duration = f"{task.completed_at - task.created_at:.1f}s"
                elif task.started_at:
                    duration = f"{time.time() - task.started_at:.1f}s"
                else:
                    duration = "-"
                
                print(f"{task.id:<12} {task.task_type:<20} {task.status.value:<12} {robot_id:<15} {created_time:<10} {duration}")
        else:
            print("작업이 없습니다.")
        
        print("="*90 + "\n")
    
    def show_statistics(self):
        """서버 통계 표시"""
        uptime = time.time() - self.stats['start_time']
        
        with self.lock:
            robot_count = len(self.robots)
            active_robots = len([r for r in self.robots.values() if not r.is_busy])
            busy_robots = len([r for r in self.robots.values() if r.is_busy])
            task_count = len(self.tasks)
            pending_tasks = len(self.task_queue)
            active_tasks = len([t for t in self.tasks.values() if t.status == TaskStatus.IN_PROGRESS])
        
        print("\n" + "="*50)
        print("서버 통계")
        print("="*50)
        print(f"업타임: {uptime/3600:.1f} 시간")
        print(f"총 연결 수: {self.stats['total_connections']}")
        print(f"현재 로봇: {robot_count}대 (활성: {active_robots}, 작업중: {busy_robots})")
        print(f"총 작업 수: {self.stats['total_tasks']}")
        print(f"현재 작업: 대기 {pending_tasks}, 진행중 {active_tasks}")
        print(f"완료 작업: {self.stats['completed_tasks']}")
        print(f"실패 작업: {self.stats['failed_tasks']}")
        
        if self.stats['total_tasks'] > 0:
            success_rate = (self.stats['completed_tasks'] / self.stats['total_tasks']) * 100
            print(f"성공률: {success_rate:.1f}%")
        
        print("="*50 + "\n")
    
    def emergency_stop_all(self):
        """모든 로봇 긴급 정지"""
        emergency_command = {
            'task_type': 'emergency_stop',
            'parameters': {}
        }
        
        with self.lock:
            for robot in self.robots.values():
                try:
                    if robot.conn:
                        robot.conn.sendall(json.dumps(emergency_command).encode('utf-8'))
                    
                    # 현재 작업 취소
                    if robot.current_task:
                        task = self.tasks.get(robot.current_task)
                        if task:
                            task.status = TaskStatus.CANCELLED
                            task.completed_at = time.time()
                    
                    robot.is_busy = False
                    robot.current_task = None
                    
                except Exception as e:
                    self.logger.error(f'긴급 정지 전송 실패 {robot.id}: {e}')
        
        self.logger.warning('모든 로봇에게 긴급 정지 신호 전송')
    
    def shutdown(self):
        """서버 종료"""
        self.logger.info('서버 종료 중...')
        self.running = False
        
        # 모든 로봇에게 종료 알림
        with self.lock:
            for robot in self.robots.values():
                if robot.conn:
                    try:
                        robot.conn.close()
                    except:
                        pass
        
        # 서버 소켓 종료
        if self.server_socket:
            try:
                self.server_socket.close()
            except:
                pass
        
        self.logger.info('서버 종료 완료')


def main():
    """메인 함수"""
    parser = argparse.ArgumentParser(description='Dobot TCP 서버')
    parser.add_argument('--host', default='127.0.1.1', help='서버 호스트 (기본값: 127.0.1.1)')
    parser.add_argument('--port', type=int, default=9988, help='서버 포트 (기본값: 9988)')
    parser.add_argument('--max-robots', type=int, default=10, help='최대 로봇 수 (기본값: 10)')
    parser.add_argument('--log-level', choices=['DEBUG', 'INFO', 'WARN', 'ERROR'], 
                       default='INFO', help='로그 레벨 (기본값: INFO)')
    
    args = parser.parse_args()
    
    try:
        server = StandaloneTCPServer(
            host=args.host,
            port=args.port,
            max_robots=args.max_robots
        )
        
        print(f"\n{'='*60}")
        print("Dobot 다중 로봇 TCP 서버")
        print(f"{'='*60}")
        print(f"호스트: {args.host}")
        print(f"포트: {args.port}")
        print(f"최대 로봇 수: {args.max_robots}")
        print(f"로그 레벨: {args.log_level}")
        print(f"{'='*60}\n")
        
        server.start()
        
    except KeyboardInterrupt:
        print("\n키보드 인터럽트로 종료합니다.")
    except Exception as e:
        print(f"서버 오류: {e}")
    finally:
        print("서버가 종료되었습니다.")


if __name__ == '__main__':
    main()