import socket
import threading
import json
import time
from datetime import datetime
from enum import Enum
from dataclasses import dataclass
from typing import Dict, List, Optional

HOST = '127.0.1.1'
PORT = 9988

class TaskStatus(Enum):
    PENDING = "pending"
    IN_PROGRESS = "in_progress"
    COMPLETED = "completed"
    FAILED = "failed"

@dataclass
class Robot:
    id: str
    conn: socket.socket
    is_busy: bool = False
    current_task: Optional[str] = None
    dobot_available: bool = False
    yolo_available: bool = False
    last_seen: float = 0
    task_history: List[dict] = None
    
    def __post_init__(self):
        if self.task_history is None:
            self.task_history = []

@dataclass
class Task:
    id: str
    task_type: str
    parameters: dict
    robot_id: Optional[str] = None
    status: TaskStatus = TaskStatus.PENDING
    created_at: float = 0
    completed_at: Optional[float] = None
    result: Optional[dict] = None
    
    def __post_init__(self):
        if self.created_at == 0:
            self.created_at = time.time()

class DobotMultiRobotServer:
    def __init__(self):
        self.robots: Dict[str, Robot] = {}
        self.tasks: Dict[str, Task] = {}
        self.task_queue: List[str] = []
        self.lock = threading.Lock()
        self.task_counter = 0
        
        # 작업 분배 스레드 시작
        threading.Thread(target=self.task_distributor, daemon=True).start()
        
        # 로봇 상태 모니터링 스레드 시작
        threading.Thread(target=self.robot_monitor, daemon=True).start()
    
    def handle_client(self, conn, addr):
        """클라이언트 연결 처리"""
        robot_id = None
        try:
            # 1) 최초 메시지로 클라이언트 ID 수신
            robot_id = conn.recv(1024).decode('utf-8').strip()
            if not robot_id:
                return
            
            # 2) 로봇 등록
            with self.lock:
                self.robots[robot_id] = Robot(
                    id=robot_id,
                    conn=conn,
                    last_seen=time.time()
                )
            
            print(f"[접속] 로봇 {robot_id} @ {addr}")
            self.show_robot_status()
            
            # 3) 메시지 수신 루프
            while True:
                data = conn.recv(4096)
                if not data:
                    break
                
                try:
                    message = json.loads(data.decode('utf-8'))
                    self.process_robot_message(robot_id, message)
                except json.JSONDecodeError:
                    # JSON이 아닌 일반 텍스트
                    text = data.decode('utf-8').strip()
                    print(f"[수신 {robot_id}] {text}")
                
        except Exception as e:
            print(f"[에러] {robot_id}: {e}")
        finally:
            # 연결 종료 시 로봇 제거
            if robot_id:
                with self.lock:
                    self.robots.pop(robot_id, None)
                print(f"[해제] {robot_id}")
                self.show_robot_status()
            conn.close()
    
    def process_robot_message(self, robot_id: str, message: dict):
        """로봇으로부터 받은 메시지 처리"""
        msg_type = message.get('type')
        
        if msg_type == 'status':
            self.update_robot_status(robot_id, message)
        elif msg_type == 'result':
            self.process_task_result(robot_id, message)
        else:
            print(f"[알 수 없는 메시지] {robot_id}: {message}")
    
    def update_robot_status(self, robot_id: str, status_msg: dict):
        """로봇 상태 업데이트"""
        with self.lock:
            if robot_id in self.robots:
                robot = self.robots[robot_id]
                robot.is_busy = status_msg.get('is_busy', False)
                robot.current_task = status_msg.get('current_task')
                robot.dobot_available = status_msg.get('dobot_available', False)
                robot.yolo_available = status_msg.get('yolo_available', False)
                robot.last_seen = time.time()
        
        print(f"[상태 업데이트] {robot_id}: {'작업중' if status_msg.get('is_busy') else '대기중'}")
    
    def process_task_result(self, robot_id: str, result_msg: dict):
        """작업 결과 처리"""
        result = result_msg.get('result', {})
        
        # 현재 로봇이 수행중인 작업 찾기
        with self.lock:
            robot = self.robots.get(robot_id)
            if robot and robot.current_task:
                # 작업 완료 처리
                for task_id, task in self.tasks.items():
                    if task.robot_id == robot_id and task.status == TaskStatus.IN_PROGRESS:
                        task.status = TaskStatus.COMPLETED if result.get('type') == 'success' else TaskStatus.FAILED
                        task.completed_at = time.time()
                        task.result = result
                        break
                
                # 로봇 상태 업데이트
                robot.is_busy = False
                robot.current_task = None
                robot.task_history.append({
                    'timestamp': time.time(),
                    'result': result
                })
        
        print(f"[작업 완료] {robot_id}: {result.get('message', '알 수 없는 결과')}")
    
    def send_task_to_robot(self, robot_id: str, task: Task) -> bool:
        """로봇에게 작업 전송"""
        with self.lock:
            robot = self.robots.get(robot_id)
        
        if not robot:
            print(f"[오류] 로봇 '{robot_id}'를 찾을 수 없습니다.")
            return False
        
        command = {
            'task_id': task.id,
            'task_type': task.task_type,
            **task.parameters
        }
        
        try:
            robot.conn.sendall(json.dumps(command).encode('utf-8'))
            
            # 작업 상태 업데이트
            task.robot_id = robot_id
            task.status = TaskStatus.IN_PROGRESS
            
            with self.lock:
                robot.is_busy = True
                robot.current_task = task.id
            
            print(f"[작업 전송] {robot_id} → {task.task_type}")
            return True
            
        except Exception as e:
            print(f"[전송 실패] {robot_id}: {e}")
            return False
    
    def task_distributor(self):
        """작업 분배 스레드"""
        while True:
            try:
                if self.task_queue:
                    with self.lock:
                        # 사용 가능한 로봇 찾기
                        available_robots = [
                            robot for robot in self.robots.values()
                            if not robot.is_busy and (time.time() - robot.last_seen) < 30
                        ]
                        
                        if available_robots and self.task_queue:
                            # 대기 중인 작업 가져오기
                            task_id = self.task_queue.pop(0)
                            task = self.tasks.get(task_id)
                            
                            if task:
                                # 가장 적합한 로봇 선택 (단순히 첫 번째 사용 가능한 로봇)
                                selected_robot = available_robots[0]
                                
                                # 작업 전송
                                if self.send_task_to_robot(selected_robot.id, task):
                                    print(f"[작업 분배] {task.task_type} → {selected_robot.id}")
                
                time.sleep(1)  # 1초마다 확인
                
            except Exception as e:
                print(f"[작업 분배 오류] {e}")
                time.sleep(5)
    
    def robot_monitor(self):
        """로봇 상태 모니터링"""
        while True:
            try:
                current_time = time.time()
                with self.lock:
                    disconnected_robots = []
                    for robot_id, robot in self.robots.items():
                        if current_time - robot.last_seen > 60:  # 60초 무응답
                            disconnected_robots.append(robot_id)
                    
                    # 연결이 끊긴 로봇 제거
                    for robot_id in disconnected_robots:
                        print(f"[타임아웃] {robot_id} 연결 끊김")
                        self.robots.pop(robot_id, None)
                
                time.sleep(30)  # 30초마다 확인
                
            except Exception as e:
                print(f"[모니터링 오류] {e}")
                time.sleep(60)
    
    def add_task(self, task_type: str, parameters: dict) -> str:
        """새 작업 추가"""
        self.task_counter += 1
        task_id = f"task_{self.task_counter:04d}"
        
        task = Task(
            id=task_id,
            task_type=task_type,
            parameters=parameters
        )
        
        with self.lock:
            self.tasks[task_id] = task
            self.task_queue.append(task_id)
        
        print(f"[작업 추가] {task_type} (ID: {task_id})")
        return task_id
    
    def show_robot_status(self):
        """로봇 상태 표시"""
        with self.lock:
            robot_list = list(self.robots.values())
        
        print("\n" + "="*70)
        print(f"로봇 상태 ({len(robot_list)}대)")
        print("="*70)
        if robot_list:
            print(f"{'ID':<15} {'상태':<10} {'Dobot':<8} {'YOLO':<8} {'현재 작업':<15} {'마지막 연결'}")
            print("-"*70)
            for robot in robot_list:
                status = "작업중" if robot.is_busy else "대기중"
                dobot = "✓" if robot.dobot_available else "✗"
                yolo = "✓" if robot.yolo_available else "✗"
                current_task = robot.current_task or "-"
                last_seen = datetime.fromtimestamp(robot.last_seen).strftime("%H:%M:%S")
                
                print(f"{robot.id:<15} {status:<10} {dobot:<8} {yolo:<8} {current_task:<15} {last_seen}")
        else:
            print("접속된 로봇이 없습니다.")
        print("="*70 + "\n")
    
    def show_task_status(self):
        """작업 상태 표시"""
        with self.lock:
            tasks = list(self.tasks.values())
        
        print("\n" + "="*80)
        print(f"작업 상태 ({len(tasks)}개)")
        print("="*80)
        if tasks:
            print(f"{'ID':<12} {'타입':<20} {'상태':<12} {'로봇':<15} {'생성 시간':<10}")
            print("-"*80)
            for task in sorted(tasks, key=lambda t: t.created_at, reverse=True)[:10]:  # 최근 10개만
                created_time = datetime.fromtimestamp(task.created_at).strftime("%H:%M:%S")
                robot_id = task.robot_id or "-"
                
                print(f"{task.id:<12} {task.task_type:<20} {task.status.value:<12} {robot_id:<15} {created_time}")
        else:
            print("작업이 없습니다.")
        print("="*80 + "\n")

def command_loop(server: DobotMultiRobotServer):
    """서버 콘솔에서 명령을 입력받아 처리"""
    print("\n=== Dobot 다중 로봇 관리 서버 ===")
    print("명령어:")
    print("  status - 로봇 상태 표시")
    print("  tasks - 작업 상태 표시")
    print("  move <robot_id> <x> <y> <z> <r> - 로봇 이동")
    print("  pickup <robot_id> <furniture_type> - 가구 픽업")
    print("  detect <robot_id> - 객체 인식")
    print("  gripper <robot_id> <open|close> - 그리퍼 제어")
    print("  home <robot_id> - 홈 위치로 이동")
    print("  broadcast <message> - 모든 로봇에게 메시지 전송")
    print("  quit - 서버 종료\n")
    
    while True:
        try:
            line = input("서버 명령> ").strip()
            if not line:
                continue
            
            parts = line.split()
            command = parts[0].lower()
            
            if command == 'status':
                server.show_robot_status()
            
            elif command == 'tasks':
                server.show_task_status()
            
            elif command == 'move' and len(parts) >= 6:
                robot_id = parts[1]
                x, y, z, r = map(float, parts[2:6])
                server.add_task('move_to_position', {
                    'x': x, 'y': y, 'z': z, 'r': r
                })
            
            elif command == 'pickup' and len(parts) >= 3:
                robot_id = parts[1]
                furniture_type = parts[2]
                server.add_task('pickup_furniture', {
                    'furniture_type': furniture_type
                })
            
            elif command == 'detect' and len(parts) >= 2:
                robot_id = parts[1]
                server.add_task('detect_objects', {})
            
            elif command == 'gripper' and len(parts) >= 3:
                robot_id = parts[1]
                state = parts[2].lower() == 'close'
                server.add_task('gripper_control', {
                    'state': state
                })
            
            elif command == 'home' and len(parts) >= 2:
                robot_id = parts[1]
                server.add_task('home_position', {})
            
            elif command == 'broadcast' and len(parts) >= 2:
                message = ' '.join(parts[1:])
                with server.lock:
                    for robot in server.robots.values():
                        try:
                            robot.conn.sendall(message.encode('utf-8'))
                        except:
                            pass
                print(f"[브로드캐스트] {message}")
            
            elif command == 'quit':
                print("서버를 종료합니다.")
                break
            
            else:
                print("잘못된 명령어입니다. 사용 가능한 명령어를 확인하세요.")
                
        except KeyboardInterrupt:
            print("\n서버를 종료합니다.")
            break
        except Exception as e:
            print(f"명령 처리 오류: {e}")

def main():
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((HOST, PORT))
    server_socket.listen()
    
    print(f"[서버 시작] {HOST}:{PORT}")
    
    # 서버 인스턴스 생성
    server = DobotMultiRobotServer()
    
    # 콘솔 명령 처리 스레드 시작
    threading.Thread(target=command_loop, args=(server,), daemon=True).start()
    
    try:
        # 클라이언트 연결 대기
        while True:
            conn, addr = server_socket.accept()
            threading.Thread(target=server.handle_client, args=(conn, addr), daemon=True).start()
    except KeyboardInterrupt:
        print("\n서버를 종료합니다.")
    finally:
        server_socket.close()

if __name__ == '__main__':
    main()
