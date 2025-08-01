"""
Dobot 제어 시스템 메인 실행 파일 - 클라이언트 메시지 전송 기능 추가
"""
import sys
import os
import threading
import time
import platform
import socket
import json

import logging
logging.getLogger('werkzeug').setLevel(logging.ERROR)
# Python 경로 문제 해결
current_dir = os.path.dirname(os.path.abspath(__file__))
if current_dir not in sys.path:
    sys.path.insert(0, current_dir)

# Flask 사전 테스트
try:
    import flask
    FLASK_WORKING = True
except ImportError:
    FLASK_WORKING = False

# 모듈 import
from dobot_tcp_server import DobotTCPServer
from dobot_gui import DobotGUI
from http_api_server import HTTPAPIServer
from utils.logger import LoggerSetup
from utils.config_loader import ConfigLoader


class ClientManager:
    """클라이언트 연결 관리 및 메시지 전송 클래스"""
    
    def __init__(self, host='127.0.0.1', port=9988):
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
            self.server_socket.bind((self.host, self.port))
            self.server_socket.listen()
            self.running = True
            
            print(f"📡 클라이언트 서버 시작: {self.host}:{self.port}")
            
            # 콘솔 명령 처리 스레드 시작
            threading.Thread(target=self.command_loop, daemon=True).start()
            
            # 클라이언트 연결 대기 스레드 시작
            threading.Thread(target=self.accept_clients, daemon=True).start()
            
        except Exception as e:
            print(f"❌ 클라이언트 서버 시작 실패: {e}")
            self.running = False
    
    def accept_clients(self):
        """클라이언트 연결 수락"""
        while self.running:
            try:
                conn, addr = self.server_socket.accept()
                threading.Thread(target=self.handle_client, args=(conn, addr), daemon=True).start()
            except Exception as e:
                if self.running:
                    print(f"❌ 클라이언트 연결 수락 오류: {e}")
                break
    
    def handle_client(self, conn, addr):
        """클라이언트 연결 처리"""
        client_id = None
        try:
            # 연결 확인 메시지 전송
            conn.sendall(b"CONNECTED:Please enter your name\n")
            
            # 클라이언트로부터 이름 수신
            conn.settimeout(30.0)  # 30초 타임아웃
            name_data = conn.recv(1024).decode('utf-8').strip()
            
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
                    'connected_at': time.time()
                }
            
            # 환영 메시지 전송
            welcome_msg = f"WELCOME:{client_id}\n"
            conn.sendall(welcome_msg.encode('utf-8'))
            
            print(f"[접속] {name_data} (ID: {client_id}) @ {addr}")
            self.show_client_list()
            
            # 모든 클라이언트에게 새 접속 알림
            self.broadcast_to_all(json.dumps({
                "type": "new_connection",
                "client_name": name_data,
                "client_id": client_id,
                "timestamp": time.time()
            }), exclude_id=client_id)
            
            # 클라이언트로부터 메시지 수신
            conn.settimeout(None)  # 타임아웃 해제
            while True:
                data = conn.recv(1024)
                if not data:
                    break
                message = data.decode('utf-8').strip()
                print(f"[수신 {name_data}] {message}")
                
                # 메시지 처리
                try:
                    msg_data = json.loads(message)
                    self.process_client_message(client_id, msg_data)
                except json.JSONDecodeError:
                    self.process_client_message(client_id, {"type": "text", "content": message})
                    
        except socket.timeout:
            print(f"[타임아웃] 클라이언트가 이름을 입력하지 않았습니다: {addr}")
        except Exception as e:
            print(f"[에러] {client_id or addr}: {e}")
        finally:
            # 연결 종료 처리
            if client_id:
                with self.lock:
                    client_info = self.clients.pop(client_id, None)
                if client_info:
                    print(f"[해제] {client_info['name']} (ID: {client_id})")
                    # 다른 클라이언트에게 접속 해제 알림
                    self.broadcast_to_all(json.dumps({
                        "type": "disconnection",
                        "client_name": client_info['name'],
                        "client_id": client_id,
                        "timestamp": time.time()
                    }))
                self.show_client_list()
            try:
                conn.close()
            except:
                pass
    
    def process_client_message(self, client_id, msg_data):
        """클라이언트로부터 받은 메시지 처리"""
        try:
            if isinstance(msg_data, dict):
                msg_type = msg_data.get('type', 'unknown')
                content = msg_data.get('content', '')
                
                client_info = self.clients.get(client_id, {})
                client_name = client_info.get('name', 'Unknown')
                
                print(f"📨 {client_name}로부터 메시지: {msg_type} - {content}")
                
                if msg_type == 'dobot_command':
                    self.handle_dobot_command(client_id, content)
                elif msg_type == 'status_request':
                    self.send_status_to_client(client_id)
                elif msg_type == 'list_request':
                    self.send_client_list(client_id)
                    
        except Exception as e:
            print(f"❌ 메시지 처리 오류: {e}")
    
    def handle_dobot_command(self, client_id, command):
        """Dobot 명령 처리"""
        client_info = self.clients.get(client_id, {})
        client_name = client_info.get('name', 'Unknown')
        
        print(f"🤖 {client_name}로부터 Dobot 명령: {command}")
        
        response = {
            "type": "dobot_response",
            "command": command,
            "status": "executed",
            "from": client_name,
            "timestamp": time.time()
        }
        self.send_to_client(client_id, json.dumps(response))
    
    def send_status_to_client(self, client_id):
        """클라이언트에게 상태 정보 전송"""
        with self.lock:
            client_list = [
                {
                    "id": cid,
                    "name": info['name'],
                    "connected_at": info['connected_at']
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
    
    def send_client_list(self, client_id):
        """클라이언트 목록 전송"""
        with self.lock:
            client_list = [
                {"id": cid, "name": info['name']}
                for cid, info in self.clients.items()
            ]
        
        response = {
            "type": "client_list",
            "clients": client_list,
            "total": len(client_list),
            "timestamp": time.time()
        }
        self.send_to_client(client_id, json.dumps(response))
    
    def send_to_client(self, target_id, message):
        """특정 클라이언트에게만 메시지 전송"""
        with self.lock:
            client_info = self.clients.get(target_id)
        
        if client_info:
            try:
                conn = client_info['conn']
                conn.sendall((message + '\n').encode('utf-8'))
                print(f"📤 [발신 {client_info['name']}] {message}")
                return True
            except Exception as e:
                print(f"❌ [오류] '{client_info['name']}' 전송 실패: {e}")
                return False
        else:
            print(f"❌ [오류] ID '{target_id}'를 찾을 수 없습니다.")
            return False
    
    def broadcast_to_all(self, message, exclude_id=None):
        """모든 클라이언트에게 메시지 전송"""
        with self.lock:
            client_list = list(self.clients.items())
        
        sent_count = 0
        for client_id, client_info in client_list:
            if client_id != exclude_id:
                if self.send_to_client(client_id, message):
                    sent_count += 1
        
        print(f"📢 브로드캐스트 완료: {sent_count}/{len(client_list)} 클라이언트")
        return sent_count
    
    def show_client_list(self):
        """현재 접속된 클라이언트 목록 표시"""
        with self.lock:
            client_list = list(self.clients.items())
        
        print("\n" + "="*50)
        print(f"접속자 목록 ({len(client_list)}명)")
        print("="*50)
        if client_list:
            for i, (client_id, info) in enumerate(client_list, 1):
                connected_time = time.time() - info['connected_at']
                print(f"{i}. {info['name']} (ID: {client_id})")
                print(f"   접속 시간: {int(connected_time)}초")
        else:
            print("접속된 클라이언트가 없습니다.")
        print("="*50 + "\n")
    
    def command_loop(self):
        """서버 콘솔에서 명령을 입력받아 처리"""
        print("\n📋 클라이언트 서버 명령어:")
        print("  list - 접속자 목록 표시")
        print("  send <이름> <메시지> - 특정 클라이언트에게 메시지 전송")
        print("  broadcast <메시지> - 모든 클라이언트에게 메시지 전송")
        print("  kick <이름> - 클라이언트 강제 종료")
        print("  status - 서버 상태 표시")
        print("  stop - 클라이언트 서버 중지\n")
        
        while self.running:
            try:
                line = input("클라이언트 서버> ").strip()
                if not line:
                    continue
                    
                parts = line.split(' ', 2)
                command = parts[0].lower()
                
                if command == 'list':
                    self.show_client_list()
                    
                elif command == 'status':
                    print(f"서버 상태: {'실행중' if self.running else '중지'}")
                    print(f"연결된 클라이언트: {len(self.clients)}개")
                    print(f"서버 주소: {self.host}:{self.port}")
                    
                elif command == 'stop':
                    print("클라이언트 서버를 중지합니다.")
                    self.stop_server()
                    break
                    
                elif command == 'broadcast' and len(parts) >= 2:
                    msg = line[10:]  # 'broadcast ' 제거
                    self.broadcast_to_all(json.dumps({
                        "type": "broadcast",
                        "from": "Server",
                        "content": msg,
                        "timestamp": time.time()
                    }))
                    
                elif command == 'send' and len(parts) >= 3:
                    target_name = parts[1]
                    msg = parts[2]
                    
                    # 이름으로 클라이언트 찾기
                    target_id = None
                    with self.lock:
                        for cid, info in self.clients.items():
                            if info['name'] == target_name:
                                target_id = cid
                                break
                    
                    if target_id:
                        self.send_to_client(target_id, json.dumps({
                            "type": "private_message",
                            "from": "Server",
                            "content": msg,
                            "timestamp": time.time()
                        }))
                    else:
                        print(f"❌ '{target_name}' 클라이언트를 찾을 수 없습니다.")
                        
                elif command == 'kick' and len(parts) >= 2:
                    target_name = parts[1]
                    
                    # 이름으로 클라이언트 찾아서 연결 종료
                    with self.lock:
                        for cid, info in list(self.clients.items()):
                            if info['name'] == target_name:
                                try:
                                    info['conn'].close()
                                    print(f"✅ {target_name} 클라이언트를 강제 종료했습니다.")
                                except:
                                    pass
                                break
                        else:
                            print(f"❌ '{target_name}' 클라이언트를 찾을 수 없습니다.")
                        
                else:
                    print("❌ 잘못된 명령어입니다.")
                    
            except EOFError:
                break
            except KeyboardInterrupt:
                break
            except Exception as e:
                print(f"명령 처리 오류: {e}")
    
    def stop_server(self):
        """서버 중지"""
        self.running = False
        if self.server_socket:
            try:
                self.server_socket.close()
            except:
                pass
        
        # 모든 클라이언트에게 서버 종료 알림
        shutdown_msg = json.dumps({
            "type": "server_shutdown",
            "timestamp": time.time()
        })
        
        # 모든 클라이언트 연결 종료
        with self.lock:
            for client_id, info in self.clients.items():
                try:
                    info['conn'].sendall((shutdown_msg + '\n').encode('utf-8'))
                    info['conn'].close()
                except:
                    pass
            self.clients.clear()
        
        print("클라이언트 서버가 중지되었습니다.")


def main():
    """메인 함수"""
    os_type = platform.system()
    print("\n" + "=" * 60)
    print(f"  🤖 Dobot 제어 시스템 시작 ({os_type})")
    print("=" * 60)
    
    # 로깅 설정
    LoggerSetup.setup_logging()
    logger = LoggerSetup.get_logger(__name__)
    
    try:
        # 설정 로드
        dobot_config = ConfigLoader.get_dobot_config()
        http_config = ConfigLoader.get_http_config()
        
        # 시뮬레이션 모드 확인
        simulation_enabled = dobot_config.get('simulation', {}).get('enabled', False)
        
        logger.info(f"Dobot 설정: {dobot_config}")
        logger.info(f"HTTP 서버 설정: {http_config}")
        logger.info(f"시뮬레이션 모드: {simulation_enabled}")
        
        # TCP 서버 생성
        tcp_server = DobotTCPServer(
            host=dobot_config.get('host', '192.168.1.6'),
            port=dobot_config.get('port', 29999),
            simulation_mode=simulation_enabled
        )
        
        # 클라이언트 관리자 생성 및 시작
        client_manager = ClientManager(
            host=dobot_config.get('client_server', {}).get('host', '127.0.0.1'),
            port=dobot_config.get('client_server', {}).get('port', 9988)
        )
        client_manager.start_server()
        
        # HTTP API 서버 생성 및 백그라운드 실행
        if FLASK_WORKING:
            http_server = HTTPAPIServer(
                tcp_server,
                client_manager,
                port=http_config.get('port', 8080)
            )
        else:
            # Flask 없을 때 기본 HTTP 서버
            from http_api_server import DummyHTTPServer
            http_server = DummyHTTPServer(
                tcp_server,
                client_manager,
                port=http_config.get('port', 8080)
            )
        
        http_thread = threading.Thread(target=http_server.run, daemon=True)
        http_thread.start()
        
        # GUI 생성 및 실행
        gui = DobotGUI(tcp_server)
        
        print(f"\n🚀 시스템 구성 요소:")
        print("1. ✅ Dobot TCP 서버")
        if simulation_enabled:
            print("   └─ 🎮 시뮬레이션 모드")
        print(f"2. ✅ 클라이언트 서버 (포트: {client_manager.port})")
        print(f"3. ✅ HTTP API 서버 (포트: {http_config.get('port', 8080)})")
        print("4. ✅ GUI 인터페이스")
        
        print(f"\n🌐 서비스 주소:")
        print(f"- HTTP API: http://localhost:{http_config.get('port', 8080)}")
        print(f"- 클라이언트 서버: {client_manager.host}:{client_manager.port}")
        
        print("\n💡 사용법:")
        print("- GUI에서 '연결' 버튼을 클릭하여 Dobot에 연결")
        print("- 클라이언트는 접속 시 이름을 입력해야 함")
        print("- 'list' 명령으로 접속자 확인")
        print("- 'send <이름> <메시지>'로 개별 메시지 전송")
        
        print("\n🎯 GUI를 시작합니다...")
        
        # GUI 실행
        gui.run()
        
    except KeyboardInterrupt:
        logger.info("사용자에 의해 종료됨")
        print("\n👋 시스템이 종료되었습니다.")
        
        if 'client_manager' in locals():
            client_manager.stop_server()
            
    except Exception as e:
        logger.error(f"시스템 오류: {e}")
        print(f"❌ 오류 발생: {e}")
        import traceback
        traceback.print_exc()
    finally:
        logger.info("시스템 종료")
        
        if 'client_manager' in locals():
            client_manager.stop_server()


if __name__ == "__main__":
    main()
