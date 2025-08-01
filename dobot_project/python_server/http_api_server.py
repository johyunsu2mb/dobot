"""
업데이트된 HTTP API 서버 - 시뮬레이션 기능 및 클라이언트 메시지 지원
"""

import json
import time
import threading
import logging
from http.server import HTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse, parse_qs

# Flask 사용 가능 여부 확인
try:
    import flask
    from flask import Flask, request, jsonify
    try:
        from flask_cors import CORS
        CORS_AVAILABLE = True
    except ImportError:
        CORS_AVAILABLE = False
    FLASK_AVAILABLE = True
except ImportError:
    FLASK_AVAILABLE = False

class HTTPAPIServer:
    """HTTP API 서버 - 시뮬레이션 모드 지원 및 클라이언트 통신 기능"""
    
    def __init__(self, tcp_server, client_manager=None, port=8080):
        self.tcp_server = tcp_server
        self.client_manager = client_manager
        self.port = port
        self.server = None
        self.running = False
        self.logger = logging.getLogger(__name__)
        
        if FLASK_AVAILABLE:
            print(f"🌐 Flask HTTP API 서버 준비됨 (포트: {port})")
        else:
            print(f"🌐 내장 HTTP API 서버 준비됨 (포트: {port})")
    
    def run(self):
        """서버 실행"""
        try:
            if FLASK_AVAILABLE:
                self._run_flask_server()
            else:
                self._run_builtin_server()
        except Exception as e:
            self.logger.error(f"HTTP 서버 실행 오류: {e}")
            print(f"❌ HTTP 서버 실행 실패: {e}")
    
    def _run_flask_server(self):
        """Flask를 사용한 완전한 API 서버"""
        app = Flask(__name__)
        
        # CORS 설정
        if CORS_AVAILABLE:
            CORS(app)
        else:
            @app.after_request
            def after_request(response):
                response.headers.add('Access-Control-Allow-Origin', '*')
                response.headers.add('Access-Control-Allow-Headers', 'Content-Type,Authorization')
                response.headers.add('Access-Control-Allow-Methods', 'GET,PUT,POST,DELETE,OPTIONS')
                return response
        
        @app.route('/')
        def index():
            return self._get_flask_index_html()
        
        @app.route('/status')
        def status():
            try:
                # TCP 서버 상태 가져오기
                if hasattr(self.tcp_server, 'get_status'):
                    dobot_status = self.tcp_server.get_status()
                else:
                    dobot_status = {
                        'connected': getattr(self.tcp_server, 'is_connected', False),
                        'simulation_mode': getattr(self.tcp_server, 'simulation_mode', False)
                    }
                
                # 클라이언트 관리자 상태
                client_info = {}
                if self.client_manager:
                    with self.client_manager.lock:
                        client_info = {
                            'client_count': len(self.client_manager.clients),
                            'client_list': list(self.client_manager.clients.keys()),
                            'server_running': self.client_manager.running
                        }
                
                response_data = {
                    'status': 'success',
                    'timestamp': time.time(),
                    'server_type': 'Flask',
                    'flask_available': True,
                    'cors_available': CORS_AVAILABLE,
                    'dobot': dobot_status,
                    'clients': client_info,
                    'endpoints': [
                        '/status', '/connect', '/disconnect', '/command', 
                        '/position', '/home', '/emergency_stop',
                        '/send_message', '/broadcast', '/clients'
                    ]
                }
                
                return jsonify(response_data)
                
            except Exception as e:
                return jsonify({
                    'status': 'error',
                    'message': str(e),
                    'timestamp': time.time()
                })
        
        @app.route('/connect', methods=['POST'])
        def connect():
            try:
                result = self.tcp_server.connect()
                return jsonify({
                    'status': 'success' if result else 'error',
                    'message': 'Connected to Dobot' if result else 'Failed to connect',
                    'connected': result,
                    'simulation_mode': getattr(self.tcp_server, 'simulation_mode', False)
                })
            except Exception as e:
                return jsonify({
                    'status': 'error',
                    'message': str(e)
                })
        
        @app.route('/disconnect', methods=['POST'])
        def disconnect():
            try:
                self.tcp_server.disconnect()
                return jsonify({
                    'status': 'success',
                    'message': 'Disconnected from Dobot',
                    'connected': False
                })
            except Exception as e:
                return jsonify({
                    'status': 'error',
                    'message': str(e)
                })
        
        @app.route('/command', methods=['POST'])
        def send_command():
            try:
                data = request.get_json()
                if not data or 'command' not in data:
                    return jsonify({
                        'status': 'error',
                        'message': 'Missing command in request body'
                    })
                
                command = data['command']
                response = self.tcp_server.send_command(command)
                
                return jsonify({
                    'status': 'success',
                    'command': command,
                    'response': response,
                    'simulation_mode': getattr(self.tcp_server, 'simulation_mode', False),
                    'timestamp': time.time()
                })
                
            except Exception as e:
                return jsonify({
                    'status': 'error',
                    'message': str(e)
                })
        
        @app.route('/position')
        def get_position():
            try:
                if hasattr(self.tcp_server, 'get_pose'):
                    position_data = self.tcp_server.get_pose()
                else:
                    position_data = None
                
                if position_data is not None:
                    return jsonify({
                        'status': 'success',
                        'position': position_data,
                        'simulation_mode': getattr(self.tcp_server, 'simulation_mode', False),
                        'timestamp': time.time()
                    })
                else:
                    return jsonify({
                        'status': 'error',
                        'message': 'Position query failed'
                    })
            except Exception as e:
                return jsonify({
                    'status': 'error',
                    'message': str(e)
                })
        
        @app.route('/home', methods=['POST'])
        def home():
            try:
                result = self.tcp_server.home()
                return jsonify({
                    'status': 'success',
                    'message': 'Homing command sent',
                    'result': result,
                    'simulation_mode': getattr(self.tcp_server, 'simulation_mode', False)
                })
            except Exception as e:
                return jsonify({
                    'status': 'error',
                    'message': str(e)
                })
        
        @app.route('/emergency_stop', methods=['POST'])
        def emergency_stop():
            try:
                if hasattr(self.tcp_server, 'emergency_stop'):
                    result = self.tcp_server.emergency_stop()
                    return jsonify({
                        'status': 'success',
                        'message': 'Emergency stop executed',
                        'result': result
                    })
                else:
                    return jsonify({
                        'status': 'error',
                        'message': 'Emergency stop not available'
                    })
            except Exception as e:
                return jsonify({
                    'status': 'error',
                    'message': str(e)
                })
        
        # 클라이언트 관리 엔드포인트들
        @app.route('/clients')
        def get_clients():
            try:
                if self.client_manager:
                    with self.client_manager.lock:
                        client_list = list(self.client_manager.clients.keys())
                    
                    return jsonify({
                        'status': 'success',
                        'clients': client_list,
                        'count': len(client_list),
                        'server_running': self.client_manager.running
                    })
                else:
                    return jsonify({
                        'status': 'error',
                        'message': 'Client manager not available'
                    })
            except Exception as e:
                return jsonify({
                    'status': 'error',
                    'message': str(e)
                })
        
        @app.route('/send_message', methods=['POST'])
        def send_message():
            try:
                data = request.get_json()
                client_id = data.get('client_id', '')
                message = data.get('message', '')
                
                if not client_id or not message:
                    return jsonify({
                        'status': 'error',
                        'message': 'client_id and message are required'
                    })
                
                if self.client_manager:
                    success = self.client_manager.send_to_client(client_id, message)
                    return jsonify({
                        'status': 'success' if success else 'error',
                        'message': f'Message {"sent" if success else "failed"}',
                        'client_id': client_id,
                        'sent_message': message
                    })
                else:
                    return jsonify({
                        'status': 'error',
                        'message': 'Client manager not available'
                    })
                    
            except Exception as e:
                return jsonify({
                    'status': 'error',
                    'message': str(e)
                })
        
        @app.route('/broadcast', methods=['POST'])
        def broadcast():
            try:
                data = request.get_json()
                message = data.get('message', '')
                
                if not message:
                    return jsonify({
                        'status': 'error',
                        'message': 'message is required'
                    })
                
                if self.client_manager:
                    sent_count = self.client_manager.broadcast_to_all(message)
                    return jsonify({
                        'status': 'success',
                        'message': f'Broadcast sent to {sent_count} clients',
                        'sent_count': sent_count,
                        'broadcast_message': message
                    })
                else:
                    return jsonify({
                        'status': 'error',
                        'message': 'Client manager not available'
                    })
                    
            except Exception as e:
                return jsonify({
                    'status': 'error',
                    'message': str(e)
                })
        
        # 시뮬레이션 전용 엔드포인트들
        @app.route('/simulation/status')
        def simulation_status():
            try:
                if hasattr(self.tcp_server, 'simulation_mode') and self.tcp_server.simulation_mode:
                    if hasattr(self.tcp_server, 'simulator') and self.tcp_server.simulator:
                        sim_status = self.tcp_server.simulator.get_status()
                        return jsonify({
                            'status': 'success',
                            'simulation': sim_status
                        })
                
                return jsonify({
                    'status': 'error',
                    'message': 'Simulation mode not active'
                })
            except Exception as e:
                return jsonify({
                    'status': 'error',
                    'message': str(e)
                })
        
        @app.route('/simulation/reset', methods=['POST'])
        def simulation_reset():
            try:
                if hasattr(self.tcp_server, 'simulation_mode') and self.tcp_server.simulation_mode:
                    result = self.tcp_server.send_command("Reset()")
                    return jsonify({
                        'status': 'success',
                        'message': 'Simulation reset',
                        'result': result
                    })
                
                return jsonify({
                    'status': 'error',
                    'message': 'Simulation mode not active'
                })
            except Exception as e:
                return jsonify({
                    'status': 'error',
                    'message': str(e)
                })
        
        # OPTIONS 메서드 처리 (CORS용)
        @app.route('/<path:path>', methods=['OPTIONS'])
        def handle_options(path):
            return '', 200
        
        try:
            print(f"🌟 Flask HTTP API 서버 시작: http://localhost:{self.port}")
            if getattr(self.tcp_server, 'simulation_mode', False):
                print("🎮 시뮬레이션 모드 API 엔드포인트 활성화됨")
            self.running = True
            app.run(host='0.0.0.0', port=self.port, debug=False, threaded=True)
        except Exception as e:
            print(f"❌ Flask 서버 오류: {e}")
            self.running = False
    
    def _run_builtin_server(self):
        """내장 HTTP 서버를 사용한 기본 API (Flask 없을 때)"""
        class APIRequestHandler(BaseHTTPRequestHandler):
            def __init__(self, *args, tcp_server=None, client_manager=None, **kwargs):
                self.tcp_server = tcp_server
                self.client_manager = client_manager
                super().__init__(*args, **kwargs)
            
            def _send_json_response(self, data, status_code=200):
                self.send_response(status_code)
                self.send_header('Content-type', 'application/json')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
                self.send_header('Access-Control-Allow-Headers', 'Content-Type')
                self.end_headers()
                self.wfile.write(json.dumps(data, indent=2, ensure_ascii=False).encode('utf-8'))
            
            def _send_html_response(self, html):
                self.send_response(200)
                self.send_header('Content-type', 'text/html; charset=utf-8')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(html.encode('utf-8'))
            
            def do_OPTIONS(self):
                self.send_response(200)
                self.send_header('Access-Control-Allow-Origin', '*')
                self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
                self.send_header('Access-Control-Allow-Headers', 'Content-Type')
                self.end_headers()
            
            def do_GET(self):
                if self.path == '/':
                    self._send_html_response(self._get_builtin_index_html())
                else:
                    self._send_json_response({'error': 'Not found'}, 404)
            
            def do_POST(self):
                self._send_json_response({'error': 'Built-in server has limited functionality'}, 501)
            
            def _get_builtin_index_html(self):
                return """
                <!DOCTYPE html>
                <html>
                <head>
                    <title>🤖 Dobot API Server (내장)</title>
                    <meta charset="utf-8">
                    <style>
                        body { font-family: Arial, sans-serif; margin: 40px; background: #f5f5f5; }
                        .container { max-width: 800px; margin: 0 auto; background: white; padding: 30px; border-radius: 10px; }
                        .warning { background-color: #fff3cd; color: #856404; padding: 15px; border-radius: 5px; margin: 15px 0; }
                    </style>
                </head>
                <body>
                    <div class="container">
                        <h1>🤖 Dobot API Server</h1>
                        <div class="warning">
                            ⚠️ 완전한 기능을 위해 Flask를 설치하세요: pip install flask flask-cors
                        </div>
                    </div>
                </body>
                </html>
                """
            
            def log_message(self, format, *args):
                pass
        
        def handler_factory(*args, **kwargs):
            return APIRequestHandler(*args, tcp_server=self.tcp_server, client_manager=self.client_manager, **kwargs)
        
        try:
            self.server = HTTPServer(("", self.port), handler_factory)
            print(f"🌐 내장 HTTP 서버 시작: http://localhost:{self.port}")
            print("💡 Flask 설치 후 재시작하면 완전한 기능을 사용할 수 있습니다")
            self.running = True
            self.server.serve_forever()
        except Exception as e:
            print(f"❌ HTTP 서버 오류: {e}")
            self.running = False
    
    def _get_flask_index_html(self):
        """Flask용 업데이트된 HTML 페이지"""
        simulation_mode = getattr(self.tcp_server, 'simulation_mode', False)
        
        return f"""
        <!DOCTYPE html>
        <html>
        <head>
            <title>🤖 Dobot API Server {'(시뮬레이션)' if simulation_mode else ''}</title>
            <meta charset="utf-8">
            <style>
                body {{ font-family: Arial, sans-serif; margin: 40px; background: #f5f5f5; }}
                .container {{ max-width: 1200px; margin: 0 auto; background: white; padding: 30px; border-radius: 10px; box-shadow: 0 2px 10px rgba(0,0,0,0.1); }}
                .status {{ padding: 15px; border-radius: 5px; margin: 15px 0; }}
                .success {{ background-color: #d4edda; color: #155724; border: 1px solid #c3e6cb; }}
                .info {{ background-color: #d1ecf1; color: #0c5460; border: 1px solid #bee5eb; }}
                .simulation {{ background-color: #fff3cd; color: #856404; border: 1px solid #ffeaa7; }}
                .endpoint {{ background-color: #f8f9fa; padding: 15px; margin: 10px 0; border-left: 4px solid #007bff; border-radius: 5px; }}
                .simulation-endpoint {{ border-left-color: #ffc107; }}
                button {{ padding: 12px 24px; margin: 8px; background: #007bff; color: white; border: none; border-radius: 5px; cursor: pointer; font-size: 14px; }}
                button:hover {{ background: #0056b3; }}
                button.success {{ background: #28a745; }}
                button.success:hover {{ background: #218838; }}
                button.danger {{ background: #dc3545; }}
                button.danger:hover {{ background: #c82333; }}
                button.warning {{ background: #ffc107; color: #212529; }}
                button.warning:hover {{ background: #e0a800; }}
                button.simulation {{ background: #6f42c1; }}
                button.simulation:hover {{ background: #5a359a; }}
                #status-info {{ background: #f8f9fa; padding: 15px; border-radius: 5px; border: 1px solid #dee2e6; }}
                #response-log {{ background: #f8f9fa; padding: 20px; border-radius: 5px; max-height: 400px; overflow-y: auto; font-family: monospace; font-size: 12px; border: 1px solid #dee2e6; }}
                .section {{ margin: 30px 0; }}
                .grid {{ display: grid; grid-template-columns: 1fr 1fr; gap: 20px; }}
                .grid-3 {{ display: grid; grid-template-columns: 1fr 1fr 1fr; gap: 15px; }}
                h1 {{ color: #343a40; }}
                h2 {{ color: #495057; border-bottom: 2px solid #dee2e6; padding-bottom: 10px; }}
                .command-input {{ width: 100%; padding: 10px; margin: 10px 0; border: 1px solid #dee2e6; border-radius: 5px; }}
                .client-input {{ width: 200px; padding: 8px; margin: 5px; border: 1px solid #dee2e6; border-radius: 5px; }}
                .client-section {{ background: #f8f9fa; padding: 20px; border-radius: 10px; margin: 20px 0; }}
            </style>
        </head>
        <body>
            <div class="container">
                <h1>🤖 Dobot API Server {'🎮' if simulation_mode else ''}</h1>
                
                <div class="status success">
                    ✅ Flask 기반 완전한 API 서버가 실행 중입니다
                </div>
                
                {f'<div class="status simulation">🎮 <strong>시뮬레이션 모드</strong> - 실제 하드웨어 없이 완전한 기능 테스트 가능</div>' if simulation_mode else ''}
                
                <div class="section">
                    <h2>📡 시스템 상태</h2>
                    <div id="status-info">상태 확인 중...</div>
                    <button onclick="checkStatus()">🔄 상태 새로고침</button>
                    {f'<button onclick="checkSimulationStatus()" class="simulation">🎮 시뮬레이션 상세</button>' if simulation_mode else ''}
                </div>
                
                <div class="section">
                    <h2>🛠️ Dobot 제어 API</h2>
                    <div class="grid">
                        <div>
                            <div class="endpoint">
                                <strong>POST /connect</strong> - Dobot 연결
                            </div>
                            <div class="endpoint">
                                <strong>POST /disconnect</strong> - 연결 해제
                            </div>
                            <div class="endpoint">
                                <strong>POST /command</strong> - 명령 전송
                            </div>
                        </div>
                        <div>
                            <div class="endpoint">
                                <strong>GET /position</strong> - 현재 위치 조회
                            </div>
                            <div class="endpoint">
                                <strong>POST /home</strong> - 홈 위치로 이동
                            </div>
                            <div class="endpoint">
                                <strong>POST /emergency_stop</strong> - 비상정지
                            </div>
                        </div>
                    </div>
                </div>
                
                {f'''
                <div class="section">
                    <h2>🎮 시뮬레이션 전용 API</h2>
                    <div class="grid">
                        <div class="endpoint simulation-endpoint">
                            <strong>GET /simulation/status</strong> - 시뮬레이션 상세 상태
                        </div>
                        <div class="endpoint simulation-endpoint">
                            <strong>POST /simulation/reset</strong> - 시뮬레이션 리셋
                        </div>
                    </div>
                </div>
                ''' if simulation_mode else ''}
                
                <div class="section">
                    <h2>📱 클라이언트 통신 API</h2>
                    <div class="grid">
                        <div class="endpoint">
                            <strong>GET /clients</strong> - 연결된 클라이언트 목록
                        </div>
                        <div class="endpoint">
                            <strong>POST /send_message</strong> - 특정 클라이언트에게 메시지
                        </div>
                        <div class="endpoint">
                            <strong>POST /broadcast</strong> - 모든 클라이언트에게 브로드캐스트
                        </div>
                    </div>
                </div>
                
                <div class="section">
                    <h2>🧪 Dobot 제어 테스트</h2>
                    <div class="grid-3">
                        <button onclick="testConnect()" class="success">🔗 연결</button>
                        <button onclick="testPosition()">📍 위치 조회</button>
                        <button onclick="testHome()" class="warning">🏠 홈으로</button>
                        <button onclick="testEmergencyStop()" class="danger">🛑 비상정지</button>
                        <button onclick="testDisconnect()" class="danger">❌연결 해제</button>
                        {f'<button onclick="resetSimulation()" class="simulation">🔄 시뮬 리셋</button>' if simulation_mode else ''}
                    </div>
                    
                    <div style="margin-top: 20px;">
                        <h3>📝 사용자 정의 명령</h3>
                        <input type="text" id="custom-command" class="command-input" placeholder="명령을 입력하세요 (예: MovJ(250,100,200,45))" />
                        <button onclick="sendCustomCommand()">📤 명령 전송</button>
                    </div>
                </div>
                
                <div class="client-section">
                    <h3>📱 클라이언트 통신 테스트</h3>
                    <div id="client-list">클라이언트 목록 로딩 중...</div>
                    <div style="margin: 15px 0;">
                        <input type="text" id="target-client" class="client-input" placeholder="클라이언트 ID" />
                        <input type="text" id="message-text" class="client-input" placeholder="메시지" />
                        <button onclick="sendMessage()">📤 개별 전송</button>
                    </div>
                    <div>
                        <input type="text" id="broadcast-text" class="client-input" placeholder="브로드캐스트 메시지" />
                        <button onclick="broadcastMessage()">📢 전체 전송</button>
                        <button onclick="refreshClients()">🔄 목록 새로고침</button>
                    </div>
                </div>
                
                <div class="section">
                    <h3>📋 응답 로그</h3>
                    <pre id="response-log">로그가 여기에 표시됩니다...\n</pre>
                    <button onclick="clearLog()">🗑️ 로그 지우기</button>
                    <button onclick="downloadLog()">💾 로그 다운로드</button>
                </div>
            </div>
            
            <script>
                function logResponse(endpoint, response) {{
                    const log = document.getElementById('response-log');
                    const timestamp = new Date().toLocaleTimeString();
                    log.textContent += `[${{timestamp}}] ${{endpoint}}:\n${{JSON.stringify(response, null, 2)}}\n\n`;
                    log.scrollTop = log.scrollHeight;
                }}
                
                async function checkStatus() {{
                    try {{
                        const response = await fetch('/status');
                        const data = await response.json();
                        
                        let statusHtml = `<strong>시스템:</strong> ${{data.status}}<br>`;
                        if (data.dobot) {{
                            statusHtml += `<strong>로봇 연결:</strong> ${{data.dobot.connected ? '🟢 연결됨' : '🔴 연결 안됨'}}<br>`;
                            statusHtml += `<strong>모드:</strong> ${{data.dobot.simulation_mode ? '🎮 시뮬레이션' : '🤖 실제 로봇'}}<br>`;
                            if (data.dobot.current_pose) {{
                                const pose = data.dobot.current_pose;
                                statusHtml += `<strong>현재 위치:</strong> X:${{pose[0]?.toFixed(1)}} Y:${{pose[1]?.toFixed(1)}} Z:${{pose[2]?.toFixed(1)}} R:${{pose[3]?.toFixed(1)}}<br>`;
                            }}
                        }}
                        if (data.clients) {{
                            statusHtml += `<strong>연결된 클라이언트:</strong> ${{data.clients.client_count}}개<br>`;
                        }}
                        statusHtml += `<strong>마지막 업데이트:</strong> ${{new Date(data.timestamp * 1000).toLocaleString()}}`;
                        
                        document.getElementById('status-info').innerHTML = statusHtml;
                        logResponse('GET /status', data);
                    }} catch (error) {{
                        document.getElementById('status-info').innerHTML = '❌ 상태 확인 실패';
                        logResponse('GET /status', {{ error: error.message }});
                    }}
                }}
                
                async function checkSimulationStatus() {{
                    try {{
                        const response = await fetch('/simulation/status');
                        const data = await response.json();
                        logResponse('GET /simulation/status', data);
                    }} catch (error) {{
                        logResponse('GET /simulation/status', {{ error: error.message }});
                    }}
                }}
                
                async function testConnect() {{
                    try {{
                        const response = await fetch('/connect', {{ method: 'POST' }});
                        const data = await response.json();
                        logResponse('POST /connect', data);
                        checkStatus();
                    }} catch (error) {{
                        logResponse('POST /connect', {{ error: error.message }});
                    }}
                }}
                
                async function testPosition() {{
                    try {{
                        const response = await fetch('/position');
                        const data = await response.json();
                        logResponse('GET /position', data);
                    }} catch (error) {{
                        logResponse('GET /position', {{ error: error.message }});
                    }}
                }}
                
                async function testHome() {{
                    try {{
                        const response = await fetch('/home', {{ method: 'POST' }});
                        const data = await response.json();
                        logResponse('POST /home', data);
                        checkStatus();
                    }} catch (error) {{
                        logResponse('POST /home', {{ error: error.message }});
                    }}
                }}
                
                async function testEmergencyStop() {{
                    try {{
                        const response = await fetch('/emergency_stop', {{ method: 'POST' }});
                        const data = await response.json();
                        logResponse('POST /emergency_stop', data);
                        checkStatus();
                    }} catch (error) {{
                        logResponse('POST /emergency_stop', {{ error: error.message }});
                    }}
                }}
                
                async function testDisconnect() {{
                    try {{
                        const response = await fetch('/disconnect', {{ method: 'POST' }});
                        const data = await response.json();
                        logResponse('POST /disconnect', data);
                        checkStatus();
                    }} catch (error) {{
                        logResponse('POST /disconnect', {{ error: error.message }});
                    }}
                }}
                
                async function resetSimulation() {{
                    try {{
                        const response = await fetch('/simulation/reset', {{ method: 'POST' }});
                        const data = await response.json();
                        logResponse('POST /simulation/reset', data);
                        checkStatus();
                    }} catch (error) {{
                        logResponse('POST /simulation/reset', {{ error: error.message }});
                    }}
                }}
                
                async function sendCustomCommand() {{
                    const command = document.getElementById('custom-command').value;
                    if (!command) {{
                        alert('명령을 입력해주세요.');
                        return;
                    }}
                    
                    try {{
                        const response = await fetch('/command', {{
                            method: 'POST',
                            headers: {{ 'Content-Type': 'application/json' }},
                            body: JSON.stringify({{ command: command }})
                        }});
                        const data = await response.json();
                        logResponse('POST /command', data);
                        document.getElementById('custom-command').value = '';
                    }} catch (error) {{
                        logResponse('POST /command', {{ error: error.message }});
                    }}
                }}
                
                async function refreshClients() {{
                    try {{
                        const response = await fetch('/clients');
                        const data = await response.json();
                        
                        if (data.status === 'success') {{
                            let clientHtml = `<strong>연결된 클라이언트:</strong> ${{data.count}}개<br>`;
                            if (data.clients.length > 0) {{
                                clientHtml += '<ul>';
                                data.clients.forEach(client => {{
                                    clientHtml += `<li>${{client}}</li>`;
                                }});
                                clientHtml += '</ul>';
                            }} else {{
                                clientHtml += '<em>연결된 클라이언트가 없습니다.</em>';
                            }}
                            document.getElementById('client-list').innerHTML = clientHtml;
                        }} else {{
                            document.getElementById('client-list').innerHTML = `<em>오류: ${{data.message}}</em>`;
                        }}
                        
                        logResponse('GET /clients', data);
                    }} catch (error) {{
                        document.getElementById('client-list').innerHTML = '<em>클라이언트 목록 조회 실패</em>';
                        logResponse('GET /clients', {{ error: error.message }});
                    }}
                }}
                
                async function sendMessage() {{
                    const clientId = document.getElementById('target-client').value;
                    const message = document.getElementById('message-text').value;
                    
                    if (!clientId || !message) {{
                        alert('클라이언트 ID와 메시지를 모두 입력해주세요.');
                        return;
                    }}
                    
                    try {{
                        const response = await fetch('/send_message', {{
                            method: 'POST',
                            headers: {{ 'Content-Type': 'application/json' }},
                            body: JSON.stringify({{ client_id: clientId, message: message }})
                        }});
                        const data = await response.json();
                        logResponse('POST /send_message', data);
                        
                        if (data.status === 'success') {{
                            document.getElementById('target-client').value = '';
                            document.getElementById('message-text').value = '';
                        }}
                    }} catch (error) {{
                        logResponse('POST /send_message', {{ error: error.message }});
                    }}
                }}
                
                async function broadcastMessage() {{
                    const message = document.getElementById('broadcast-text').value;
                    
                    if (!message) {{
                        alert('브로드캐스트 메시지를 입력해주세요.');
                        return;
                    }}
                    
                    try {{
                        const response = await fetch('/broadcast', {{
                            method: 'POST',
                            headers: {{ 'Content-Type': 'application/json' }},
                            body: JSON.stringify({{ message: message }})
                        }});
                        const data = await response.json();
                        logResponse('POST /broadcast', data);
                        
                        if (data.status === 'success') {{
                            document.getElementById('broadcast-text').value = '';
                        }}
                    }} catch (error) {{
                        logResponse('POST /broadcast', {{ error: error.message }});
                    }}
                }}
                
                function clearLog() {{
                    document.getElementById('response-log').textContent = '로그가 여기에 표시됩니다...\n';
                }}
                
                function downloadLog() {{
                    const log = document.getElementById('response-log').textContent;
                    const blob = new Blob([log], {{ type: 'text/plain' }});
                    const url = window.URL.createObjectURL(blob);
                    const a = document.createElement('a');
                    a.href = url;
                    a.download = `dobot_api_log_${{new Date().toISOString().slice(0, 19).replace(/:/g, '-')}}.txt`;
                    a.click();
                    window.URL.revokeObjectURL(url);
                }}
                
                // Enter 키로 명령 전송
                document.getElementById('custom-command').addEventListener('keypress', function(e) {{
                    if (e.key === 'Enter') {{
                        sendCustomCommand();
                    }}
                }});
                
                // Enter 키로 메시지 전송
                document.getElementById('message-text').addEventListener('keypress', function(e) {{
                    if (e.key === 'Enter') {{
                        sendMessage();
                    }}
                }});
                
                document.getElementById('broadcast-text').addEventListener('keypress', function(e) {{
                    if (e.key === 'Enter') {{
                        broadcastMessage();
                    }}
                }});
                
                // 페이지 로드 시 상태 확인
                checkStatus();
                refreshClients();
                
                // 5초마다 자동 상태 업데이트
                setInterval(checkStatus, 5000);
                setInterval(refreshClients, 10000);
            </script>
        </body>
        </html>
        """
    
    def stop(self):
        """서버 중지"""
        self.running = False
        if self.server:
            self.server.shutdown()
            print("🛑 HTTP 서버가 중지되었습니다")

# 이전 버전과의 호환성을 위한 별칭
class DummyHTTPServer:
    def __init__(self, tcp_server, port=8080):
        print("⚠️ DummyHTTPServer는 더 이상 사용되지 않습니다. HTTPAPIServer를 사용하세요.")
        self.server = HTTPAPIServer(tcp_server, port)
    
    def run(self):
        return self.server.run()

if __name__ == "__main__":
    # 테스트용 실행
    print("🧪 HTTP API 서버 테스트 모드")
    
    class MockTCPServer:
        def __init__(self):
            self.is_connected = False
            self.simulation_mode = True
        
        def connect(self):
            self.is_connected = True
            return True
        
        def disconnect(self):
            self.is_connected = False
        
        def send_command(self, command):
            return f"Mock response for: {command}"
        
        def get_pose(self):
            return [100.0, 200.0, 300.0, 0.0]
        
        def home(self):
            return "Homing completed"
        
        def get_status(self):
            return {
                'connected': self.is_connected,
                'simulation_mode': self.simulation_mode,
                'current_pose': self.get_pose()
            }
    
    class MockClientManager:
        def __init__(self):
            self.clients = {'client_1': None, 'client_2': None}
            self.running = True
            import threading
            self.lock = threading.Lock()
        
        def send_to_client(self, client_id, message):
            return client_id in self.clients
        
        def broadcast_to_all(self, message):
            return len(self.clients)
    
    mock_server = MockTCPServer()
    mock_client_manager = MockClientManager()
    api_server = HTTPAPIServer(mock_server, mock_client_manager, 8080)
    
    try:
        print("테스트 서버를 시작합니다...")
        api_server.run()
    except KeyboardInterrupt:
        print("\n테스트 서버를 종료합니다.")
        api_server.stop()