"""
간단한 Dobot GUI (Windows)
"""

import tkinter as tk
from tkinter import ttk, messagebox
import threading
import time

class DobotGUI:
    def __init__(self, tcp_server):
        self.tcp_server = tcp_server
        self.tcp_server.gui = self
        
        self.root = tk.Tk()
        self.root.title("Dobot Control System (Windows)")
        self.root.geometry("600x500")
        
        self.setup_gui()
        print("🖥️ GUI 초기화 완료")
    
    def setup_gui(self):
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # 연결 상태
        conn_frame = ttk.LabelFrame(main_frame, text="연결 상태", padding="10")
        conn_frame.grid(row=0, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=5)
        
        self.status_label = ttk.Label(conn_frame, text="❌ 연결 안됨", foreground="red")
        self.status_label.grid(row=0, column=0, padx=5)
        
        ttk.Button(conn_frame, text="연결", command=self.connect_dobot).grid(row=0, column=1, padx=5)
        ttk.Button(conn_frame, text="해제", command=self.disconnect_dobot).grid(row=0, column=2, padx=5)
        
        # 위치 제어
        pos_frame = ttk.LabelFrame(main_frame, text="위치 제어", padding="10")
        pos_frame.grid(row=1, column=0, sticky=(tk.W, tk.E), pady=5)
        
        # 좌표 입력
        coords = [("X:", "200"), ("Y:", "0"), ("Z:", "100"), ("R:", "0")]
        self.entries = {}
        
        for i, (label, default) in enumerate(coords):
            ttk.Label(pos_frame, text=label).grid(row=i//2, column=(i%2)*2, padx=5, pady=2)
            entry = ttk.Entry(pos_frame, width=10)
            entry.grid(row=i//2, column=(i%2)*2+1, padx=5, pady=2)
            entry.insert(0, default)
            self.entries[label[0].lower()] = entry
        
        ttk.Button(pos_frame, text="이동", command=self.move_to_position).grid(row=2, column=0, columnspan=4, pady=10)
        
        # 엔드이펙터
        ef_frame = ttk.LabelFrame(main_frame, text="엔드이펙터", padding="10")
        ef_frame.grid(row=1, column=1, sticky=(tk.W, tk.E), pady=5)
        
        ttk.Button(ef_frame, text="석션 ON", command=lambda: self.set_suction(True)).pack(pady=2)
        ttk.Button(ef_frame, text="석션 OFF", command=lambda: self.set_suction(False)).pack(pady=2)
        ttk.Button(ef_frame, text="비상정지", command=self.emergency_stop).pack(pady=2)
        
        # 로그
        log_frame = ttk.LabelFrame(main_frame, text="로그", padding="5")
        log_frame.grid(row=2, column=0, columnspan=2, sticky=(tk.W, tk.E, tk.N, tk.S), pady=5)
        
        self.log_text = tk.Text(log_frame, height=10, width=60)
        scrollbar = ttk.Scrollbar(log_frame, orient="vertical", command=self.log_text.yview)
        self.log_text.configure(yscrollcommand=scrollbar.set)
        
        self.log_text.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        scrollbar.grid(row=0, column=1, sticky=(tk.N, tk.S))
        
        # 그리드 설정
        main_frame.columnconfigure(0, weight=1)
        main_frame.columnconfigure(1, weight=1)
        main_frame.rowconfigure(2, weight=1)
        log_frame.columnconfigure(0, weight=1)
        log_frame.rowconfigure(0, weight=1)
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
    
    def connect_dobot(self):
        def connect_thread():
            success = self.tcp_server.connect_to_dobot()
            self.root.after(0, self._update_connection_status, success)
        threading.Thread(target=connect_thread, daemon=True).start()
    
    def _update_connection_status(self, success):
        if success:
            self.status_label.config(text="✅ 연결됨", foreground="green")
            self.log_message("연결 성공!")
        else:
            messagebox.showerror("연결 오류", "Dobot 연결에 실패했습니다.\n\nIP 주소를 확인하세요.")
    
    def disconnect_dobot(self):
        self.tcp_server.disconnect_from_dobot()
        self.status_label.config(text="❌ 연결 안됨", foreground="red")
        self.log_message("연결 해제됨")
    
    def move_to_position(self):
        try:
            x = float(self.entries['x'].get())
            y = float(self.entries['y'].get())
            z = float(self.entries['z'].get())
            r = float(self.entries['r'].get())
            
            if self.tcp_server.move_to_position(x, y, z, r):
                self.log_message(f"이동 명령: ({x}, {y}, {z}, {r})")
            else:
                self.log_message("이동 명령 실패")
        except ValueError:
            messagebox.showerror("입력 오류", "숫자를 입력하세요.")
    
    def set_suction(self, enable):
        if self.tcp_server.set_end_effector_suction(enable):
            self.log_message(f"석션 {'ON' if enable else 'OFF'}")
        else:
            self.log_message("석션 제어 실패")
    
    def emergency_stop(self):
        if messagebox.askyesno("확인", "비상정지 하시겠습니까?"):
            if self.tcp_server.emergency_stop():
                self.log_message("비상정지 실행됨")
            else:
                self.log_message("비상정지 실패")
    
    def log_message(self, message):
        def add_log():
            timestamp = time.strftime("%H:%M:%S")
            self.log_text.insert(tk.END, f"[{timestamp}] {message}\n")
            self.log_text.see(tk.END)
        
        if threading.current_thread() == threading.main_thread():
            add_log()
        else:
            self.root.after(0, add_log)
    
    def update_status(self, status):
        self.log_message(status)
    
    def run(self):
        self.log_message("Dobot 제어 시스템 시작됨")
        self.log_message("IP 주소를 확인하고 연결 버튼을 클릭하세요")
        try:
            self.root.mainloop()
        except KeyboardInterrupt:
            print("GUI 종료됨")
        finally:
            if self.tcp_server.is_connected:
                self.tcp_server.disconnect_from_dobot()
    
    # 추가 메서드들 (호환성)
    def move_joint(self): return self.move_to_position()
    def move_linear(self): return self.move_to_position()
    def set_gripper(self, enable): return self.set_suction(enable)
    def get_current_position(self): self.log_message("위치 조회 요청")
    def go_to_preset(self, pos): pass
    def enable_robot(self): self.log_message("로봇 활성화")
    def disable_robot(self): self.log_message("로봇 비활성화")
    def clear_alarms(self): self.log_message("알람 해제")
    def clear_log(self): self.log_text.delete(1.0, tk.END)
    def save_log(self): self.log_message("로그 저장 기능 준비 중")
    def update_current_position(self, pos): pass
