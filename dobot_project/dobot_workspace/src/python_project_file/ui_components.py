"""
ui_components.py - UI 컴포넌트들
Enhanced Dobot Robot & YOLO Object Detection System
"""

import tkinter as tk
import logging
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from mpl_toolkits.mplot3d import Axes3D
from config import ROBOT_ARM_CONFIG, FURNITURE_INFO

# matplotlib 한글 폰트 설정
plt.rcParams['font.family'] = 'NanumGothic'
plt.rcParams['axes.unicode_minus'] = False

class RobotArmVisualizer:
    """향상된 로봇팔 3D 시각화 클래스"""
    def __init__(self, parent_frame):
        self.parent_frame = parent_frame
        self.fig = plt.figure(figsize=(8, 6))
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.canvas = FigureCanvasTkAgg(self.fig, parent_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # 안전한 로거 초기화
        try:
            self.logger = logging.getLogger('robot_system.visualizer')
        except:
            self.logger = None
       
        # 로봇팔 설정
        self.link_lengths = ROBOT_ARM_CONFIG['link_lengths']
        self.joint_angles = [0, 0, 0, 0, 0]
        self.gripper_state = "open"
       
        # 가구 위치 설정
        self.furniture_positions = {name: info['position'] for name, info in FURNITURE_INFO.items()}
        self.furniture_available = {name: True for name in self.furniture_positions}
        self.base_position = ROBOT_ARM_CONFIG['base_position']
        self.final_position = ROBOT_ARM_CONFIG['final_position']
        
        self.setup_workspace()
        self.update_visualization()
       
    def setup_workspace(self):
        """작업 공간 설정"""
        self.ax.set_xlim([-400, 400])
        self.ax.set_ylim([-400, 400])
        self.ax.set_zlim([-200, 200])
        self.ax.set_xlabel('X (mm)')
        self.ax.set_ylabel('Y (mm)')
        self.ax.set_zlabel('Z (mm)')
        self.ax.set_title('Enhanced Dobot Robot Arm Control System')
       
    def forward_kinematics(self, angles):
        """순기구학 계산"""
        positions = [[0, 0, 0]]
       
        x = (self.link_lengths[1] * np.cos(angles[0]) * np.cos(angles[1]) +
             self.link_lengths[2] * np.cos(angles[0]) * np.cos(angles[1] + angles[2]))
        y = (self.link_lengths[1] * np.sin(angles[0]) * np.cos(angles[1]) +
             self.link_lengths[2] * np.sin(angles[0]) * np.cos(angles[1] + angles[2]))
        z = (self.link_lengths[0] +
             self.link_lengths[1] * np.sin(angles[1]) +
             self.link_lengths[2] * np.sin(angles[1] + angles[2]))
       
        positions.append([x, y, z])
        return np.array(positions)

    def update_visualization(self):
        """시각화 업데이트"""
        self.ax.clear()
        self.setup_workspace()
       
        # 로봇팔 그리기
        positions = self.forward_kinematics(self.joint_angles)
       
        for i in range(len(positions) - 1):
            self.ax.plot3D([positions[i][0], positions[i+1][0]],
                          [positions[i][1], positions[i+1][1]],
                          [positions[i][2], positions[i+1][2]],
                          'r-', linewidth=4, alpha=0.8)
       
        for i, pos in enumerate(positions):
            size = 120 if i == 0 else 80
            self.ax.scatter(pos[0], pos[1], pos[2], c='orange', s=size, alpha=0.9)
       
        end_pos = positions[-1]
        color = 'green' if self.gripper_state == 'closed' else 'red'
        self.ax.scatter(end_pos[0], end_pos[1], end_pos[2], c=color, s=200, alpha=0.9)
       
        # 가구들 표시
        furniture_colors = {"소파": 'red', "의자": 'blue', "책상": 'green', "침대": 'purple'}
        furniture_shapes = {"소파": 's', "의자": 'o', "책상": '^', "침대": 'D'}
       
        for furniture, available in self.furniture_available.items():
            if available:
                pos = self.furniture_positions[furniture]
                self.ax.scatter(pos[0], pos[1], pos[2],
                              c=furniture_colors[furniture],
                              s=400,
                              marker=furniture_shapes[furniture],
                              alpha=0.8,
                              label=furniture,
                              edgecolors='black',
                              linewidth=2)
       
        # 베이스 및 최종 위치 표시
        base_pos = self.base_position
        self.ax.scatter(base_pos[0], base_pos[1], base_pos[2],
                       c='blue', s=600, marker='h', alpha=0.8,
                       label='Base', edgecolors='black', linewidth=3)
        
        final_pos = self.final_position
        self.ax.scatter(final_pos[0], final_pos[1], final_pos[2],
                       c='gold', s=600, marker='*', alpha=0.8,
                       label='Final Position', edgecolors='black', linewidth=3)
       
        self.ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left', fontsize=8)
        self.ax.view_init(elev=25, azim=45)
        self.canvas.draw()
   
    def reset_simulation(self):
        """시뮬레이션 리셋 (안전한 로깅)"""
        self.joint_angles = [0, 0, 0, 0, 0]
        self.gripper_state = "open"
        
        for furniture in self.furniture_available:
            self.furniture_available[furniture] = True
            
        self.update_visualization()
        try:
            if self.logger:
                self.logger.info("Simulation has been reset.")
        except:
            print("Simulation has been reset.")

class CameraDisplay:
    """카메라 디스플레이 컴포넌트"""
    def __init__(self, parent_frame):
        self.parent_frame = parent_frame
        self.camera_label = None
        self.setup_camera_display()
    
    def setup_camera_display(self):
        """카메라 디스플레이 설정"""
        self.camera_label = tk.Label(
            self.parent_frame,
            text="Camera OFF\nPress [CAMERA] button to activate YOLO",
            font=("Arial", 11),
            bg='#2c3e50',
            fg='#95a5a6',
            relief='sunken',
            bd=2
        )
        self.camera_label.pack(fill=tk.BOTH, expand=True)
    
    def update_frame(self, photo):
        """프레임 업데이트"""
        if self.camera_label:
            self.camera_label.config(image=photo, text="")
            self.camera_label.image = photo  # 참조 유지
    
    def show_off_message(self):
        """카메라 OFF 메시지 표시"""
        if self.camera_label:
            self.camera_label.config(
                image="",
                text="Camera OFF\nPress [CAMERA] button to activate YOLO"
            )

class LogDisplay:
    """로그 디스플레이 컴포넌트"""
    def __init__(self, parent_frame, font_family="Arial"):
        self.parent_frame = parent_frame
        self.font_family = font_family
        self.text_widget = None
        self.setup_log_display()
    
    def setup_log_display(self):
        """로그 디스플레이 설정"""
        import tkinter.ttk as ttk
        
        log_scroll_frame = tk.Frame(self.parent_frame, bg='#34495e')
        log_scroll_frame.pack(fill=tk.BOTH, expand=True)
       
        log_scrollbar = ttk.Scrollbar(log_scroll_frame)
        log_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
       
        self.text_widget = tk.Text(
            log_scroll_frame,
            font=(self.font_family, 10),
            bg='#2c3e50',
            fg='#ecf0f1',
            wrap=tk.WORD,
            yscrollcommand=log_scrollbar.set,
            height=12,
            state=tk.DISABLED
        )
        self.text_widget.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        log_scrollbar.config(command=self.text_widget.yview)
    
    def add_message(self, message: str, color: str = "#ecf0f1"):
        """메시지 추가"""
        from datetime import datetime
        
        timestamp = datetime.now().strftime("%H:%M:%S")
        log_entry = f"[{timestamp}] {message}"
        
        try:
            if self.text_widget:
                self.text_widget.config(state=tk.NORMAL)
                self.text_widget.insert(tk.END, log_entry + "\n")
                self.text_widget.config(state=tk.DISABLED)
                self.text_widget.see(tk.END)
        except Exception as e:
            print(f"GUI log update error: {e}")

class DetectionDisplay:
    """객체 인식 결과 디스플레이 컴포넌트"""
    def __init__(self, parent_frame, font_family="Arial"):
        self.parent_frame = parent_frame
        self.font_family = font_family
        self.text_widget = None
        self.setup_detection_display()
    
    def setup_detection_display(self):
        """인식 결과 디스플레이 설정"""
        import tkinter.ttk as ttk
        
        detection_scroll_frame = tk.Frame(self.parent_frame, bg='#34495e')
        detection_scroll_frame.pack(fill=tk.BOTH, expand=True)
       
        detection_scrollbar = ttk.Scrollbar(detection_scroll_frame)
        detection_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
       
        self.text_widget = tk.Text(
            detection_scroll_frame,
            font=(self.font_family, 9),
            bg='#2c3e50',
            fg='#ecf0f1',
            wrap=tk.WORD,
            yscrollcommand=detection_scrollbar.set,
            height=8,
            state=tk.DISABLED
        )
        self.text_widget.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        detection_scrollbar.config(command=self.text_widget.yview)
    
    def add_detection(self, message: str):
        """인식 결과 추가"""
        from datetime import datetime
        
        timestamp = datetime.now().strftime("%H:%M:%S")
        log_entry = f"[{timestamp}] {message}\n"
       
        if self.text_widget:
            self.text_widget.config(state=tk.NORMAL)
            self.text_widget.insert(tk.END, log_entry)
            self.text_widget.config(state=tk.DISABLED)
            self.text_widget.see(tk.END)
