"""
camera_calibration_utility.py - 카메라 캘리브레이션 및 좌표 변환 유틸리티
Enhanced Dobot Robot & YOLO Object Detection System
"""

import cv2
import numpy as np
import json
import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import threading
from typing import List, Tuple, Dict
import time
import logging

class CalibrationPoint:
    """캘리브레이션 포인트 데이터"""
    def __init__(self, pixel_x: int, pixel_y: int, world_x: float, world_y: float, world_z: float):
        self.pixel_x = pixel_x
        self.pixel_y = pixel_y
        self.world_x = world_x
        self.world_y = world_y
        self.world_z = world_z

class CameraCalibrationTool:
    """카메라 캘리브레이션 도구"""
    
    def __init__(self, robot_controller=None):
        self.robot = robot_controller
        self.cap = None
        self.current_frame = None
        self.calibration_points = []
        self.is_capturing = False
        
        # 캘리브레이션 결과
        self.transformation_matrix = None
        self.calibration_completed = False
        
        # GUI 설정
        self.setup_gui()
        
    def setup_gui(self):
        """GUI 설정"""
        self.root = tk.Tk()
        self.root.title("Camera-Robot Calibration Tool")
        self.root.geometry("1200x800")
        
        # 메인 프레임
        main_frame = tk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # 좌측: 카메라 디스플레이
        left_frame = tk.Frame(main_frame, relief='raised', bd=2)
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 5))
        
        # 카메라 제목
        tk.Label(left_frame, text="Camera View", font=('Arial', 14, 'bold')).pack(pady=5)
        
        # 카메라 디스플레이
        self.camera_label = tk.Label(left_frame, bg='black', width=80, height=30)
        self.camera_label.pack(padx=10, pady=10)
        self.camera_label.bind("<Button-1>", self.on_camera_click)
        
        # 카메라 제어
        camera_control_frame = tk.Frame(left_frame)
        camera_control_frame.pack(pady=5)
        
        self.start_camera_btn = tk.Button(
            camera_control_frame, text="Start Camera", 
            command=self.start_camera, bg='#2ecc71', fg='white'
        )
        self.start_camera_btn.pack(side=tk.LEFT, padx=5)
        
        self.stop_camera_btn = tk.Button(
            camera_control_frame, text="Stop Camera",
            command=self.stop_camera, bg='#e74c3c', fg='white'
        )
        self.stop_camera_btn.pack(side=tk.LEFT, padx=5)
        
        # 우측: 제어 패널
        right_frame = tk.Frame(main_frame, relief='raised', bd=2, width=400)
        right_frame.pack(side=tk.RIGHT, fill=tk.Y, padx=(5, 0))
        right_frame.pack_propagate(False)
        
        # 캘리브레이션 설명
        instruction_frame = tk.LabelFrame(right_frame, text="Instructions", font=('Arial', 12, 'bold'))
        instruction_frame.pack(fill=tk.X, padx=10, pady=10)
        
        instructions = [
            "1. 로봇을 알려진 위치로 이동",
            "2. 카메라에서 로봇 끝점 클릭",
            "3. 'Add Point' 클릭하여 저장",
            "4. 최소 4개 점 수집 후 캘리브레이션",
            "5. 'Calculate' 클릭하여 변환 행렬 계산"
        ]
        
        for i, instruction in enumerate(instructions, 1):
            tk.Label(instruction_frame, text=instruction, font=('Arial', 9), 
                    justify=tk.LEFT).pack(anchor='w', padx=5, pady=2)
        
        # 현재 로봇 위치 입력
        position_frame = tk.LabelFrame(right_frame, text="Robot Position", font=('Arial', 12, 'bold'))
        position_frame.pack(fill=tk.X, padx=10, pady=10)
        
        # 좌표 입력 필드
        coords = ['X', 'Y', 'Z']
        self.coord_vars = {}
        
        for i, coord in enumerate(coords):
            frame = tk.Frame(position_frame)
            frame.pack(fill=tk.X, padx=5, pady=2)
            
            tk.Label(frame, text=f"{coord}:", width=3).pack(side=tk.LEFT)
            var = tk.StringVar(value="0")
            entry = tk.Entry(frame, textvariable=var, width=10)
            entry.pack(side=tk.LEFT, padx=5)
            self.coord_vars[coord] = var
            
            # 현재 위치 가져오기 버튼 (첫 번째 좌표에만)
            if i == 0:
                tk.Button(frame, text="Get Current", 
                         command=self.get_current_robot_position,
                         bg='#3498db', fg='white').pack(side=tk.RIGHT, padx=5)
        
        # 클릭 위치 표시
        click_frame = tk.LabelFrame(right_frame, text="Last Click", font=('Arial', 12, 'bold'))
        click_frame.pack(fill=tk.X, padx=10, pady=10)
        
        self.click_info_label = tk.Label(click_frame, text="No click yet", font=('Arial', 10))
        self.click_info_label.pack(pady=5)
        
        # 포인트 추가 버튼
        self.add_point_btn = tk.Button(
            right_frame, text="Add Calibration Point",
            command=self.add_calibration_point,
            bg='#f39c12', fg='white', font=('Arial', 12, 'bold')
        )
        self.add_point_btn.pack(fill=tk.X, padx=10, pady=10)
        
        # 캘리브레이션 포인트 리스트
        points_frame = tk.LabelFrame(right_frame, text="Calibration Points", font=('Arial', 12, 'bold'))
        points_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # 리스트박스
        self.points_listbox = tk.Listbox(points_frame, height=8)
        self.points_listbox.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # 포인트 삭제 버튼
        tk.Button(points_frame, text="Delete Selected", 
                 command=self.delete_selected_point,
                 bg='#e74c3c', fg='white').pack(pady=5)
        
        # 캘리브레이션 실행
        calc_frame = tk.Frame(right_frame)
        calc_frame.pack(fill=tk.X, padx=10, pady=10)
        
        self.calculate_btn = tk.Button(
            calc_frame, text="Calculate Calibration",
            command=self.calculate_calibration,
            bg='#9b59b6', fg='white', font=('Arial', 12, 'bold')
        )
        self.calculate_btn.pack(fill=tk.X, pady=5)
        
        # 저장/로드 버튼
        save_load_frame = tk.Frame(calc_frame)
        save_load_frame.pack(fill=tk.X, pady=5)
        
        tk.Button(save_load_frame, text="Save", command=self.save_calibration,
                 bg='#27ae60', fg='white').pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)
        tk.Button(save_load_frame, text="Load", command=self.load_calibration,
                 bg='#34495e', fg='white').pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)
        
        # 상태 표시
        self.status_label = tk.Label(right_frame, text="Ready", font=('Arial', 10), 
                                   bg='#ecf0f1', relief='sunken')
        self.status_label.pack(fill=tk.X, padx=10, pady=5)
        
        # 마지막 클릭 위치
        self.last_click_pixel = None
        
    def start_camera(self):
        """카메라 시작"""
        try:
            self.cap = cv2.VideoCapture(0)
            if not self.cap.isOpened():
                messagebox.showerror("Error", "Failed to open camera")
                return
            
            self.is_capturing = True
            self.update_camera_display()
            self.start_camera_btn.config(state='disabled')
            self.stop_camera_btn.config(state='normal')
            self.status_label.config(text="Camera active", bg='#2ecc71')
            
        except Exception as e:
            messagebox.showerror("Error", f"Camera start failed: {e}")
    
    def stop_camera(self):
        """카메라 중지"""
        self.is_capturing = False
        if self.cap:
            self.cap.release()
            self.cap = None
        
        self.start_camera_btn.config(state='normal')
        self.stop_camera_btn.config(state='disabled')
        self.status_label.config(text="Camera stopped", bg='#e74c3c')
    
    def update_camera_display(self):
        """카메라 디스플레이 업데이트"""
        if not self.is_capturing or not self.cap:
            return
        
        try:
            ret, frame = self.cap.read()
            if ret:
                self.current_frame = frame.copy()
                
                # 캘리브레이션 포인트 표시
                display_frame = self.draw_calibration_points(frame)
                
                # GUI에 표시
                self.display_frame_in_label(display_frame)
            
            # 다음 프레임 스케줄링
            self.root.after(33, self.update_camera_display)
            
        except Exception as e:
            print(f"Camera update error: {e}")
    
    def display_frame_in_label(self, frame):
        """프레임을 라벨에 표시"""
        # 크기 조정
        height, width = frame.shape[:2]
        label_width = self.camera_label.winfo_width()
        label_height = self.camera_label.winfo_height()
        
        if label_width > 1 and label_height > 1:
            # 비율 유지하며 리사이징
            scale = min(label_width/width, label_height/height)
            new_width = int(width * scale)
            new_height = int(height * scale)
            
            resized_frame = cv2.resize(frame, (new_width, new_height))
        else:
            resized_frame = cv2.resize(frame, (640, 480))
        
        # BGR to RGB
        rgb_frame = cv2.cvtColor(resized_frame, cv2.COLOR_BGR2RGB)
        
        # PIL Image로 변환
        from PIL import Image, ImageTk
        pil_image = Image.fromarray(rgb_frame)
        photo = ImageTk.PhotoImage(pil_image)
        
        # 라벨 업데이트
        self.camera_label.configure(image=photo)
        self.camera_label.image = photo  # 참조 유지
    
    def draw_calibration_points(self, frame):
        """캘리브레이션 포인트를 프레임에 그리기"""
        display_frame = frame.copy()
        
        # 기존 포인트들 표시
        for i, point in enumerate(self.calibration_points):
            cv2.circle(display_frame, (point.pixel_x, point.pixel_y), 8, (0, 255, 0), -1)
            cv2.circle(display_frame, (point.pixel_x, point.pixel_y), 15, (0, 255, 0), 2)
            
            # 포인트 번호 표시
            cv2.putText(display_frame, str(i+1), 
                       (point.pixel_x + 10, point.pixel_y - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # 마지막 클릭 위치 표시
        if self.last_click_pixel:
            x, y = self.last_click_pixel
            cv2.circle(display_frame, (x, y), 5, (0, 0, 255), -1)
            cv2.circle(display_frame, (x, y), 20, (0, 0, 255), 2)
            cv2.putText(display_frame, "CLICK", (x + 25, y - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        
        # 그리드 오버레이 (선택적)
        self.draw_grid_overlay(display_frame)
        
        return display_frame
    
    def draw_grid_overlay(self, frame):
        """그리드 오버레이 그리기"""
        height, width = frame.shape[:2]
        
        # 십자선 (중심)
        cv2.line(frame, (width//2, 0), (width//2, height), (128, 128, 128), 1)
        cv2.line(frame, (0, height//2), (width, height//2), (128, 128, 128), 1)
        
        # 1/3 지점 선들
        cv2.line(frame, (width//3, 0), (width//3, height), (64, 64, 64), 1)
        cv2.line(frame, (2*width//3, 0), (2*width//3, height), (64, 64, 64), 1)
        cv2.line(frame, (0, height//3), (width, height//3), (64, 64, 64), 1)
        cv2.line(frame, (0, 2*height//3), (width, 2*height//3), (64, 64, 64), 1)
    
    def on_camera_click(self, event):
        """카메라 화면 클릭 이벤트"""
        if not self.current_frame is None:
            # 라벨 좌표를 실제 프레임 좌표로 변환
            label_width = self.camera_label.winfo_width()
            label_height = self.camera_label.winfo_height()
            
            if label_width > 0 and label_height > 0:
                frame_height, frame_width = self.current_frame.shape[:2]
                
                # 비율 계산
                scale_x = frame_width / label_width
                scale_y = frame_height / label_height
                
                # 실제 프레임 좌표
                frame_x = int(event.x * scale_x)
                frame_y = int(event.y * scale_y)
                
                # 경계 체크
                frame_x = max(0, min(frame_x, frame_width - 1))
                frame_y = max(0, min(frame_y, frame_height - 1))
                
                self.last_click_pixel = (frame_x, frame_y)
                
                # UI 업데이트
                self.click_info_label.config(
                    text=f"Pixel: ({frame_x}, {frame_y})\nLabel: ({event.x}, {event.y})"
                )
    
    def get_current_robot_position(self):
        """현재 로봇 위치 가져오기"""
        if self.robot and hasattr(self.robot, 'current_position'):
            pos = self.robot.current_position
            self.coord_vars['X'].set(str(pos[0]))
            self.coord_vars['Y'].set(str(pos[1]))
            self.coord_vars['Z'].set(str(pos[2]))
            self.status_label.config(text=f"Got robot position: {pos[:3]}", bg='#3498db')
        else:
            messagebox.showwarning("Warning", "Robot not connected or position unavailable")
    
    def add_calibration_point(self):
        """캘리브레이션 포인트 추가"""
        if not self.last_click_pixel:
            messagebox.showwarning("Warning", "Please click on camera view first")
            return
        
        try:
            # 월드 좌표 가져오기
            world_x = float(self.coord_vars['X'].get())
            world_y = float(self.coord_vars['Y'].get())
            world_z = float(self.coord_vars['Z'].get())
            
            # 픽셀 좌표
            pixel_x, pixel_y = self.last_click_pixel
            
            # 포인트 생성
            point = CalibrationPoint(pixel_x, pixel_y, world_x, world_y, world_z)
            self.calibration_points.append(point)
            
            # 리스트 업데이트
            point_text = f"Point {len(self.calibration_points)}: P({pixel_x},{pixel_y}) -> W({world_x},{world_y},{world_z})"
            self.points_listbox.insert(tk.END, point_text)
            
            # 상태 업데이트
            self.status_label.config(
                text=f"Added point {len(self.calibration_points)}/4 minimum", 
                bg='#f39c12'
            )
            
            # 캘리브레이션 버튼 활성화 체크
            if len(self.calibration_points) >= 4:
                self.calculate_btn.config(state='normal')
            
            # 클릭 위치 초기화
            self.last_click_pixel = None
            self.click_info_label.config(text="Click camera for next point")
            
        except ValueError:
            messagebox.showerror("Error", "Invalid coordinate values")
    
    def delete_selected_point(self):
        """선택된 포인트 삭제"""
        selection = self.points_listbox.curselection()
        if selection:
            index = selection[0]
            del self.calibration_points[index]
            self.points_listbox.delete(index)
            
            # 리스트 번호 재정렬
            self.points_listbox.delete(0, tk.END)
            for i, point in enumerate(self.calibration_points):
                point_text = f"Point {i+1}: P({point.pixel_x},{point.pixel_y}) -> W({point.world_x},{point.world_y},{point.world_z})"
                self.points_listbox.insert(tk.END, point_text)
            
            # 버튼 상태 업데이트
            if len(self.calibration_points) < 4:
                self.calculate_btn.config(state='disabled')
    
    def calculate_calibration(self):
        """캘리브레이션 계산"""
        if len(self.calibration_points) < 4:
            messagebox.showwarning("Warning", "Need at least 4 calibration points")
            return
        
        try:
            # 픽셀 좌표와 월드 좌표 분리
            pixel_points = []
            world_points = []
            
            for point in self.calibration_points:
                pixel_points.append([point.pixel_x, point.pixel_y])
                world_points.append([point.world_x, point.world_y, point.world_z])
            
            pixel_points = np.array(pixel_points, dtype=np.float32)
            world_points = np.array(world_points, dtype=np.float32)
            
            # 호모그래피 계산 (2D-2D 변환)
            if len(self.calibration_points) >= 4:
                # 평면 호모그래피 (Z값은 별도 처리)
                world_2d = world_points[:, :2]  # X, Y만 사용
                
                homography, mask = cv2.findHomography(pixel_points, world_2d, cv2.RANSAC)
                
                if homography is not None:
                    self.transformation_matrix = homography
                    self.calibration_completed = True
                    
                    # 정확도 테스트
                    accuracy = self.test_calibration_accuracy()
                    
                    self.status_label.config(
                        text=f"Calibration complete! Accuracy: {accuracy:.2f}mm", 
                        bg='#27ae60'
                    )
                    
                    messagebox.showinfo("Success", 
                        f"Calibration completed successfully!\n"
                        f"Average error: {accuracy:.2f}mm\n"
                        f"Transform matrix calculated.")
                else:
                    raise Exception("Failed to calculate homography")
            
        except Exception as e:
            messagebox.showerror("Error", f"Calibration failed: {e}")
            self.status_label.config(text="Calibration failed", bg='#e74c3c')
    
    def test_calibration_accuracy(self):
        """캘리브레이션 정확도 테스트"""
        if not self.calibration_completed or self.transformation_matrix is None:
            return float('inf')
        
        total_error = 0
        for point in self.calibration_points:
            # 픽셀을 월드 좌표로 변환
            predicted_world = self.pixel_to_world_transform(point.pixel_x, point.pixel_y)
            
            # 실제 월드 좌표와 비교 (X, Y만)
            actual_world = [point.world_x, point.world_y]
            
            # 유클리드 거리 계산
            error = np.sqrt((predicted_world[0] - actual_world[0])**2 + 
                           (predicted_world[1] - actual_world[1])**2)
            total_error += error
        
        return total_error / len(self.calibration_points)
    
    def pixel_to_world_transform(self, pixel_x, pixel_y):
        """픽셀 좌표를 월드 좌표로 변환"""
        if not self.calibration_completed or self.transformation_matrix is None:
            return [0, 0, 0]
        
        # 호모그래피 변환
        pixel_point = np.array([[pixel_x, pixel_y]], dtype=np.float32)
        pixel_point = pixel_point.reshape(-1, 1, 2)
        
        world_point = cv2.perspectiveTransform(pixel_point, self.transformation_matrix)
        world_x, world_y = world_point[0][0]
        
        # Z 좌표는 평균값 사용 (또는 별도 계산)
        avg_z = np.mean([p.world_z for p in self.calibration_points])
        
        return [float(world_x), float(world_y), float(avg_z)]
    
    def save_calibration(self):
        """캘리브레이션 결과 저장"""
        if not self.calibration_completed:
            messagebox.showwarning("Warning", "No calibration to save")
            return
        
        try:
            filename = filedialog.asksaveasfilename(
                defaultextension=".json",
                filetypes=[("JSON files", "*.json"), ("All files", "*.*")]
            )
            
            if filename:
                calibration_data = {
                    'transformation_matrix': self.transformation_matrix.tolist(),
                    'calibration_points': [
                        {
                            'pixel_x': p.pixel_x,
                            'pixel_y': p.pixel_y,
                            'world_x': p.world_x,
                            'world_y': p.world_y,
                            'world_z': p.world_z
                        }
                        for p in self.calibration_points
                    ],
                    'accuracy': self.test_calibration_accuracy()
                }
                
                with open(filename, 'w') as f:
                    json.dump(calibration_data, f, indent=2)
                
                messagebox.showinfo("Success", f"Calibration saved to {filename}")
                
        except Exception as e:
            messagebox.showerror("Error", f"Save failed: {e}")
    
    def load_calibration(self):
        """캘리브레이션 결과 로드"""
        try:
            filename = filedialog.askopenfilename(
                filetypes=[("JSON files", "*.json"), ("All files", "*.*")]
            )
            
            if filename:
                with open(filename, 'r') as f:
                    calibration_data = json.load(f)
                
                # 변환 행렬 로드
                self.transformation_matrix = np.array(calibration_data['transformation_matrix'])
                self.calibration_completed = True
                
                # 캘리브레이션 포인트 로드
                self.calibration_points = []
                for point_data in calibration_data['calibration_points']:
                    point = CalibrationPoint(
                        point_data['pixel_x'], point_data['pixel_y'],
                        point_data['world_x'], point_data['world_y'], point_data['world_z']
                    )
                    self.calibration_points.append(point)
                
                # UI 업데이트
                self.points_listbox.delete(0, tk.END)
                for i, point in enumerate(self.calibration_points):
                    point_text = f"Point {i+1}: P({point.pixel_x},{point.pixel_y}) -> W({point.world_x},{point.world_y},{point.world_z})"
                    self.points_listbox.insert(tk.END, point_text)
                
                accuracy = calibration_data.get('accuracy', 0)
                self.status_label.config(
                    text=f"Calibration loaded! Accuracy: {accuracy:.2f}mm", 
                    bg='#27ae60'
                )
                
                messagebox.showinfo("Success", f"Calibration loaded from {filename}")
                
        except Exception as e:
            messagebox.showerror("Error", f"Load failed: {e}")
    
    def run(self):
        """GUI 실행"""
        self.root.mainloop()
    
    def get_calibration_data(self):
        """캘리브레이션 데이터 반환 (외부 사용용)"""
        if self.calibration_completed:
            return {
                'transformation_matrix': self.transformation_matrix,
                'pixel_to_world_func': self.pixel_to_world_transform
            }
        return None

class EnhancedCameraCalibration:
    """향상된 카메라 캘리브레이션 클래스 (yolo_dobot_integration.py 교체용)"""
    
    def __init__(self, calibration_file=None):
        self.transformation_matrix = None
        self.calibration_completed = False
        self.logger = logging.getLogger('robot_system.enhanced_calibration')
        
        # 기본 파라미터 (캘리브레이션이 없을 때 사용)
        self.default_params = {
            'camera_height': 500.0,
            'camera_position': [0, -300, 500],
            'table_height': -50.0,
            'fov_x': 60,
            'fov_y': 45
        }
        
        if calibration_file:
            self.load_calibration_file(calibration_file)
    
    def load_calibration_file(self, filename):
        """캘리브레이션 파일 로드"""
        try:
            with open(filename, 'r') as f:
                data = json.load(f)
            
            self.transformation_matrix = np.array(data['transformation_matrix'])
            self.calibration_completed = True
            self.logger.info(f"Calibration loaded from {filename}")
            
        except Exception as e:
            self.logger.error(f"Failed to load calibration: {e}")
            self.calibration_completed = False
    
    def pixel_to_world(self, pixel_x: int, pixel_y: int, 
                      frame_width: int, frame_height: int,
                      assumed_height: float = None) -> Tuple[float, float, float]:
        """픽셀 좌표를 월드 좌표로 변환 (향상된 버전)"""
        
        if self.calibration_completed and self.transformation_matrix is not None:
            # 캘리브레이션된 변환 사용
            try:
                pixel_point = np.array([[pixel_x, pixel_y]], dtype=np.float32)
                pixel_point = pixel_point.reshape(-1, 1, 2)
                
                world_point = cv2.perspectiveTransform(pixel_point, self.transformation_matrix)
                world_x, world_y = world_point[0][0]
                
                # Z 좌표는 assumed_height 또는 기본값 사용
                world_z = assumed_height if assumed_height is not None else self.default_params['table_height']
                
                self.logger.debug(f"Calibrated transform: ({pixel_x}, {pixel_y}) -> ({world_x:.1f}, {world_y:.1f}, {world_z:.1f})")
                
                return float(world_x), float(world_y), float(world_z)
                
            except Exception as e:
                self.logger.error(f"Calibrated transform failed: {e}")
                # 기본 방법으로 fallback
        
        # 기본 변환 방법 (캘리브레이션이 없을 때)
        return self._default_pixel_to_world(pixel_x, pixel_y, frame_width, frame_height, assumed_height)
    
    def _default_pixel_to_world(self, pixel_x: int, pixel_y: int, 
                               frame_width: int, frame_height: int,
                               assumed_height: float = None) -> Tuple[float, float, float]:
        """기본 픽셀-월드 변환 (캘리브레이션 없을 때)"""
        try:
            # 정규화된 좌표
            norm_x = (pixel_x - frame_width / 2) / (frame_width / 2)
            norm_y = (pixel_y - frame_height / 2) / (frame_height / 2)
            
            # 기본 파라미터 사용
            target_height = assumed_height if assumed_height is not None else self.default_params['table_height']
            camera_height = self.default_params['camera_height']
            camera_to_target_distance = camera_height - target_height
            
            # 시야각 기반 스케일 계산
            fov_x = self.default_params['fov_x']
            fov_y = self.default_params['fov_y']
            
            scale_x = 2 * camera_to_target_distance * np.tan(np.radians(fov_x / 2))
            scale_y = 2 * camera_to_target_distance * np.tan(np.radians(fov_y / 2))
            
            # 월드 좌표 계산
            world_x = norm_x * scale_x / 2
            world_y = self.default_params['camera_position'][1] + camera_to_target_distance + (norm_y * scale_y / 2)
            world_z = target_height
            
            return world_x, world_y, world_z
            
        except Exception as e:
            self.logger.error(f"Default transform failed: {e}")
            return 0.0, 0.0, self.default_params['table_height']

# 메인 애플리케이션과 통합하는 함수
def create_calibration_tool(robot_controller=None):
    """캘리브레이션 도구 생성"""
    return CameraCalibrationTool(robot_controller)

def integrate_calibration_with_main(main_app):
    """메인 애플리케이션에 캘리브레이션 기능 통합"""
    
    def open_calibration_tool():
        """캘리브레이션 도구 열기"""
        try:
            cal_tool = CameraCalibrationTool(main_app.robot_controller)
            
            # 별도 스레드에서 실행
            def run_calibration():
                cal_tool.run()
                
                # 캘리브레이션 완료 후 메인 앱에 적용
                if cal_tool.calibration_completed:
                    # 향상된 캘리브레이션으로 교체
                    enhanced_cal = EnhancedCameraCalibration()
                    enhanced_cal.transformation_matrix = cal_tool.transformation_matrix
                    enhanced_cal.calibration_completed = True
                    
                    # YOLO 검출기의 캘리브레이션 교체
                    if hasattr(main_app, 'yolo_dobot_integration'):
                        main_app.yolo_dobot_integration.detector.calibration = enhanced_cal
                        main_app.log_display.add_message("[🎯] 새로운 캘리브레이션이 적용되었습니다!")
            
            cal_thread = threading.Thread(target=run_calibration, daemon=True)
            cal_thread.start()
            
        except Exception as e:
            main_app.log_display.add_message(f"[❌] 캘리브레이션 도구 오류: {e}")
    
    # 메인 GUI에 캘리브레이션 버튼 추가
    if hasattr(main_app, 'system_frame'):  # system_frame이 있다면
        calibration_btn = tk.Button(
            main_app.system_frame,
            text="🎯 Camera Calibration",
            font=(main_app.korean_font, 10, 'bold'),
            bg='#9b59b6',
            fg='white',
            width=22,
            height=2,
            command=open_calibration_tool
        )
        calibration_btn.pack(fill=tk.X, pady=2)

# 사용 예시
if __name__ == "__main__":
    # 독립 실행
    cal_tool = CameraCalibrationTool()
    cal_tool.run()