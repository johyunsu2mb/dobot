"""
yolo_detector.py - YOLO 객체 인식 클래스 (main.py 호환 버전)
Enhanced Dobot Robot & YOLO Object Detection System
"""

import logging
from config import DEPENDENCIES

# 조건부 임포트
if DEPENDENCIES['YOLO_AVAILABLE'] and DEPENDENCIES['CV2_AVAILABLE']:
    try:
        from ultralytics import YOLO
        import cv2
    except ImportError:
        DEPENDENCIES['YOLO_AVAILABLE'] = False
        DEPENDENCIES['CV2_AVAILABLE'] = False

class YOLODetector:
    """향상된 YOLO 객체 인식 클래스 (main.py 완전 호환)"""
    
    def __init__(self, model_path="./dobot_project-main/python_project_file/best.pt"):
        self.model = None
        self.custom_labels = None
        self.model_loaded = False
        self.logger = logging.getLogger('robot_system.yolo')
        self.class_names = None
        
        # 모델 로드 시도
        if DEPENDENCIES['YOLO_AVAILABLE']:
            self._load_model(model_path)
        else:
            self._safe_log("warning", "YOLO dependencies not available, running without object detection")
    
    def _load_model(self, model_path: str) -> bool:
        """YOLO 모델 로드"""
        try:
            self.model = YOLO(model_path)
            self.model_loaded = True
            
            # 클래스 이름 가져오기 (다양한 형태 지원)
            try:
                # 새로운 형태
                self.class_names = self.model.model.names
            except:
                try:
                    # 이전 형태
                    self.class_names = self.model.names
                except:
                    self.class_names = {}
            
            self._safe_log("info", f"YOLOv8 model loaded successfully from {model_path}")
            return True
            
        except Exception as e:
            self._safe_log("error", f"YOLO model loading failed: {e}")
            self.model_loaded = False
            return False
    
    def _safe_log(self, level: str, message: str):
        """안전한 로깅 (로깅 실패시 print 사용)"""
        try:
            getattr(self.logger, level)(message)
        except:
            print(f"[{level.upper()}] {message}")
    
    def load_custom_labels(self, label_file_path: str) -> bool:
        """커스텀 라벨 파일 로드 (main.py 호환)"""
        try:
            with open(label_file_path, 'r', encoding='utf-8') as f:
                self.custom_labels = [line.strip() for line in f.readlines()]
            self._safe_log("info", f"Custom labels loaded successfully: {len(self.custom_labels)} classes")
            return True
        except Exception as e:
            self._safe_log("error", f"Label file loading failed: {e}")
            return False
    
    def detect_objects(self, frame, confidence: float = 0.8):
        """객체 인식 수행 (main.py 완전 호환 + 중심좌표 추가)"""
        if not self.model_loaded or self.model is None:
            return frame, []
        
        try:
            results = self.model.predict(frame, conf=confidence, verbose=False)
            detections = []
            
            if results and len(results) > 0:
                result = results[0]
                if result.boxes is not None:
                    boxes = result.boxes.xyxy.cpu().numpy()
                    confidences = result.boxes.conf.cpu().numpy()
                    class_ids = result.boxes.cls.cpu().numpy()
                    
                    for i, (box, conf, cls_id) in enumerate(zip(boxes, confidences, class_ids)):
                        x1, y1, x2, y2 = map(int, box)
                        
                        # 중심 좌표 계산 (새로 추가된 기능)
                        cx = (x1 + x2) // 2
                        cy = (y1 + y2) // 2
                        
                        # 클래스 이름 처리 (향상된 로직)
                        class_name = self._get_class_name(int(cls_id))
                        
                        detections.append({
                            'bbox': (x1, y1, x2, y2),
                            'center': (cx, cy),  # 새로 추가된 중심좌표
                            'confidence': float(conf),
                            'class': class_name,
                            'class_id': int(cls_id)
                        })
            
            # 향상된 시각화로 프레임에 그리기
            annotated_frame = self.draw_detections(frame.copy(), detections)
            return annotated_frame, detections
            
        except Exception as e:
            self._safe_log("error", f"Object detection error: {e}")
            return frame, []
    
    def _get_class_name(self, cls_id: int) -> str:
        """클래스 이름 가져오기 (다양한 형태 지원)"""
        # 커스텀 라벨 우선
        if self.custom_labels and cls_id < len(self.custom_labels):
            return self.custom_labels[cls_id]
        
        # 모델의 클래스 이름 사용
        if self.class_names:
            if isinstance(self.class_names, dict):
                return self.class_names.get(cls_id, f"id:{cls_id}")
            elif isinstance(self.class_names, list) and cls_id < len(self.class_names):
                return self.class_names[cls_id]
        
        return f"id:{cls_id}"
    
    def draw_detections(self, frame, detections):
        """인식 결과를 프레임에 그리기 (향상된 시각화)"""
        if not DEPENDENCIES['CV2_AVAILABLE']:
            return frame
        
        for detection in detections:
            x1, y1, x2, y2 = detection['bbox']
            confidence = detection['confidence']
            class_name = detection['class']
            
            # 중심좌표가 있는 경우 표시
            if 'center' in detection:
                cx, cy = detection['center']
                # 중심점 그리기
                cv2.circle(frame, (cx, cy), 3, (0, 0, 255), -1)
                # 중심좌표를 포함한 라벨
                label = f"{class_name}: {confidence:.2f} ({cx},{cy})"
            else:
                label = f"{class_name}: {confidence:.2f}"
            
            # 바운딩 박스 그리기
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            
            # 텍스트 배경
            (text_width, text_height), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
            cv2.rectangle(frame, (x1, y1 - text_height - 10), (x1 + text_width, y1), (0, 255, 0), -1)
            
            # 텍스트 그리기
            cv2.putText(frame, label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
        
        return frame
    
    # main.py에서 사용할 수 있는 추가 기능들
    def get_detection_summary(self, detections):
        """검출 결과 요약 (main.py의 detection_display용)"""
        if not detections:
            return "No objects detected"
        
        summary_parts = []
        for detection in detections:
            class_name = detection['class']
            confidence = detection['confidence']
            if 'center' in detection:
                cx, cy = detection['center']
                summary_parts.append(f"{class_name}({confidence:.2f})@({cx},{cy})")
            else:
                summary_parts.append(f"{class_name}({confidence:.2f})")
        
        return f"Detected: {', '.join(summary_parts)}"
    
    def is_available(self) -> bool:
        """YOLO 사용 가능 여부 확인"""
        return DEPENDENCIES['YOLO_AVAILABLE'] and self.model_loaded
    
    def get_status_info(self) -> dict:
        """상태 정보 반환 (디버깅용)"""
        return {
            'model_loaded': self.model_loaded,
            'yolo_available': DEPENDENCIES['YOLO_AVAILABLE'],
            'cv2_available': DEPENDENCIES['CV2_AVAILABLE'],
            'custom_labels_count': len(self.custom_labels) if self.custom_labels else 0,
            'model_classes_count': len(self.class_names) if self.class_names else 0
        }

# 하위 호환성을 위한 별칭
EnhancedYOLODetector = YOLODetector
