"""
logger_setup.py - 로깅 시스템 설정
Enhanced Dobot Robot & YOLO Object Detection System
"""

import logging
import os
from logging.handlers import RotatingFileHandler
from datetime import datetime
from typing import List

def setup_logging():
    """로깅 시스템 설정 (Windows 한글/이모지 인코딩 문제 해결)"""
    logger = logging.getLogger('robot_system')
    logger.setLevel(logging.INFO)
    
    # 기존 핸들러 제거
    for handler in logger.handlers[:]:
        logger.removeHandler(handler)
    
    # 로그 디렉터리 생성
    if not os.path.exists('logs'):
        os.makedirs('logs')
    
    # 파일 핸들러 (UTF-8 인코딩 명시)
    file_handler = RotatingFileHandler(
        'logs/robot_system.log',
        maxBytes=10*1024*1024,  # 10MB
        backupCount=5,
        encoding='utf-8'  # UTF-8 인코딩 명시
    )
    
    # 콘솔 핸들러 (안전한 인코딩 처리)
    console_handler = SafeConsoleHandler()
    
    # 포맷터
    file_formatter = logging.Formatter(
        '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    console_formatter = SafeFormatter(
        '%(asctime)s - %(levelname)s - %(message)s'
    )
    
    file_handler.setFormatter(file_formatter)
    console_handler.setFormatter(console_formatter)
    
    logger.addHandler(file_handler)
    logger.addHandler(console_handler)
    
    return logger

class SafeConsoleHandler(logging.StreamHandler):
    """Windows 콘솔에서 안전한 출력을 위한 핸들러"""
    
    def emit(self, record):
        try:
            # 기존 방식으로 시도
            super().emit(record)
        except UnicodeEncodeError:
            try:
                # 안전한 문자로 변환하여 재시도
                safe_record = self.make_safe_record(record)
                super().emit(safe_record)
            except:
                # 최후의 수단: 에러 없이 패스
                pass
    
    def make_safe_record(self, record):
        """레코드를 안전한 문자로 변환"""
        import copy
        safe_record = copy.copy(record)
        
        # 메시지에서 이모지와 특수 문자 제거
        safe_message = self.safe_encode(record.getMessage())
        safe_record.msg = safe_message
        safe_record.args = ()
        
        return safe_record
    
    def safe_encode(self, text):
        """텍스트를 안전한 형태로 인코딩"""
        if not isinstance(text, str):
            text = str(text)
        
        # 이모지와 특수 유니코드 문자를 안전한 문자로 대체
        emoji_map = {
            '🤖': '[ROBOT]',
            '🎉': '[PARTY]',
            '⚠️': '[WARNING]',
            '🎯': '[TARGET]',
            '📋': '[CLIPBOARD]',
            '✅': '[CHECK]',
            '❌': '[CROSS]',
            '📍': '[PIN]',
            '🔄': '[REFRESH]',
            '📷': '[CAMERA]',
            '🔌': '[PLUG]',
            '🟢': '[GREEN]',
            '🔴': '[RED]',
            '🛋️': '[SOFA]',
            '🪑': '[CHAIR]',
            '📚': '[BOOK]',
            '🛏️': '[BED]',
            '🤏': '[PINCH]',
            '🏷️': '[LABEL]',
            '⚙️': '[GEAR]',
            '🎮': '[GAMEPAD]',
            '🪑': '[CHAIR2]'
        }
        
        # 이모지 대체
        for emoji, replacement in emoji_map.items():
            text = text.replace(emoji, replacement)
        
        # 나머지 특수 문자들을 안전하게 처리
        try:
            # Windows 콘솔에서 지원하는 인코딩으로 테스트
            text.encode('cp949', errors='replace')
        except:
            # 안전한 ASCII 문자만 유지
            text = text.encode('ascii', errors='replace').decode('ascii')
        
        return text

class SafeFormatter(logging.Formatter):
    """안전한 포맷터"""
    
    def format(self, record):
        try:
            return super().format(record)
        except UnicodeEncodeError:
            # 안전한 메시지로 대체
            record.msg = "[ENCODING_ERROR] " + str(record.msg).encode('ascii', errors='replace').decode('ascii')
            record.args = ()
            return super().format(record)

class OrderLogger:
    """향상된 주문 로그 관리 클래스 (인코딩 안전성 향상)"""
    def __init__(self):
        self.log_file = "logs/order_log.txt"
        self.orders = []
        self.logger = logging.getLogger('robot_system.orders')
        
        # 로그 디렉터리 생성
        os.makedirs(os.path.dirname(self.log_file), exist_ok=True)
   
    def log_order(self, item: str, status: str = "시작", details: str = ""):
        """주문을 로그에 기록 (안전한 인코딩)"""
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        log_entry = f"[{timestamp}] {item} 주문 - {status}"
        if details:
            log_entry += f" ({details})"
        
        self.orders.append(log_entry)
        
        # 안전한 로깅
        try:
            self.logger.info(log_entry)
        except UnicodeEncodeError:
            # 안전한 ASCII 변환
            safe_entry = log_entry.encode('ascii', errors='replace').decode('ascii')
            self.logger.info(f"[SAFE_LOG] {safe_entry}")
       
        # 파일 저장 (UTF-8)
        try:
            with open(self.log_file, "a", encoding="utf-8") as f:
                f.write(log_entry + "\n")
        except Exception as e:
            try:
                # 안전한 로깅으로 에러 기록
                self.logger.error(f"로그 파일 기록 실패: {str(e)}")
            except:
                print(f"Log file write failed: {e}")  # 최후의 수단
   
    def get_recent_orders(self, count: int = 5) -> List[str]:
        """최근 주문 내역 반환"""
        return self.orders[-count:] if self.orders else []

# 전역 로거 초기화
system_logger = setup_logging()
