"""
간단한 로거 설정
"""

import logging

class LoggerSetup:
    @staticmethod
    def setup_logging():
        logging.basicConfig(level=logging.INFO)
    
    @staticmethod
    def get_logger(name):
        return logging.getLogger(name)
