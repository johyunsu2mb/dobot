"""
간단한 설정 로더
"""

class ConfigLoader:
    @staticmethod
    def load_config():
        return {}
    
    @staticmethod
    def get_dobot_config():
        return {'host': '192.168.1.6', 'port': 29999}
    
    @staticmethod
    def get_http_config():
        return {'port': 8080}
    
    @staticmethod
    def get_preset_positions():
        return {
            'home': [200.0, 0.0, 100.0, 0.0],
            'work_pos_1': [250.0, 0.0, 150.0, 0.0]
        }
