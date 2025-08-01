"""
커스텀 메시지 및 서비스 인터페이스 정의
"""

import json
from typing import Dict, List, Any, Optional
from dataclasses import dataclass, asdict
from enum import Enum

class MessageType(Enum):
    """메시지 타입 열거형"""
    COMMAND = "command"
    STATUS = "status" 
    RESULT = "result"
    REQUEST = "request"
    RESPONSE = "response"
    EMERGENCY = "emergency"

class TaskType(Enum):
    """작업 타입 열거형"""
    MOVE_TO_POSITION = "move_to_position"
    PICKUP_FURNITURE = "pickup_furniture"
    DETECT_OBJECTS = "detect_objects"
    GRIPPER_CONTROL = "gripper_control"
    HOME_POSITION = "home_position"
    GET_POSITION = "get_position"
    EMERGENCY_STOP = "emergency_stop"
    CALIBRATION = "calibration"
    CUSTOM = "custom"

@dataclass
class Position:
    """위치 정보 데이터 클래스"""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    r: float = 0.0  # 회전각
    
    def to_dict(self) -> Dict[str, float]:
        return asdict(self)
    
    @classmethod
    def from_dict(cls, data: Dict[str, float]) -> 'Position':
        return cls(**data)
    
    def distance_to(self, other: 'Position') -> float:
        """다른 위치까지의 거리 계산"""
        return ((self.x - other.x)**2 + (self.y - other.y)**2 + (self.z - other.z)**2)**0.5

@dataclass 
class RobotCommandMsg:
    """로봇 명령 메시지"""
    robot_id: str
    task_type: str
    parameters: Dict[str, Any]
    task_id: Optional[str] = None
    priority: int = 0
    timeout: float = 120.0
    timestamp: float = 0.0
    
    def to_json(self) -> str:
        return json.dumps(asdict(self))
    
    @classmethod
    def from_json(cls, json_str: str) -> 'RobotCommandMsg':
        data = json.loads(json_str)
        return cls(**data)

@dataclass
class RobotStatusMsg:
    """로봇 상태 메시지"""
    robot_id: str
    is_busy: bool
    current_task: Optional[str]
    position: Optional[Position]
    gripper_state: bool
    dobot_available: bool
    yolo_available: bool
    connection_status: Dict[str, bool]
    error_count: int = 0
    last_error: Optional[str] = None
    timestamp: float = 0.0
    
    def to_json(self) -> str:
        data = asdict(self)
        if self.position:
            data['position'] = self.position.to_dict()
        return json.dumps(data)
    
    @classmethod
    def from_json(cls, json_str: str) -> 'RobotStatusMsg':
        data = json.loads(json_str)
        if 'position' in data and data['position']:
            data['position'] = Position.from_dict(data['position'])
        return cls(**data)

@dataclass
class TaskRequestMsg:
    """작업 요청 메시지"""
    task_type: str
    parameters: Dict[str, Any]
    target_robot_id: Optional[str] = None
    priority: int = 0
    timeout: float = 120.0
    requester_id: str = "unknown"
    timestamp: float = 0.0
    
    def to_json(self) -> str:
        return json.dumps(asdict(self))
    
    @classmethod
    def from_json(cls, json_str: str) -> 'TaskRequestMsg':
        data = json.loads(json_str)
        return cls(**data)

@dataclass
class SystemStatusMsg:
    """시스템 상태 메시지"""
    total_robots: int
    active_robots: int
    busy_robots: int
    total_tasks: int
    pending_tasks: int
    active_tasks: int
    completed_tasks: int
    failed_tasks: int
    tcp_enabled: bool
    ros2_enabled: bool
    server_uptime: float
    timestamp: float = 0.0
    
    def to_json(self) -> str:
        return json.dumps(asdict(self))
    
    @classmethod
    def from_json(cls, json_str: str) -> 'SystemStatusMsg':
        data = json.loads(json_str)
        return cls(**data)

# 메시지 생성 헬퍼 함수들
def create_move_command(robot_id: str, x: float, y: float, z: float, r: float = 0.0) -> RobotCommandMsg:
    """이동 명령 생성"""
    return RobotCommandMsg(
        robot_id=robot_id,
        task_type=TaskType.MOVE_TO_POSITION.value,
        parameters={'x': x, 'y': y, 'z': z, 'r': r}
    )

def create_pickup_command(robot_id: str, furniture_type: str) -> RobotCommandMsg:
    """픽업 명령 생성"""
    return RobotCommandMsg(
        robot_id=robot_id,
        task_type=TaskType.PICKUP_FURNITURE.value,
        parameters={'furniture_type': furniture_type}
    )

def create_gripper_command(robot_id: str, state: bool) -> RobotCommandMsg:
    """그리퍼 제어 명령 생성"""
    return RobotCommandMsg(
        robot_id=robot_id,
        task_type=TaskType.GRIPPER_CONTROL.value,
        parameters={'state': state}
    )

def create_emergency_stop_command(robot_id: str = "all") -> RobotCommandMsg:
    """긴급정지 명령 생성"""
    return RobotCommandMsg(
        robot_id=robot_id,
        task_type=TaskType.EMERGENCY_STOP.value,
        parameters={},
        priority=999  # 최고 우선순위
    )