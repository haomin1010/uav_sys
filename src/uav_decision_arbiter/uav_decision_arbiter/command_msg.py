"""
统一的命令消息格式定义
"""
import json
from dataclasses import dataclass, field, asdict
from typing import List, Optional, Dict, Any
from enum import Enum


class CommandMode(Enum):
    """命令模式枚举"""
    TRAJECTORY = "trajectory"
    VELOCITY = "velocity"
    POSITION = "position"
    ATTITUDE = "attitude"


class SourceID(Enum):
    """决策源ID枚举"""
    HUMAN = "human"
    AUTOPILOT = "autopilot"   # 自动驾驶（起飞、降落等）
    CENTRAL = "central"
    RL = "rl"


# 优先级映射
PRIORITY_MAP = {
    SourceID.HUMAN: 200,      # 人类控制：最高优先级
    SourceID.AUTOPILOT: 100,  # 自动驾驶：高优先级（仅次于人类）
    SourceID.CENTRAL: 150,    # 中央算力：中优先级
    SourceID.RL: 120,         # RL决策：低优先级
}


@dataclass
class TrajectoryPoint:
    """轨迹点"""
    t_offset: float  # 相对时间偏移（秒）
    x: float
    y: float
    z: float
    yaw: float
    
    def to_dict(self) -> Dict:
        return asdict(self)
    
    @classmethod
    def from_dict(cls, data: Dict) -> 'TrajectoryPoint':
        return cls(**data)


@dataclass
class Velocity:
    """速度指令"""
    vx: float
    vy: float
    vz: float
    yaw_rate: float
    
    def to_dict(self) -> Dict:
        return asdict(self)
    
    @classmethod
    def from_dict(cls, data: Dict) -> 'Velocity':
        return cls(**data)


@dataclass
class Setpoint:
    """位置设定点"""
    x: float
    y: float
    z: float
    yaw: float
    
    def to_dict(self) -> Dict:
        return asdict(self)
    
    @classmethod
    def from_dict(cls, data: Dict) -> 'Setpoint':
        return cls(**data)


@dataclass
class Header:
    """消息头"""
    timestamp: float      # UNIX epoch seconds (高精度)
    source_id: str        # "human", "central", "rl"
    seq: int              # 序列号
    target_uav_id: str = "all"  # 目标无人机ID，"all"表示广播，或指定"uav1", "uav2"等
    
    def to_dict(self) -> Dict:
        return asdict(self)
    
    @classmethod
    def from_dict(cls, data: Dict) -> 'Header':
        # 兼容旧版本（没有target_uav_id的消息）
        if 'target_uav_id' not in data:
            data['target_uav_id'] = "all"
        return cls(**data)


@dataclass
class Meta:
    """元数据"""
    priority: int         # 0-255，数值越大优先级越高
    mode: str             # "trajectory" | "velocity" | "position" | "attitude"
    expires_at: float     # 有效期截止时间戳（UNIX epoch）
    
    def to_dict(self) -> Dict:
        return asdict(self)
    
    @classmethod
    def from_dict(cls, data: Dict) -> 'Meta':
        return cls(**data)


@dataclass
class Body:
    """命令主体"""
    trajectory: Optional[List[TrajectoryPoint]] = None
    velocity: Optional[Velocity] = None
    setpoint: Optional[Setpoint] = None
    
    def to_dict(self) -> Dict:
        result = {}
        if self.trajectory is not None:
            result['trajectory'] = [p.to_dict() for p in self.trajectory]
        if self.velocity is not None:
            result['velocity'] = self.velocity.to_dict()
        if self.setpoint is not None:
            result['setpoint'] = self.setpoint.to_dict()
        return result
    
    @classmethod
    def from_dict(cls, data: Dict) -> 'Body':
        trajectory = None
        if 'trajectory' in data:
            trajectory = [TrajectoryPoint.from_dict(p) for p in data['trajectory']]
        
        velocity = None
        if 'velocity' in data:
            velocity = Velocity.from_dict(data['velocity'])
        
        setpoint = None
        if 'setpoint' in data:
            setpoint = Setpoint.from_dict(data['setpoint'])
        
        return cls(trajectory=trajectory, velocity=velocity, setpoint=setpoint)


@dataclass
class CommandMsg:
    """统一的命令消息"""
    header: Header
    meta: Meta
    body: Body
    
    def to_json(self) -> str:
        """转换为JSON字符串"""
        return json.dumps({
            'header': self.header.to_dict(),
            'meta': self.meta.to_dict(),
            'body': self.body.to_dict()
        }, indent=2)
    
    @classmethod
    def from_json(cls, json_str: str) -> 'CommandMsg':
        """从JSON字符串解析"""
        data = json.loads(json_str)
        return cls(
            header=Header.from_dict(data['header']),
            meta=Meta.from_dict(data['meta']),
            body=Body.from_dict(data['body'])
        )
    
    def to_dict(self) -> Dict:
        """转换为字典"""
        return {
            'header': self.header.to_dict(),
            'meta': self.meta.to_dict(),
            'body': self.body.to_dict()
        }
    
    @classmethod
    def from_dict(cls, data: Dict) -> 'CommandMsg':
        """从字典解析"""
        return cls(
            header=Header.from_dict(data['header']),
            meta=Meta.from_dict(data['meta']),
            body=Body.from_dict(data['body'])
        )
    
    def is_expired(self, current_time: float) -> bool:
        """检查命令是否过期"""
        return current_time > self.meta.expires_at
    
    def is_valid(self, current_time: float) -> bool:
        """检查命令是否有效（未过期且有正确的内容）"""
        if self.is_expired(current_time):
            return False
        
        # 根据模式检查是否有对应的命令内容
        mode = self.meta.mode
        if mode == CommandMode.TRAJECTORY.value:
            return self.body.trajectory is not None and len(self.body.trajectory) > 0
        elif mode == CommandMode.VELOCITY.value:
            return self.body.velocity is not None
        elif mode == CommandMode.POSITION.value:
            return self.body.setpoint is not None
        
        return False

