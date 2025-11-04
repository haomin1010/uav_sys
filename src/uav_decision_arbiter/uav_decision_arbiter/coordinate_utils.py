"""
坐标系转换工具
支持ENU（ROS/MAVROS标准）和NED（AirSim/PX4标准）之间的转换
"""
from typing import Dict, Tuple
import math


def enu_to_ned(enu_pos: Dict[str, float]) -> Dict[str, float]:
    """
    ENU (East-North-Up) 转 NED (North-East-Down)
    
    Args:
        enu_pos: {'x': east, 'y': north, 'z': up}
    
    Returns:
        {'x': north, 'y': east, 'z': down}
    """
    return {
        'x': enu_pos['y'],    # North = ENU的North
        'y': enu_pos['x'],    # East = ENU的East
        'z': -enu_pos['z']    # Down = -Up
    }


def ned_to_enu(ned_pos: Dict[str, float]) -> Dict[str, float]:
    """
    NED (North-East-Down) 转 ENU (East-North-Up)
    
    Args:
        ned_pos: {'x': north, 'y': east, 'z': down}
    
    Returns:
        {'x': east, 'y': north, 'z': up}
    """
    return {
        'x': ned_pos['y'],    # East = NED的East
        'y': ned_pos['x'],    # North = NED的North
        'z': -ned_pos['z']    # Up = -Down
    }


def quaternion_to_euler(q: Dict[str, float]) -> Tuple[float, float, float]:
    """
    四元数转欧拉角（roll, pitch, yaw）
    
    Args:
        q: {'w': w, 'x': x, 'y': y, 'z': z}
    
    Returns:
        (roll, pitch, yaw) in radians
    """
    w, x, y, z = q['w'], q['x'], q['y'], q['z']
    
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    
    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)
    
    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    
    return roll, pitch, yaw


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Dict[str, float]:
    """
    欧拉角转四元数
    
    Args:
        roll, pitch, yaw: 欧拉角 (radians)
    
    Returns:
        {'w': w, 'x': x, 'y': y, 'z': z}
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    
    return {
        'w': cr * cp * cy + sr * sp * sy,
        'x': sr * cp * cy - cr * sp * sy,
        'y': cr * sp * cy + sr * cp * sy,
        'z': cr * cp * sy - sr * sp * cy
    }


def add_offset(pos: Dict[str, float], offset: Dict[str, float]) -> Dict[str, float]:
    """
    位置加偏移量
    
    Args:
        pos: {'x': ..., 'y': ..., 'z': ...}
        offset: {'x': ..., 'y': ..., 'z': ...}
    
    Returns:
        pos + offset
    """
    return {
        'x': pos['x'] + offset['x'],
        'y': pos['y'] + offset['y'],
        'z': pos['z'] + offset['z']
    }


def calculate_relative_position(pos1: Dict[str, float], 
                                pos2: Dict[str, float]) -> Dict[str, float]:
    """
    计算两个位置的相对位置（pos2相对于pos1）
    
    Args:
        pos1: 参考位置
        pos2: 目标位置
    
    Returns:
        relative = pos2 - pos1
    """
    return {
        'x': pos2['x'] - pos1['x'],
        'y': pos2['y'] - pos1['y'],
        'z': pos2['z'] - pos1['z']
    }


def distance_3d(pos1: Dict[str, float], pos2: Dict[str, float]) -> float:
    """计算3D距离"""
    dx = pos2['x'] - pos1['x']
    dy = pos2['y'] - pos1['y']
    dz = pos2['z'] - pos1['z']
    return math.sqrt(dx*dx + dy*dy + dz*dz)

