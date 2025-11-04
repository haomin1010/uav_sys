"""
RL策略（简单示例）
可以替换为你的实际RL算法（PPO、SAC等）
"""
import numpy as np
from typing import Dict, List


class SimpleRLPolicy:
    """
    简单的RL策略示例
    使用基于规则的导航（可替换为神经网络）
    """
    
    def __init__(self, uav_ids: List[str], obs_dim: int = 9, act_dim: int = 4):
        self.uav_ids = uav_ids
        self.obs_dim = obs_dim
        self.act_dim = act_dim
        
        # 这里可以加载训练好的模型
        # self.model = torch.load('model.pth')
        
        # 策略参数
        self.max_speed = 2.0
        self.arrival_threshold = 1.0
        
    def get_action(self, observation: np.ndarray, uav_id: str) -> np.ndarray:
        """
        根据观测获取动作
        
        Args:
            observation: [x, y, z, vx, vy, vz, target_x, target_y, target_z]
            uav_id: 无人机ID
        
        Returns:
            action: [vx, vy, vz, yaw_rate]
        """
        # 解析观测
        position = observation[:3]
        velocity = observation[3:6]
        target = observation[6:9]
        
        # 简单策略：P控制器朝向目标
        error = target - position
        distance = np.linalg.norm(error)
        
        if distance < self.arrival_threshold:
            # 到达目标，悬停
            action = np.zeros(4)
        else:
            # 朝向目标移动
            direction = error / (distance + 1e-6)
            
            # 速度与距离成正比，但限制最大速度
            speed = min(distance * 0.5, self.max_speed)
            velocity_command = direction * speed
            
            # yaw控制（朝向目标）
            yaw_target = np.arctan2(error[1], error[0])
            yaw_rate = yaw_target * 0.5  # 简单P控制
            
            action = np.array([
                velocity_command[0],
                velocity_command[1],
                velocity_command[2],
                yaw_rate
            ])
        
        return action
    
    def get_batch_actions(self, observations: Dict[str, np.ndarray]) -> Dict[str, np.ndarray]:
        """批量获取动作"""
        actions = {}
        for uav_id, obs in observations.items():
            actions[uav_id] = self.get_action(obs, uav_id)
        return actions


class FormationRLPolicy(SimpleRLPolicy):
    """
    编队飞行策略
    添加了编队保持和避碰功能
    """
    
    def __init__(self, uav_ids: List[str]):
        super().__init__(uav_ids)
        
        # 编队参数
        self.formation_gain = 0.3
        self.collision_distance = 2.0
        self.avoidance_gain = 0.5
    
    def get_action(self, observation: np.ndarray, uav_id: str, 
                   all_positions: Dict[str, np.ndarray] = None) -> np.ndarray:
        """
        考虑编队和避碰的动作
        
        Args:
            observation: 自身观测
            uav_id: 无人机ID
            all_positions: 所有无人机位置（用于避碰）
        """
        # 基础动作（朝向目标）
        action = super().get_action(observation, uav_id)
        
        if all_positions is None:
            return action
        
        # 添加避碰行为
        position = observation[:3]
        avoidance_vector = np.zeros(3)
        
        for other_id, other_pos in all_positions.items():
            if other_id == uav_id:
                continue
            
            # 计算与其他无人机的相对位置
            relative = position - other_pos
            distance = np.linalg.norm(relative)
            
            # 如果太近，产生排斥力
            if distance < self.collision_distance and distance > 0.1:
                # 排斥力与距离成反比
                repulsion = (self.collision_distance - distance) / distance
                avoidance_vector += relative * repulsion
        
        # 合并避碰向量
        action[:3] += avoidance_vector * self.avoidance_gain
        
        # 限制速度
        speed = np.linalg.norm(action[:3])
        if speed > self.max_speed:
            action[:3] = action[:3] / speed * self.max_speed
        
        return action


class CircleFormationPolicy:
    """
    圆形编队策略示例
    所有无人机围绕中心点做圆周运动
    """
    
    def __init__(self, uav_ids: List[str], radius: float = 5.0, angular_speed: float = 0.2):
        self.uav_ids = uav_ids
        self.num_uavs = len(uav_ids)
        self.radius = radius
        self.angular_speed = angular_speed
        
        # 为每架无人机分配角度
        self.angles = {}
        for i, uav_id in enumerate(uav_ids):
            self.angles[uav_id] = 2 * np.pi * i / self.num_uavs
    
    def get_batch_actions(self, observations: Dict[str, np.ndarray], time: float) -> Dict[str, np.ndarray]:
        """
        圆形编队控制
        """
        actions = {}
        center = np.array([10.0, 10.0, 5.0])  # 编队中心
        
        for uav_id, obs in observations.items():
            current_pos = obs[:3]
            
            # 计算目标位置（圆周上）
            angle = self.angles[uav_id] + self.angular_speed * time
            target_pos = center + np.array([
                self.radius * np.cos(angle),
                self.radius * np.sin(angle),
                0.0
            ])
            
            # P控制朝向目标
            error = target_pos - current_pos
            velocity = error * 0.8  # P增益
            
            # 限制速度
            speed = np.linalg.norm(velocity)
            max_speed = 2.0
            if speed > max_speed:
                velocity = velocity / speed * max_speed
            
            actions[uav_id] = np.array([
                velocity[0],
                velocity[1],
                velocity[2],
                self.angular_speed  # yaw跟随圆周方向
            ])
        
        return actions

