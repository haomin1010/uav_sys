"""
RL环境模拟
简化的多机无人机环境，用于RL决策
"""
import numpy as np
from typing import Dict, List, Tuple, Optional


class UAVState:
    """单架无人机状态"""
    
    def __init__(self, uav_id: str, position: np.ndarray):
        self.uav_id = uav_id
        self.position = position.copy()  # [x, y, z]
        self.velocity = np.zeros(3)      # [vx, vy, vz]
        self.yaw = 0.0
        self.yaw_rate = 0.0
        
    def update(self, action: np.ndarray, dt: float = 0.1):
        """根据动作更新状态（简单积分）"""
        # action = [vx, vy, vz, yaw_rate]
        self.velocity = action[:3]
        self.yaw_rate = action[3] if len(action) > 3 else 0.0
        
        # 位置积分
        self.position += self.velocity * dt
        self.yaw += self.yaw_rate * dt
        
        # 限制高度
        self.position[2] = np.clip(self.position[2], 0.0, 50.0)


class MultiUAVEnvironment:
    """多机无人机环境"""
    
    def __init__(self, uav_ids: List[str]):
        self.uav_ids = uav_ids
        self.num_uavs = len(uav_ids)
        
        # 初始化每架无人机的状态
        self.uavs: Dict[str, UAVState] = {}
        for i, uav_id in enumerate(uav_ids):
            # 初始位置稍微分散
            initial_pos = np.array([0.0, i * 2.0, 0.0])
            self.uavs[uav_id] = UAVState(uav_id, initial_pos)
        
        # 目标位置（任务目标）
        self.targets: Dict[str, np.ndarray] = {}
        for uav_id in uav_ids:
            self.targets[uav_id] = np.array([10.0, 10.0, 5.0])
        
        # 时间步
        self.dt = 0.1
        self.time = 0.0
    
    def reset(self, initial_positions: Optional[Dict[str, np.ndarray]] = None):
        """重置环境"""
        if initial_positions:
            for uav_id, pos in initial_positions.items():
                if uav_id in self.uavs:
                    self.uavs[uav_id].position = pos.copy()
        
        self.time = 0.0
    
    def step(self, actions: Dict[str, np.ndarray]) -> Dict[str, Dict]:
        """环境步进"""
        # 更新每架无人机
        for uav_id, action in actions.items():
            if uav_id in self.uavs:
                self.uavs[uav_id].update(action, self.dt)
        
        self.time += self.dt
        
        # 计算奖励和观测
        obs = self.get_observations()
        rewards = self.compute_rewards()
        dones = self.check_done()
        info = self.get_info()
        
        return obs, rewards, dones, info
    
    def get_observations(self) -> Dict[str, np.ndarray]:
        """获取观测"""
        obs = {}
        for uav_id, uav in self.uavs.items():
            # 观测 = [自身位置, 自身速度, 目标位置, 其他无人机相对位置]
            target = self.targets.get(uav_id, np.zeros(3))
            
            # 简化观测：[x, y, z, vx, vy, vz, target_x, target_y, target_z]
            obs[uav_id] = np.concatenate([
                uav.position,
                uav.velocity,
                target
            ])
        
        return obs
    
    def compute_rewards(self) -> Dict[str, float]:
        """计算奖励"""
        rewards = {}
        for uav_id, uav in self.uavs.items():
            target = self.targets.get(uav_id, np.zeros(3))
            
            # 距离目标的距离
            distance = np.linalg.norm(uav.position - target)
            
            # 奖励 = -距离（越接近目标越好）
            reward = -distance * 0.1
            
            # 到达目标奖励
            if distance < 1.0:
                reward += 10.0
            
            # 碰撞惩罚（简化：检测高度）
            if uav.position[2] < 0.5:
                reward -= 50.0
            
            rewards[uav_id] = reward
        
        return rewards
    
    def check_done(self) -> Dict[str, bool]:
        """检查是否完成"""
        dones = {}
        for uav_id, uav in self.uavs.items():
            target = self.targets.get(uav_id, np.zeros(3))
            distance = np.linalg.norm(uav.position - target)
            
            # 到达目标或碰撞
            done = distance < 0.5 or uav.position[2] < 0.1
            dones[uav_id] = done
        
        return dones
    
    def get_info(self) -> Dict[str, Dict]:
        """获取附加信息"""
        info = {}
        for uav_id, uav in self.uavs.items():
            info[uav_id] = {
                'position': uav.position.tolist(),
                'velocity': uav.velocity.tolist(),
                'yaw': uav.yaw,
                'time': self.time
            }
        return info
    
    def set_targets(self, targets: Dict[str, np.ndarray]):
        """设置目标位置"""
        self.targets.update(targets)
    
    def get_state(self, uav_id: str) -> Optional[UAVState]:
        """获取指定无人机状态"""
        return self.uavs.get(uav_id)

