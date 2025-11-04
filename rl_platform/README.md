# RL决策平台

## 📝 概述

这是一个完整的RL决策平台，包括：
- **可视化界面**（Pygame） - 实时显示多架无人机
- **RL算法模块** - 简单但可运行的决策策略
- **ROS2通信** - 与系统无缝集成

## 🏗️ 模块结构

```
rl_platform/
├── __init__.py                # 包初始化
├── rl_env.py                  # 环境模拟
├── rl_policy.py               # RL策略
├── rl_visualizer.py           # 可视化界面
├── rl_platform_node.py        # ROS2主节点
├── requirements.txt           # 依赖
└── README.md                  # 本文件
```

## ✨ 功能特性

### 1. 可视化界面
- ✅ 实时显示多架无人机位置
- ✅ 显示速度向量
- ✅ 显示轨迹历史
- ✅ 显示目标点
- ✅ 信息面板（位置、速度、控制源）
- ✅ 交互控制（暂停/继续/重置）

### 2. RL算法
提供两种策略：

**SimpleRLPolicy**（简单导航）:
- 基于P控制器朝向目标飞行
- 自动避碰（相邻无人机）
- 高度限制

**CircleFormationPolicy**（圆形编队）:
- 所有无人机围绕中心点旋转
- 保持圆形队形
- 演示编队飞行

### 3. ROS2通信

**发送**：
- 决策命令 → `/{uav_id}/rl/decision_output`
- 格式：`{"action": [vx, vy, vz, yaw_rate]}`

**接收**：
- 可视化命令 ← `/{uav_id}/rl/visualization_cmd`
- 编队同步 ← `/uav/formation_sync`
- 权威命令 ← `/uav/authoritative_cmd`

## 🚀 使用方法

### 启动RL平台

```bash
# 确保环境已激活
cd /home/lihaomin/project/uav_sys
source setup_env.sh

# 启动RL平台
python3 rl_platform/rl_platform_node.py
```

### 完整系统测试

```bash
# 终端1: 启动多机系统
ros2 launch uav_decision_arbiter multi_uav.launch.py

# 终端2: 启动RL平台（带可视化）
python3 rl_platform/rl_platform_node.py

# 终端3: 模拟PX4（可选）
python3 examples/mock_multi_px4.py
```

## 🎮 界面说明

### 主视图（左侧）
- **网格**：坐标参考
- **红色轴**：X轴（东向）
- **绿色轴**：Y轴（北向）
- **彩色圆点**：无人机（带ID标签）
- **箭头**：速度向量
- **浅色线**：历史轨迹
- **橙色X**：目标点

### 信息面板（右侧）
- **Control**: 当前控制源（rl/central/human）
- **RL Step**: 决策步数
- **各无人机状态**:
  - 位置 (x, y, z)
  - 速度 (m/s)
  - 目标距离 (m)
- **编队信息**: 编队长机、队形等

### 键盘控制
- `SPACE` - 暂停/继续RL决策
- `R` - 重置环境
- `Q` - 退出程序

## 📊 策略说明

### SimpleRLPolicy（简单导航）

**原理**：P控制器

```python
# 计算到目标的误差
error = target - position

# 速度命令 ∝ 误差
velocity = error * gain

# 避碰：距离其他无人机太近时产生排斥力
if distance_to_other < 2.0:
    velocity += repulsion_force
```

**适用场景**：
- 点到点导航
- 多机独立任务
- 简单避碰

### CircleFormationPolicy（圆形编队）

**原理**：参数化圆周运动

```python
# 每架无人机在圆周上的角度
angle[uav_id] = 2π * id / num_uavs + ω * time

# 目标位置
target = center + radius * [cos(angle), sin(angle), 0]

# P控制朝向目标
velocity = (target - position) * gain
```

**适用场景**：
- 编队飞行演示
- 圆形巡逻
- 协同监控

## 🔧 自定义策略

### 方法1：修改现有策略

编辑 `rl_policy.py`：

```python
class MyCustomPolicy(SimpleRLPolicy):
    def get_action(self, observation, uav_id, all_positions=None):
        # 你的决策逻辑
        action = ...
        return action
```

修改 `rl_platform_node.py`：

```python
# 导入你的策略
from rl_policy import MyCustomPolicy

# 在__init__中
self.policy = MyCustomPolicy(self.uav_ids)
```

### 方法2：使用神经网络

```python
import torch

class NeuralPolicy:
    def __init__(self, model_path):
        self.model = torch.load(model_path)
        self.model.eval()
    
    def get_action(self, observation, uav_id):
        with torch.no_grad():
            obs_tensor = torch.FloatTensor(observation)
            action = self.model(obs_tensor).numpy()
        return action
```

## 📡 ROS2接口

### 发布话题

```python
# 每架无人机独立发布
/{uav1}/rl/decision_output
/{uav2}/rl/decision_output
/{uav3}/rl/decision_output

# 消息格式 (std_msgs/String)
{
  "action": [vx, vy, vz, yaw_rate],  # 速度命令
  "timestamp": 1762247595.123
}
```

### 订阅话题

```python
# 接收可视化命令
/{uav_id}/rl/visualization_cmd

# 接收编队同步
/uav/formation_sync

# 监听权威命令（判断谁在控制）
/uav/authoritative_cmd
```

## 🎯 测试场景

### 场景1：RL单独控制

```bash
# 启动系统
ros2 launch uav_decision_arbiter multi_uav.launch.py

# 启动RL平台
python3 rl_platform/rl_platform_node.py
```

**观察**：
- 可视化界面显示3架无人机
- 无人机朝向目标移动
- 信息面板显示"Control: rl"（绿色）

### 场景2：优先级抢占

```bash
# RL平台运行中，启动中央算力
ros2 topic pub /uav2/central/decision_output std_msgs/msg/String \
  'data: "{\"type\": \"position\", \"position\": {\"x\": 15, \"y\": 15, \"z\": 5, \"yaw\": 0}}"'
```

**观察**：
- uav2切换到"Control: central"（变灰）
- uav1和uav3仍然"Control: rl"（绿色）
- uav2的轨迹改变（被中央控制）

### 场景3：编队同步

```bash
# 启动完整系统
ros2 launch uav_decision_arbiter multi_uav.launch.py

# 模拟PX4
python3 examples/mock_multi_px4.py

# 启动RL平台
python3 rl_platform/rl_platform_node.py
```

**观察**：
- RL平台接收编队同步信息
- 无人机初始位置自动设置
- 信息面板显示编队信息

## 💡 扩展建议

### 1. 集成真实RL算法

```python
# 使用PyTorch + Stable-Baselines3
from stable_baselines3 import PPO

class RealRLPolicy:
    def __init__(self):
        self.model = PPO.load("trained_model.zip")
    
    def get_action(self, observation):
        action, _ = self.model.predict(observation)
        return action
```

### 2. 添加3D视图

修改 `rl_visualizer.py`，添加侧视图或3D透视。

### 3. 增加传感器模拟

```python
# 在环境中添加
def get_lidar_scan(self, uav_id):
    """模拟激光雷达"""
    ...

def get_camera_image(self, uav_id):
    """模拟相机图像"""
    ...
```

### 4. 强化学习训练

```python
from stable_baselines3 import PPO

# 训练
env = MultiUAVEnvironment(['uav1', 'uav2', 'uav3'])
model = PPO("MlpPolicy", env, verbose=1)
model.learn(total_timesteps=100000)
model.save("multi_uav_ppo")
```

## 🐛 故障排除

### pygame错误

如果提示pygame无法导入：
```bash
conda activate uav_sys
pip install pygame
```

### 界面不显示

检查DISPLAY环境变量：
```bash
echo $DISPLAY
# WSL需要X Server（如VcXsrv、XMing）
```

### 无法连接ROS2

确认环境已source：
```bash
source setup_env.sh
python3 -c "import rclpy; print('OK')"
```

## 📚 代码示例

### 最简单的使用

```python
#!/usr/bin/env python3
import rclpy
from rl_platform_node import RLPlatformNode

rclpy.init()
node = RLPlatformNode()
rclpy.spin(node)
```

### 自定义配置

```python
# 使用不同策略
node = RLPlatformNode()
node.policy_type = 'circle'  # 或 'simple'

# 调整控制频率
node.control_frequency = 20.0  # Hz
```

## 总结

这个RL平台提供了：
- ✅ 完整的可视化界面
- ✅ 可运行的RL决策逻辑
- ✅ ROS2集成（发送/接收）
- ✅ 多机支持（1-10架）
- ✅ 易于扩展和替换

可以直接运行演示，也可以替换为你的实际RL算法！

**下一步**：运行 `python3 rl_platform/rl_platform_node.py` 查看效果！

