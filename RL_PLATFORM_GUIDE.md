# RL平台完整使用指南

## 🎯 系统组成

你的RL决策平台现在包括4个核心模块：

```
┌─────────────────────────────────────────────────────┐
│              RL Decision Platform                    │
├─────────────────────────────────────────────────────┤
│                                                       │
│  ┌──────────┐   ┌──────────┐   ┌──────────┐        │
│  │   环境    │   │   策略    │   │  可视化   │        │
│  │  rl_env   │──▶│ rl_policy│──▶│rl_visualizer│     │
│  └──────────┘   └──────────┘   └──────────┘        │
│       ↕                               ↕              │
│  ┌──────────────────────────────────────────┐      │
│  │         ROS2通信 (rl_platform_node)       │      │
│  └──────────────────────────────────────────┘      │
│       ↕             ↕             ↕                  │
└───────┼─────────────┼─────────────┼──────────────────┘
        │             │             │
        ▼             ▼             ▼
   发送决策      接收同步      监听控制权
```

## 📦 模块说明

### 1. `rl_env.py` - 环境模拟

**UAVState类**：
- 单架无人机状态（位置、速度、姿态）
- 状态更新（基于动作积分）

**MultiUAVEnvironment类**：
- 多机环境管理
- 观测/奖励计算
- 环境步进

### 2. `rl_policy.py` - 决策策略

**SimpleRLPolicy**：
- P控制器导航
- 自动避碰

**CircleFormationPolicy**：
- 圆形编队飞行
- 演示协同行为

### 3. `rl_visualizer.py` - 可视化

**UAVVisualizer类**：
- Pygame图形界面
- 实时渲染
- 信息显示

### 4. `rl_platform_node.py` - ROS2节点

**RLPlatformNode类**：
- 集成上述模块
- ROS2通信
- 主控制循环

## 🚀 快速开始

### 单独运行RL平台

```bash
# 方式1：使用启动脚本
./run_rl_platform.sh

# 方式2：手动启动
source setup_env.sh
python3 rl_platform/rl_platform_node.py
```

**你会看到**：
- Pygame窗口打开
- 显示3架无人机
- 开始朝目标移动

### 与系统集成运行

```bash
# 终端1：多机系统
ros2 launch uav_decision_arbiter multi_uav.launch.py

# 终端2：RL平台（可视化）
python3 rl_platform/rl_platform_node.py

# 终端3：模拟PX4（可选）
python3 examples/mock_multi_px4.py
```

## 🎨 界面截图说明

```
┌──────────────────────────────────────────────────────────────┐
│  UAV RL Platform - 多机决策可视化                             │
├────────────────────────────────┬─────────────────────────────┤
│                                │  RL Decision Platform        │
│         主视图（俯视）           │                             │
│                                │  Control: rl ✓              │
│    Y(North)                   │  RL Step: 123               │
│      ↑                         │                             │
│      │                         │  UAV1                       │
│      │   ○ uav3               │    Pos: (5.2, 3.1, 2.0)    │
│      │      →                  │    Spd: 1.2 m/s            │
│      │                         │    Dst: 7.8 m              │
│      ○─────→ X(East)           │                             │
│    uav1  ╲                     │  UAV2                       │
│            ○ uav2              │    Pos: (6.1, 1.2, 2.0)    │
│              →                  │    Spd: 1.5 m/s            │
│                                │    Dst: 8.3 m              │
│    X 目标点                     │                             │
│                                │  UAV3                       │
│    浅色线 = 轨迹                │    Pos: (4.8, 5.3, 2.0)    │
│    箭头 = 速度向量              │    Spd: 1.1 m/s            │
│                                │    Dst: 7.2 m              │
│                                │                             │
│                                │  Formation Info             │
│                                │    Leader: uav1             │
│                                │                             │
│                                │  Controls:                  │
│                                │    SPACE - Pause/Resume     │
│                                │    R - Reset                │
│                                │    Q - Quit                 │
└────────────────────────────────┴─────────────────────────────┘
```

## 🎮 交互操作

| 按键 | 功能 | 说明 |
|------|------|------|
| SPACE | 暂停/继续 | 暂停RL决策，无人机悬停 |
| R | 重置环境 | 无人机回到初始位置 |
| Q | 退出 | 关闭RL平台 |

## 📡 ROS2通信详解

### 发送决策（输出）

**话题**：`/{uav_id}/rl/decision_output`

**示例消息**：
```json
{
  "action": [1.2, 0.5, 0.3, 0.1],
  "timestamp": 1762247595.123
}
```

**字段说明**：
- `action[0]`: vx（东向速度，m/s）
- `action[1]`: vy（北向速度，m/s）
- `action[2]`: vz（上向速度，m/s）
- `action[3]`: yaw_rate（偏航角速度，rad/s）

**代码位置**：
```python
# rl_platform_node.py: publish_decisions()
def publish_decisions(self, actions):
    for uav_id, action in actions.items():
        decision = {
            "action": action.tolist(),
            "timestamp": time.time()
        }
        self.decision_publishers[uav_id].publish(...)
```

### 接收同步（输入）

#### 1. 编队同步

**话题**：`/uav/formation_sync`

**接收时机**：系统启动3秒后

**处理**：
```python
def on_formation_sync(self, msg):
    # 解析编队信息
    data = json.loads(msg.data)
    
    # 更新环境初始位置
    for uav_id, uav_data in data['uavs'].items():
        pos = uav_data['px4_position_enu']
        self.env.uavs[uav_id].position = np.array([pos['x'], pos['y'], pos['z']])
    
    # 清空轨迹重新开始
    self.visualizer.clear_trajectories()
```

#### 2. 可视化命令

**话题**：`/{uav_id}/rl/visualization_cmd`

**用途**：接收其他控制源的命令用于显示

**示例**：
```json
{
  "type": "initial_position",
  "position": {"x": 5.0, "y": 3.0, "z": 2.0}
}
```

#### 3. 权威命令

**话题**：`/uav/authoritative_cmd`

**用途**：监听谁在控制无人机

**处理**：
```python
def on_authoritative_cmd(self, msg):
    data = json.loads(msg.data)
    source = data['header']['source_id']
    
    # 更新界面显示（绿色=RL控制，灰色=其他控制）
    self.current_source = source
```

## 🧠 策略开发

### 替换为你的RL算法

#### 步骤1：训练模型

```python
# train_rl.py
from stable_baselines3 import PPO
from rl_env import MultiUAVEnvironment

env = MultiUAVEnvironment(['uav1', 'uav2', 'uav3'])
model = PPO("MlpPolicy", env, verbose=1)
model.learn(total_timesteps=1000000)
model.save("my_uav_model")
```

#### 步骤2：加载模型

修改 `rl_policy.py`：

```python
import torch

class MyRLPolicy:
    def __init__(self, model_path):
        self.model = torch.load(model_path)
        self.model.eval()
    
    def get_action(self, observation, uav_id):
        obs_tensor = torch.FloatTensor(observation)
        with torch.no_grad():
            action = self.model(obs_tensor).numpy()
        return action
```

#### 步骤3：使用新策略

修改 `rl_platform_node.py`：

```python
from rl_policy import MyRLPolicy

# 在__init__中
self.policy = MyRLPolicy('my_uav_model.pth')
```

### 观测空间说明

```python
# 当前观测（9维）
observation = [
    x, y, z,              # 自身位置 (ENU)
    vx, vy, vz,           # 自身速度
    target_x, target_y, target_z  # 目标位置
]
```

**如需扩展**，在`rl_env.py`中修改：

```python
def get_observations(self):
    obs = {}
    for uav_id, uav in self.uavs.items():
        # 添加更多信息
        obs[uav_id] = np.concatenate([
            uav.position,          # 3D
            uav.velocity,          # 3D
            target,                # 3D
            nearby_obstacles,      # ND（新增）
            other_uavs_positions,  # N*3D（新增）
        ])
    return obs
```

### 动作空间说明

```python
# 当前动作（4维）
action = [vx, vy, vz, yaw_rate]

# 速度命令
vx: -2.0 ~ 2.0 m/s
vy: -2.0 ~ 2.0 m/s  
vz: -1.0 ~ 1.0 m/s
yaw_rate: -1.0 ~ 1.0 rad/s
```

## 📊 可视化界面使用

### 查看轨迹

轨迹自动显示，最多保留100个历史点。

**清空轨迹**：按`R`键重置

### 监控状态

右侧信息面板实时显示：
- 每架无人机的位置/速度
- 到目标的距离
- 当前控制源（绿色=RL在控制）
- RL决策步数

### 判断控制权

**绿色 "Control: rl"**：
- RL算法在控制无人机
- 你的决策正在生效

**灰色 "Control: central/human"**：
- 其他源抢占了控制
- RL决策被忽略（但仍在计算）

## 🔗 与多机系统集成

### 完整工作流程

```
1. 启动多机系统
   ros2 launch uav_decision_arbiter multi_uav.launch.py
   
   效果：
   - Arbiter等待决策源
   - Adapters准备就绪
   - FormationSync等待PX4数据

2. 启动RL平台
   python3 rl_platform/rl_platform_node.py
   
   效果：
   - Pygame窗口打开
   - RL开始发布决策 /{uav_id}/rl/decision_output
   - RL Adapters接收并转换
   
3. Arbiter仲裁
   - 收到RL命令（优先级100）
   - 无其他源 → RL生效
   - 发布权威命令

4. 同步到三平台
   - PX4执行RL命令（真机/SITL）
   - AirSim执行RL命令（仿真）
   - RL平台继续可视化

5. 如果中央算力介入
   - 发布 /uav2/central/decision_output
   - Arbiter: uav2切换到central（优先级150）
   - RL平台界面：uav2变灰（失去控制）
   - uav1和uav3仍然绿色（RL控制）
```

### 数据流图

```
RL Platform
    │
    ├─ 决策 → /{uav1}/rl/decision_output → RL Adapter → Arbiter
    ├─ 决策 → /{uav2}/rl/decision_output → RL Adapter → Arbiter  
    ├─ 决策 → /{uav3}/rl/decision_output → RL Adapter → Arbiter
    │
    ← 编队同步 ← /uav/formation_sync ← Formation Sync
    ← 可视化 ← /{uavX}/rl/visualization_cmd ← RL Adapter
    ← 监听 ← /uav/authoritative_cmd ← Arbiter
```

## 🧪 测试场景

### 测试1：RL可视化

```bash
# 只启动RL平台（不连接系统）
python3 rl_platform/rl_platform_node.py
```

**观察**：
- 界面正常显示
- 无人机移动
- 轨迹绘制
- 没有ROS2消息（因为系统未启动）

### 测试2：RL控制无人机

```bash
# 终端1
ros2 launch uav_decision_arbiter multi_uav.launch.py

# 终端2
python3 rl_platform/rl_platform_node.py

# 终端3：查看决策是否发布
ros2 topic echo /uav1/rl/decision_output
```

### 测试3：优先级抢占

```bash
# RL运行中，发布中央决策
ros2 topic pub /uav2/central/decision_output std_msgs/msg/String \
  'data: "{\"type\": \"position\", \"position\": {\"x\": 15, \"y\": 5, \"z\": 3, \"yaw\": 0}}"' \
  --rate 2
```

**观察RL界面**：
- uav2的"Control"变灰
- uav2轨迹改变（跟随central命令）
- uav1和uav3仍然绿色（RL控制）

### 测试4：编队同步

```bash
# 完整启动
ros2 launch uav_decision_arbiter multi_uav.launch.py
python3 examples/mock_multi_px4.py
python3 rl_platform/rl_platform_node.py
```

**观察**：
- RL平台收到编队同步消息
- 无人机位置自动初始化
- 右侧显示"Formation Info"

## 🎓 开发指南

### 修改环境

编辑 `rl_env.py`：

```python
class MultiUAVEnvironment:
    def __init__(self, uav_ids):
        # 修改初始位置
        initial_pos = np.array([x, y, z])
        
        # 修改目标位置
        self.targets[uav_id] = np.array([target_x, target_y, target_z])
    
    def compute_rewards(self):
        # 修改奖励函数
        reward = your_reward_function()
        return reward
```

### 修改策略

创建新策略文件 `my_policy.py`：

```python
class MyPolicy:
    def get_batch_actions(self, observations, time):
        actions = {}
        for uav_id, obs in observations.items():
            # 你的决策逻辑
            action = self.compute_action(obs)
            actions[uav_id] = action
        return actions
```

### 修改可视化

编辑 `rl_visualizer.py`：

```python
def draw_custom_element(self):
    """添加自定义可视化元素"""
    # 绘制障碍物
    # 绘制传感器范围
    # 绘制通信链接
    ...
```

## 📈 性能参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| control_frequency | 10 Hz | RL决策频率 |
| visualization_fps | 30 FPS | 界面刷新率 |
| max_trajectory_length | 100 | 轨迹历史点数 |
| max_speed | 2.0 m/s | 最大速度限制 |

调整频率：

```python
# rl_platform_node.py
self.declare_parameter('control_frequency', 20.0)  # 提高到20Hz
```

## 🐛 常见问题

### 问题1：pygame界面不显示

**原因**：WSL环境缺少X Server

**解决**：
1. 安装VcXsrv或XMing
2. 设置DISPLAY：`export DISPLAY=:0`
3. 或禁用可视化：
```python
self.declare_parameter('enable_visualization', False)
```

### 问题2：RL决策不生效

**检查**：
```bash
# 查看是否发布
ros2 topic echo /uav1/rl/decision_output

# 查看仲裁器状态
ros2 topic echo /uav/arbiter/status

# 确认RL优先级最高或无其他源
```

### 问题3：无人机不移动

**可能原因**：
- 已到达目标（distance < 1.0m）
- RL决策被暂停（按了SPACE）
- 动作被限幅为0

**检查**：
- 查看界面上的速度箭头
- 查看日志中的action值

## 🔧 高级配置

### 多机数量调整

```python
# 2架无人机
self.declare_parameter('uav_ids', ['uav1', 'uav2'])

# 5架无人机
self.declare_parameter('uav_ids', ['uav1', 'uav2', 'uav3', 'uav4', 'uav5'])
```

### 切换策略

```python
# 圆形编队
self.declare_parameter('policy_type', 'circle')

# 简单导航
self.declare_parameter('policy_type', 'simple')
```

### 调整目标

编辑 `rl_env.py`：

```python
# 为每架设置不同目标
self.targets = {
    'uav1': np.array([10, 10, 5]),
    'uav2': np.array([15, 5, 5]),
    'uav3': np.array([5, 15, 5])
}
```

## 📝 代码模板

### 最简单的RL策略

```python
class MinimalPolicy:
    def get_batch_actions(self, observations, time):
        actions = {}
        for uav_id, obs in observations.items():
            # 固定速度前进
            actions[uav_id] = np.array([1.0, 0.0, 0.0, 0.0])
        return actions
```

### 基于神经网络的策略

```python
import torch
import torch.nn as nn

class NeuralPolicy(nn.Module):
    def __init__(self, obs_dim=9, act_dim=4):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(obs_dim, 128),
            nn.ReLU(),
            nn.Linear(128, 64),
            nn.ReLU(),
            nn.Linear(64, act_dim),
            nn.Tanh()  # 输出[-1,1]
        )
    
    def forward(self, obs):
        return self.net(obs) * 2.0  # 缩放到[-2,2] m/s

# 使用
policy_net = NeuralPolicy()
policy_net.load_state_dict(torch.load('model.pth'))

def get_action(observation):
    with torch.no_grad():
        action = policy_net(torch.FloatTensor(observation))
    return action.numpy()
```

## 🎬 完整示例

保存为 `run_rl_demo.sh`：

```bash
#!/bin/bash
echo "完整RL平台演示"
echo "================"

# 终端1: 后台启动多机系统
gnome-terminal -- bash -c "
  cd /home/lihaomin/project/uav_sys
  source setup_env.sh
  ros2 launch uav_decision_arbiter multi_uav.launch.py
"

sleep 3

# 终端2: 后台模拟PX4
gnome-terminal -- bash -c "
  cd /home/lihaomin/project/uav_sys
  source setup_env.sh
  python3 examples/mock_multi_px4.py
"

sleep 2

# 终端3: 启动RL平台（前台，显示界面）
cd /home/lihaomin/project/uav_sys
source setup_env.sh
python3 rl_platform/rl_platform_node.py
```

## 总结

你的RL平台现在拥有：

- ✅ **完整的可视化界面**（Pygame）
  - 多架无人机实时显示
  - 轨迹、速度、状态信息
  - 交互控制

- ✅ **可运行的RL算法**
  - 简单导航策略
  - 圆形编队策略
  - 易于替换为神经网络

- ✅ **完整的ROS2集成**
  - 发送决策命令
  - 接收编队同步
  - 监听控制权变化

- ✅ **多机支持**
  - 支持任意数量无人机
  - 独立决策或协同控制
  - 自动与系统集成

**现在运行**：`python3 rl_platform/rl_platform_node.py` 体验完整的RL平台！🎉


