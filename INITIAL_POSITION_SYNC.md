# 初始位置同步功能使用指南

## 功能说明

此功能实现了**启动时初始化同步**，让AirSim仿真无人机和RL平台的起始位置与真实PX4飞控保持一致。

### 工作流程

```
┌──────────────┐
│ PX4真机/SITL │
│  (MAVROS)    │
└──────┬───────┘
       │ 启动后2秒
       │ 读取当前位置
       ▼
 /uav/initial_position
       │
       ├──────────────┬──────────────┐
       ▼              ▼              ▼
┌────────────┐  ┌──────────┐  ┌──────────┐
│ AirSim     │  │ RL平台   │  │ 监控显示  │
│ Adapter    │  │ Adapter  │  │          │
└────────────┘  └──────────┘  └──────────┘
   │                │
   │ 设置位置       │ 通知初始位置
   ▼                ▼
AirSim仿真    RL环境初始化
```

## 配置说明

所有配置都在 `config/default.yaml` 中：

### PX4 Adapter（发布者）

```yaml
px4_adapter:
  ros__parameters:
    publish_initial_position: true   # 是否发布初始位置
    initial_position_delay: 2.0      # 启动后等待MAVROS数据的延迟（秒）
```

- `publish_initial_position`: 设为`false`可禁用此功能
- `initial_position_delay`: 等待时间，确保MAVROS数据已准备好

### AirSim Adapter（接收者）

```yaml
airsim_adapter:
  ros__parameters:
    sync_initial_position: true      # 是否同步初始位置
    use_airsim_api: false            # 是否实际连接AirSim
```

- `sync_initial_position`: 是否订阅并设置初始位置
- `use_airsim_api`: 设为`true`时才会实际调用AirSim API设置位置

### RL Adapter（接收者）

```yaml
rl_adapter:
  ros__parameters:
    sync_initial_position: true      # 是否同步初始位置
```

- RL Adapter会接收初始位置并发送到 `/rl/visualization_cmd`
- 你的RL平台可以订阅这个话题来获取初始位置信息

## 使用方法

### 场景1：完整系统（有PX4/MAVROS）

```bash
# 1. 确保PX4 SITL或真机已连接
# 2. 启动MAVROS（如果使用真机）
ros2 run mavros mavros_node --ros-args \
  -p fcu_url:="udp://:14540@127.0.0.1:14557"

# 3. 启动完整系统
source setup_env.sh
ros2 launch uav_decision_arbiter system.launch.py
```

**预期结果**：
- PX4 Adapter启动2秒后，发布初始位置到 `/uav/initial_position`
- AirSim Adapter接收并设置仿真无人机位置（如果启用）
- RL Adapter接收并转发给RL平台

### 场景2：测试模式（无PX4）

```bash
# 1. 启动系统
source setup_env.sh
ros2 launch uav_decision_arbiter system.launch.py

# 2. 在新终端，手动发布初始位置
python3 examples/test_initial_position.py
```

**观察输出**：
```
[airsim_adapter]: 收到初始位置: source=test_px4, pos=(0.00, 0.00, 0.00)
[airsim_adapter]: AirSim API未启用，仅记录初始位置（不实际设置）

[rl_adapter]: 收到初始位置: source=test_px4, pos=(0.00, 0.00, 0.00)
[rl_adapter]: ✓ 初始位置已发送给RL平台
```

## 话题接口

### `/uav/initial_position` (发布)

**类型**: `std_msgs/String`  
**格式**: JSON

```json
{
  "timestamp": 1762247595.123,
  "source": "px4",
  "position": {
    "x": 0.0,
    "y": 0.0,
    "z": 0.0
  },
  "orientation": {
    "w": 1.0,
    "x": 0.0,
    "y": 0.0,
    "z": 0.0
  }
}
```

### 监控命令

```bash
# 查看初始位置消息
ros2 topic echo /uav/initial_position

# 查看AirSim状态
ros2 topic echo /uav/airsim/state

# 查看RL状态
ros2 topic echo /uav/rl/state
```

## 集成到你的RL平台

你的RL代码需要订阅 `/rl/visualization_cmd` 来接收初始位置：

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class MyRLPlatform(Node):
    def __init__(self):
        super().__init__('my_rl_platform')
        
        # 订阅可视化/初始化命令
        self.sub_viz = self.create_subscription(
            String,
            '/rl/visualization_cmd',
            self.on_viz_cmd,
            10
        )
    
    def on_viz_cmd(self, msg):
        data = json.loads(msg.data)
        
        if data.get('type') == 'initial_position':
            # 接收到初始位置
            pos = data['position']
            self.reset_environment(
                x=pos['x'],
                y=pos['y'],
                z=pos['z']
            )
            print(f'RL环境已重置到初始位置: ({pos["x"]}, {pos["y"]}, {pos["z"]})')
```

## AirSim集成

如果要实际控制AirSim仿真：

### 1. 安装AirSim

```bash
conda activate uav_sys
pip install airsim
```

### 2. 启动AirSim

启动虚幻引擎中的AirSim仿真环境。

### 3. 修改配置

编辑 `config/default.yaml`：

```yaml
airsim_adapter:
  ros__parameters:
    use_airsim_api: true            # 启用API
    airsim_ip: "127.0.0.1"          # AirSim地址
    vehicle_name: "Drone1"          # 飞机名称
    sync_initial_position: true
```

### 4. 启动系统

```bash
source setup_env.sh
ros2 launch uav_decision_arbiter system.launch.py
```

**预期结果**：
- AirSim中的无人机会自动移动到PX4的当前位置
- 日志中会显示：`✓ AirSim位置已设置: (x, y, z)`

## 故障排除

### 问题1：PX4未发布初始位置

**检查**：
```bash
ros2 topic echo /uav/initial_position
```

**原因**：
- `publish_initial_position: false` - 功能被禁用
- MAVROS未连接 - 检查 `/mavros/local_position/pose`
- 延迟时间太短 - 增加 `initial_position_delay`

**解决**：
```bash
# 检查MAVROS话题
ros2 topic list | grep mavros

# 查看PX4 Adapter日志
ros2 node info /px4_adapter
```

### 问题2：AirSim位置未改变

**原因**：
- `use_airsim_api: false` - API未启用
- AirSim未运行
- 连接失败

**解决**：
1. 确认AirSim正在运行
2. 检查配置：`use_airsim_api: true`
3. 查看日志中的连接错误

### 问题3：RL平台未收到初始位置

**检查**：
```bash
# 确认RL Adapter正在运行
ros2 node list | grep rl_adapter

# 查看RL可视化话题
ros2 topic echo /rl/visualization_cmd
```

**确认**：你的RL代码是否订阅了 `/rl/visualization_cmd`

## 扩展功能

### 手动触发位置同步

如果需要在运行时重新同步位置，可以手动发布：

```bash
ros2 topic pub /uav/initial_position std_msgs/msg/String \
  'data: "{\"timestamp\": 1762247595.0, \"source\": \"manual\", \"position\": {\"x\": 5.0, \"y\": 3.0, \"z\": 2.0}, \"orientation\": {\"w\": 1.0, \"x\": 0.0, \"y\": 0.0, \"z\": 0.0}}"' \
  --once
```

### 禁用功能

如果不需要初始位置同步，可以在配置中禁用：

```yaml
px4_adapter:
  ros__parameters:
    publish_initial_position: false

airsim_adapter:
  ros__parameters:
    sync_initial_position: false

rl_adapter:
  ros__parameters:
    sync_initial_position: false
```

## 总结

这个功能实现了**简单、可靠**的初始位置同步：
- ✅ 启动时自动同步
- ✅ 配置灵活，可按需启用/禁用
- ✅ 支持测试模式（无需真实硬件）
- ✅ 易于集成到现有RL平台

如有问题，参考 `TROUBLESHOOTING.md` 或查看节点日志。

