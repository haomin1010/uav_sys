# 多机编队系统完整使用指南

## 🎯 系统概述

你的系统现在支持**集中仲裁 + 编队相对位置同步**：

- ✅ 一个仲裁器管理多架无人机（默认3架）
- ✅ 自动坐标系转换（ENU ↔ NED）
- ✅ 保持三平台相对位置一致
- ✅ 支持广播和定向命令

## 📊 系统架构

```
         全局决策源
   ┌──────────────────────┐
   │  RL编队算法          │ 可广播或定向
   │  中央任务规划        │
   │  人类指挥控制        │
   └──────┬───────────────┘
          │
          ▼
   ┌──────────────────────┐
   │  集中Arbiter         │ 为每架无人机独立仲裁
   │  (多机管理)          │
   └──────┬───────────────┘
          │
    ┌─────┴─────┬─────────┐
    ▼           ▼         ▼
┌────────┐  ┌────────┐  ┌────────┐
│ UAV1   │  │ UAV2   │  │ UAV3   │
│ 编队长机│  │ 右翼    │  │ 左翼    │
└───┬────┘  └───┬────┘  └───┬────┘
    │           │           │
    ├─ PX4真机  ├─ PX4真机  ├─ PX4真机
    ├─ AirSim   ├─ AirSim   ├─ AirSim
    └─ RL展示   └─ RL展示   └─ RL展示
    
编队位置同步：
  PX4(ENU局部) ──→ 转换 ──→ AirSim(NED全局)
  保持相对位置一致
```

## 🚀 快速开始

### 测试模式（无需硬件）

```bash
# 终端1: 启动多机系统
cd /home/lihaomin/project/uav_sys
source setup_env.sh
ros2 launch uav_decision_arbiter multi_uav.launch.py

# 终端2: 模拟3架PX4位置
python3 examples/mock_multi_px4.py

# 观察输出：
# [formation_sync]: ✓ 编队初始位置同步完成！
# [uav1_airsim_adapter]: ✓ uav1 AirSim位置已设置(NED): (0.00, 0.00, 0.00)
# [uav2_airsim_adapter]: ✓ uav2 AirSim位置已设置(NED): (0.00, -5.00, 0.00)
# [uav3_airsim_adapter]: ✓ uav3 AirSim位置已设置(NED): (0.00, 5.00, 0.00)
```

## 📁 关键配置文件

### `config/multi_uav.yaml`

#### 1. 仲裁器（管理所有无人机）

```yaml
arbiter:
  ros__parameters:
    centralized_mode: true
    uav_ids: ["uav1", "uav2", "uav3"]  # 根据实际数量调整
```

#### 2. 编队队形配置

```yaml
formation_sync:
  ros__parameters:
    leader_uav: "uav1"  # 编队长机
    
    # 标准编队相对位置（NED，相对长机）
    formation_offsets:
      uav1: {x: 0.0, y: 0.0, z: 0.0}      # 长机
      uav2: {x: 0.0, y: -5.0, z: 0.0}     # 右翼5米
      uav3: {x: 0.0, y: 5.0, z: 0.0}      # 左翼5米
    
    # AirSim生成位置（NED全局）
    airsim_spawn_offsets:
      uav1: {x: 0.0, y: 0.0, z: 0.0}
      uav2: {x: 0.0, y: -10.0, z: 0.0}
      uav3: {x: 0.0, y: 10.0, z: 0.0}
```

**如何理解这个配置**：

```
编队相对位置 (formation_offsets):
  - 定义了标准编队队形
  - 相对于长机的位置
  - 用于保持队形一致性

AirSim生成位置 (airsim_spawn_offsets):
  - AirSim世界中的初始位置
  - 应与settings.json中Vehicles位置一致
  - 用于映射PX4局部坐标到AirSim全局坐标
```

#### 3. 各无人机适配器配置

```yaml
# UAV1的AirSim适配器
uav1_airsim_adapter:
  ros__parameters:
    uav_id: "uav1"
    vehicle_name: "Drone1"
    use_airsim_api: false  # 测试时false，实际使用true

# UAV2的AirSim适配器
uav2_airsim_adapter:
  ros__parameters:
    uav_id: "uav2"
    vehicle_name: "Drone2"
    use_airsim_api: false

# UAV3的AirSim适配器
uav3_airsim_adapter:
  ros__parameters:
    uav_id: "uav3"
    vehicle_name: "Drone3"
    use_airsim_api: false
```

## 🎨 自定义编队队形

### V字编队（战斗机队形）

```yaml
formation_offsets:
  uav1: {x: 0.0, y: 0.0, z: 0.0}
  uav2: {x: -5.0, y: -5.0, z: 0.0}  # 右后方
  uav3: {x: -5.0, y: 5.0, z: 0.0}   # 左后方
```
```
     uav1 →
    /     \
  uav2   uav3
```

### 纵队编队（一列纵队）

```yaml
formation_offsets:
  uav1: {x: 0.0, y: 0.0, z: 0.0}
  uav2: {x: -5.0, y: 0.0, z: 0.0}   # 后5米
  uav3: {x: -10.0, y: 0.0, z: 0.0}  # 后10米
```
```
  uav1 → uav2 → uav3
```

### 立体编队（3D空间）

```yaml
formation_offsets:
  uav1: {x: 0.0, y: 0.0, z: 0.0}
  uav2: {x: 0.0, y: -5.0, z: -2.0}  # 右下
  uav3: {x: 0.0, y: 5.0, z: 2.0}    # 左上
```

### 5机编队（菱形）

```yaml
arbiter:
  ros__parameters:
    uav_ids: ["uav1", "uav2", "uav3", "uav4", "uav5"]

formation_sync:
  ros__parameters:
    formation_offsets:
      uav1: {x: 0.0, y: 0.0, z: 0.0}       # 中心
      uav2: {x: 0.0, y: -5.0, z: 0.0}      # 右
      uav3: {x: 0.0, y: 5.0, z: 0.0}       # 左
      uav4: {x: 5.0, y: 0.0, z: 0.0}       # 前
      uav5: {x: -5.0, y: 0.0, z: 0.0}      # 后
```

## 🔌 实际部署

### 步骤1：准备AirSim

编辑 `AirSim/settings.json`：

```json
{
  "SettingsVersion": 1.2,
  "SimMode": "Multirotor",
  "Vehicles": {
    "Drone1": {
      "VehicleType": "SimpleFlight",
      "X": 0, "Y": 0, "Z": 0,
      "Yaw": 0
    },
    "Drone2": {
      "VehicleType": "SimpleFlight",
      "X": 0, "Y": -10, "Z": 0,
      "Yaw": 0
    },
    "Drone3": {
      "VehicleType": "SimpleFlight",
      "X": 0, "Y": 10, "Z": 0,
      "Yaw": 0
    }
  }
}
```

**关键**：Vehicle位置应与 `airsim_spawn_offsets` 一致！

### 步骤2：准备PX4

#### 方式A：SITL仿真

```bash
# UAV1
cd PX4-Autopilot
DONT_RUN=1 make px4_sitl_default gazebo
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
gazebo Tools/sitl_gazebo/worlds/empty.world

# 在新终端启动PX4实例
cd PX4-Autopilot
MAV_SYS_ID=1 PX4_SIM_MODEL=iris_0 ./build/px4_sitl_default/bin/px4

# UAV2（重复，改变ID和端口）
MAV_SYS_ID=2 PX4_SIM_MODEL=iris_1 ./build/px4_sitl_default/bin/px4

# UAV3
MAV_SYS_ID=3 PX4_SIM_MODEL=iris_2 ./build/px4_sitl_default/bin/px4
```

#### 方式B：真机

3架真实PX4飞控，分别连接到机载电脑或地面站。

### 步骤3：启动MAVROS

每架无人机一个MAVROS实例：

```bash
# UAV1 MAVROS
ros2 run mavros mavros_node --ros-args \
  -r __ns:=/uav1 \
  -p fcu_url:="udp://:14540@127.0.0.1:14557" \
  -p system_id:=1

# UAV2 MAVROS
ros2 run mavros mavros_node --ros-args \
  -r __ns:=/uav2 \
  -p fcu_url:="udp://:14541@127.0.0.1:14558" \
  -p system_id:=2

# UAV3 MAVROS
ros2 run mavros mavros_node --ros-args \
  -r __ns:=/uav3 \
  -p fcu_url:="udp://:14542@127.0.0.1:14559" \
  -p system_id:=3
```

### 步骤4：启动多机系统

```bash
source setup_env.sh
ros2 launch uav_decision_arbiter multi_uav.launch.py
```

### 步骤5：验证同步

```bash
# 查看编队同步命令
ros2 topic echo /uav/formation_sync

# 查看集中仲裁状态
ros2 topic echo /uav/arbiter/status

# 查看话题列表
ros2 topic list | grep uav
```

## 📡 话题结构（多机模式）

```
决策输入（每架独立）：
  /uav1/rl/decision_output
  /uav2/rl/decision_output
  /uav3/rl/decision_output
  
  /uav1/central/decision_output
  /uav2/central/decision_output
  /uav3/central/decision_output

统一命令（合并后）：
  /uav/source/rl/cmd
  /uav/source/central/cmd
  /uav/source/human/cmd

权威命令（带target_uav_id）：
  /uav/authoritative_cmd

编队同步：
  /uav/formation_sync

PX4位置（输入）：
  /uav1/mavros/local_position/pose
  /uav2/mavros/local_position/pose
  /uav3/mavros/local_position/pose
```

## 🎮 发布决策命令

### 方式1：广播命令（所有无人机）

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class FormationRL(Node):
    def __init__(self):
        super().__init__('formation_rl')
        
        # 发布到RL决策话题（自动广播到所有无人机）
        self.pub = self.create_publisher(
            String,
            '/rl/decision_output',  # 没有uav_id前缀=广播
            10
        )
    
    def publish_formation_command(self):
        """发布编队命令（所有无人机执行相同动作）"""
        decision = {
            "action": [1.0, 0.0, 0.5, 0.0],  # vx, vy, vz, yaw_rate
            # 不指定target_uav_id = 广播
        }
        
        msg = String()
        msg.data = json.dumps(decision)
        self.pub.publish(msg)
```

### 方式2：定向命令（指定无人机）

```python
class TaskAssigner(Node):
    def __init__(self):
        super().__init__('task_assigner')
        
        # 为每架无人机创建独立发布器
        self.pubs = {
            'uav1': self.create_publisher(String, '/uav1/central/decision_output', 10),
            'uav2': self.create_publisher(String, '/uav2/central/decision_output', 10),
            'uav3': self.create_publisher(String, '/uav3/central/decision_output', 10),
        }
    
    def assign_tasks(self):
        """为每架无人机分配不同任务"""
        tasks = {
            'uav1': {"type": "position", "position": {"x": 0, "y": 0, "z": 5}},
            'uav2': {"type": "position", "position": {"x": 10, "y": 0, "z": 5}},
            'uav3': {"type": "position", "position": {"x": 5, "y": 10, "z": 5}}
        }
        
        for uav_id, task in tasks.items():
            msg = String()
            msg.data = json.dumps(task)
            self.pubs[uav_id].publish(msg)
```

## 🧪 测试流程

### 完整测试（3架模拟PX4）

```bash
# 终端1: 启动系统
source setup_env.sh
ros2 launch uav_decision_arbiter multi_uav.launch.py

# 终端2: 模拟PX4
python3 examples/mock_multi_px4.py

# 终端3: 发布RL编队命令
ros2 topic pub /rl/decision_output std_msgs/msg/String \
  'data: "{\"action\": [1.0, 0.0, 0.0, 0.0]}"' --rate 10

# 观察：
# 1. formation_sync同步编队位置
# 2. arbiter为每架无人机独立仲裁
# 3. 所有选择RL源（因为没有其他源）
```

### 测试优先级抢占（多机场景）

```bash
# 启动RL命令（所有无人机）
ros2 topic pub /rl/decision_output std_msgs/msg/String \
  'data: "{\"action\": [1.0, 0.0, 0.0, 0.0]}"' --rate 10

# 中央算力接管UAV2
ros2 topic pub /uav2/central/decision_output std_msgs/msg/String \
  'data: "{\"type\": \"position\", \"position\": {\"x\": 10, \"y\": 5, \"z\": 3, \"yaw\": 0}}"' \
  --rate 1

# 结果：
#   uav1: RL控制（优先级100）
#   uav2: Central控制（优先级150，抢占RL）
#   uav3: RL控制（优先级100）
```

## 📊 状态监控

### 查看仲裁器状态（多机）

```bash
ros2 topic echo /uav/arbiter/status
```

**输出格式**：
```json
{
  "mode": "centralized",
  "uav_count": 3,
  "uavs": {
    "uav1": {
      "current_source": "rl",
      "sources": {
        "rl": {"has_cmd": true, "cmd_valid": true},
        "central": {"has_cmd": false},
        "human": {"has_cmd": false}
      }
    },
    "uav2": {
      "current_source": "central",  ← 被抢占
      "sources": {...}
    },
    "uav3": {
      "current_source": "rl",
      "sources": {...}
    }
  }
}
```

### 查看编队同步

```bash
ros2 topic echo /uav/formation_sync
```

**关键字段**：
- `px4_position_enu`: PX4原始位置（ENU）
- `px4_position_ned`: 转换后位置（NED）
- `airsim_target_position`: AirSim目标位置（NED全局）
- `formation_offset`: 编队偏移量

## 🎓 集成到你的系统

### RL平台集成

你的RL算法需要发布决策到相应话题：

```python
class MyRLPlatform(Node):
    def __init__(self):
        super().__init__('my_rl_platform')
        
        # 根据需求选择广播或定向
        
        # 方式1：广播（编队统一动作）
        self.pub_broadcast = self.create_publisher(
            String,
            '/rl/decision_output',  # 所有无人机
            10
        )
        
        # 方式2：定向（每机独立控制）
        self.pubs = {
            'uav1': self.create_publisher(String, '/uav1/rl/decision_output', 10),
            'uav2': self.create_publisher(String, '/uav2/rl/decision_output', 10),
            'uav3': self.create_publisher(String, '/uav3/rl/decision_output', 10),
        }
    
    def publish_formation_control(self):
        """编队统一控制"""
        decision = {"action": [vx, vy, vz, yaw_rate]}
        self.pub_broadcast.publish(json.dumps(decision))
    
    def publish_individual_control(self):
        """每机独立控制"""
        for uav_id, action in self.compute_actions().items():
            decision = {"action": action}
            self.pubs[uav_id].publish(json.dumps(decision))
```

### 接收编队同步信息

```python
class MyRLPlatform(Node):
    def __init__(self):
        super().__init__('my_rl_platform')
        
        # 订阅编队同步命令
        self.sub = self.create_subscription(
            String,
            '/uav/formation_sync',
            self.on_formation_sync,
            10
        )
    
    def on_formation_sync(self, msg):
        """接收编队初始位置"""
        data = json.loads(msg.data)
        
        if data['type'] == 'formation_sync':
            print("收到编队同步：")
            for uav_id, info in data['uavs'].items():
                pos = info['px4_position_enu']
                offset = info['formation_offset']
                print(f"  {uav_id}: 位置={pos}, 编队偏移={offset}")
            
            # 使用这些信息初始化你的RL环境
            self.reset_environment(data['uavs'])
```

## 💻 完整示例代码

保存为 `examples/multi_uav_rl_example.py`：

```python
#!/usr/bin/env python3
"""
多机RL控制示例
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
import math

class MultiUAVRLController(Node):
    def __init__(self):
        super().__init__('multi_uav_rl_controller')
        
        # 创建每架无人机的发布器
        self.pubs = {}
        for uav_id in ['uav1', 'uav2', 'uav3']:
            self.pubs[uav_id] = self.create_publisher(
                String,
                f'/{uav_id}/rl/decision_output',
                10
            )
        
        # 订阅编队同步（获取初始位置）
        self.sub_formation = self.create_subscription(
            String,
            '/uav/formation_sync',
            self.on_formation_sync,
            10
        )
        
        # 定时发布控制命令
        self.timer = self.create_timer(0.1, self.publish_controls)
        self.counter = 0
        
        self.get_logger().info('多机RL控制器已启动')
    
    def on_formation_sync(self, msg):
        """接收编队同步"""
        data = json.loads(msg.data)
        self.get_logger().info('收到编队同步信息：')
        for uav_id, info in data['uavs'].items():
            self.get_logger().info(f'  {uav_id}: {info["px4_position_enu"]}')
    
    def publish_controls(self):
        """发布控制命令"""
        self.counter += 1
        t = self.counter * 0.1
        
        # 示例：圆形编队运动
        vx = 1.0 * math.cos(t * 0.5)
        vy = 1.0 * math.sin(t * 0.5)
        
        for uav_id in self.pubs.keys():
            decision = {
                "action": [vx, vy, 0.0, 0.2],
                "timestamp": time.time()
            }
            
            msg = String()
            msg.data = json.dumps(decision)
            self.pubs[uav_id].publish(msg)
        
        if self.counter % 10 == 0:
            self.get_logger().info(f'发布编队控制命令 #{self.counter}')

def main():
    rclpy.init()
    node = MultiUAVRLController()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
```

## 📖 相关文档

- **`FORMATION_SYNC.md`** (本文件) - 编队同步详解
- **`CENTRALIZED_ARCHITECTURE.md`** - 集中仲裁架构
- **`config/multi_uav.yaml`** - 多机配置文件
- **`launch/multi_uav.launch.py`** - 多机启动文件

## 🔧 常用命令

```bash
# 启动多机系统
ros2 launch uav_decision_arbiter multi_uav.launch.py

# 查看所有节点
ros2 node list
# 输出：/arbiter, /formation_sync, /uav1_rl_adapter, /uav2_rl_adapter, ...

# 查看话题
ros2 topic list | grep uav

# 查看编队状态
ros2 topic echo /uav/formation_sync --once

# 查看某架无人机的仲裁状态
ros2 topic echo /uav/arbiter/status | grep -A 10 uav2

# 手动发布测试命令
ros2 topic pub /uav2/central/decision_output std_msgs/msg/String \
  'data: "{\"type\": \"position\", \"position\": {\"x\": 5, \"y\": 3, \"z\": 2, \"yaw\": 0}}"'
```

## ✅ 验证清单

- [ ] 配置文件中uav_ids正确
- [ ] AirSim settings.json中Vehicles配置正确
- [ ] MAVROS命名空间匹配（/uav1, /uav2, /uav3）
- [ ] formation_offsets符合预期队形
- [ ] airsim_spawn_offsets与AirSim一致
- [ ] 启动日志显示"编队初始位置同步完成"
- [ ] AirSim中无人机位置正确

## 总结

系统现在支持：
- ✅ **集中仲裁**：一个Arbiter管理多机
- ✅ **编队同步**：自动保持相对位置一致
- ✅ **坐标转换**：ENU ↔ NED自动处理
- ✅ **灵活部署**：支持2-10+架无人机
- ✅ **双模式**：可切换单机/多机模式

开始你的多机编队飞行吧！🚁🚁🚁

