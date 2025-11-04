# 编队相对位置同步功能

## 🎯 功能概述

该功能确保多架无人机在**AirSim仿真**、**RL平台**和**PX4真机**三个平台之间保持**相同的相对位置关系**（编队队形）。

## 核心特性

- ✅ **坐标系自动转换**：ENU（MAVROS）↔ NED（AirSim）
- ✅ **相对位置保持**：基于配置的标准编队队形
- ✅ **启动时同步**：自动读取PX4并同步到AirSim/RL
- ✅ **实时监控**：检测编队偏差并告警
- ✅ **灵活配置**：支持任意编队队形

## 📐 坐标系说明

### MAVROS (PX4) - ENU坐标系
```
      ↑ Z (Up)
      |
      |
      o----→ X (East)
     /
    ↙ Y (North)
```
- 原点：各机独立起飞点
- 右手坐标系

### AirSim - NED坐标系
```
      X (North)
      ↑
      |
      o----→ Y (East)
     /
    ↙ Z (Down)
```
- 原点：虚幻引擎PlayerStart位置
- 全局统一坐标系

### 转换关系
```
NED.x = ENU.y    (North = ENU_North)
NED.y = ENU.x    (East = ENU_East)
NED.z = -ENU.z   (Down = -Up)
```

## 🛠️ 配置说明

### 编队队形配置

在 `config/multi_uav.yaml` 中定义：

```yaml
formation_sync:
  ros__parameters:
    leader_uav: "uav1"  # 编队长机
    
    # 编队标准队形（NED坐标系，相对于长机）
    formation_offsets:
      uav1: {x: 0.0, y: 0.0, z: 0.0}       # 长机
      uav2: {x: 0.0, y: -5.0, z: 0.0}      # 右翼5米
      uav3: {x: 0.0, y: 5.0, z: 0.0}       # 左翼5米
```

**队形示例**：

#### 1. 一字队形（横向）
```yaml
formation_offsets:
  uav1: {x: 0.0, y: 0.0, z: 0.0}
  uav2: {x: 0.0, y: -5.0, z: 0.0}  # 右边5米
  uav3: {x: 0.0, y: 5.0, z: 0.0}   # 左边5米
```
```
  uav3 ---- uav1(长机) ---- uav2
   西         中心          东
```

#### 2. 纵队队形
```yaml
formation_offsets:
  uav1: {x: 0.0, y: 0.0, z: 0.0}
  uav2: {x: -5.0, y: 0.0, z: 0.0}  # 后方5米
  uav3: {x: -10.0, y: 0.0, z: 0.0} # 后方10米
```
```
  uav1(长机) → 飞行方向
  uav2
  uav3
```

#### 3. V字队形
```yaml
formation_offsets:
  uav1: {x: 0.0, y: 0.0, z: 0.0}
  uav2: {x: -3.0, y: -3.0, z: 0.0}  # 右后方
  uav3: {x: -3.0, y: 3.0, z: 0.0}   # 左后方
```
```
     uav1(长机)
    /     \
  uav2   uav3
```

#### 4. 立体编队（不同高度）
```yaml
formation_offsets:
  uav1: {x: 0.0, y: 0.0, z: 0.0}
  uav2: {x: 0.0, y: -5.0, z: -2.0}  # 右下方
  uav3: {x: 0.0, y: 5.0, z: 2.0}    # 左上方
```

### AirSim生成位置配置

```yaml
formation_sync:
  ros__parameters:
    # AirSim中无人机的初始spawn位置（NED全局坐标）
    airsim_spawn_offsets:
      uav1: {x: 0.0, y: 0.0, z: 0.0}
      uav2: {x: 0.0, y: -10.0, z: 0.0}
      uav3: {x: 0.0, y: 10.0, z: 0.0}
```

**说明**：
- 这是AirSim虚幻引擎中无人机的生成位置
- 需要与AirSim settings.json中配置的Vehicles对应
- 用于将PX4局部坐标映射到AirSim全局坐标

## 🔄 同步工作流程

### 启动时初始化

```
步骤1: 启动系统
  ├─ formation_sync节点启动
  ├─ 等待3秒（让MAVROS连接）
  └─ 订阅所有PX4位置话题

步骤2: 读取PX4位置（每架独立ENU坐标）
  UAV1 PX4: (0.0, 0.0, 0.0) ENU  ← 起飞点1
  UAV2 PX4: (0.0, 0.0, 0.0) ENU  ← 起飞点2（不同位置）
  UAV3 PX4: (0.0, 0.0, 0.0) ENU  ← 起飞点3

步骤3: 计算编队位置
  以UAV1（长机）为参考：
  - UAV1标准位置 = 长机位置 + (0, 0, 0)
  - UAV2标准位置 = 长机位置 + (0, -5, 0) NED
  - UAV3标准位置 = 长机位置 + (0, 5, 0) NED

步骤4: 映射到AirSim全局坐标
  UAV1 → AirSim Drone1: spawn(0,0,0) + offset(0,0,0) = (0,0,0)
  UAV2 → AirSim Drone2: spawn(0,-10,0) + offset(0,-5,0) = (0,-15,0)
  UAV3 → AirSim Drone3: spawn(0,10,0) + offset(0,5,0) = (0,15,0)

步骤5: 设置AirSim位置
  调用 simSetVehiclePose() 设置每架无人机位置

结果: 三个平台相对位置一致！
```

### 实际运行示例

假设真实场景：
```
真实情况（PX4 ENU局部坐标）：
  UAV1起飞点: GPS(lat1, lon1) → PX4读到 (0, 0, 0)
  UAV2起飞点: GPS(lat2, lon2) → PX4读到 (0, 0, 0)（不同起飞点）
  UAV3起飞点: GPS(lat3, lon3) → PX4读到 (0, 0, 0)

同步后AirSim（NED全局坐标）：
  Drone1: (0, 0, 0)
  Drone2: (0, -5, 0)  ← 保持右翼5米
  Drone3: (0, 5, 0)   ← 保持左翼5米

相对距离：
  Distance(Drone1, Drone2) = 5米 ✓
  Distance(Drone1, Drone3) = 5米 ✓
  Distance(Drone2, Drone3) = 10米 ✓
```

## 📝 使用方法

### 1. 启动多机编队系统

```bash
source setup_env.sh
ros2 launch uav_decision_arbiter multi_uav.launch.py
```

**日志输出**：
```
[formation_sync]: 编队位置同步节点已启动
[formation_sync]: 管理无人机: ['uav1', 'uav2', 'uav3']
[formation_sync]: 编队长机: uav1
[formation_sync]: 编队队形: {...}
... 等待3秒 ...
[formation_sync]: 开始执行编队初始位置同步...
[formation_sync]: 长机uav1 PX4位置(ENU): (0.00, 0.00, 0.00)
[formation_sync]: uav1: PX4_ENU=(0.00, 0.00, 0.00) → AirSim_NED=(0.00, 0.00, 0.00)
[formation_sync]: uav2: PX4_ENU=(0.00, 0.00, 0.00) → AirSim_NED=(0.00, -5.00, 0.00)
[formation_sync]: uav3: PX4_ENU=(0.00, 0.00, 0.00) → AirSim_NED=(0.00, 5.00, 0.00)
[formation_sync]: ✓ 编队初始位置同步完成！

[uav1_airsim_adapter]: 收到编队同步命令: uav1
[uav1_airsim_adapter]: ✓ uav1 AirSim位置已设置(NED): (0.00, 0.00, 0.00)
[uav2_airsim_adapter]: 收到编队同步命令: uav2
[uav2_airsim_adapter]: ✓ uav2 AirSim位置已设置(NED): (0.00, -5.00, 0.00)
[uav3_airsim_adapter]: 收到编队同步命令: uav3
[uav3_airsim_adapter]: ✓ uav3 AirSim位置已设置(NED): (0.00, 5.00, 0.00)
```

### 2. 查看编队同步命令

```bash
ros2 topic echo /uav/formation_sync
```

### 3. 自定义编队队形

编辑 `config/multi_uav.yaml`：

```yaml
formation_sync:
  ros__parameters:
    formation_offsets:
      uav1: {x: 0.0, y: 0.0, z: 0.0}
      uav2: {x: -5.0, y: -3.0, z: 0.0}  # 你的自定义队形
      uav3: {x: -5.0, y: 3.0, z: 0.0}
```

重启系统即可。

## 🔌 集成实际硬件

### AirSim配置

#### 1. AirSim settings.json

```json
{
  "SettingsVersion": 1.2,
  "SimMode": "Multirotor",
  "Vehicles": {
    "Drone1": {
      "VehicleType": "SimpleFlight",
      "X": 0, "Y": 0, "Z": 0
    },
    "Drone2": {
      "VehicleType": "SimpleFlight",
      "X": 0, "Y": -10, "Z": 0
    },
    "Drone3": {
      "VehicleType": "SimpleFlight",
      "X": 0, "Y": 10, "Z": 0
    }
  }
}
```

**注意**：`airsim_spawn_offsets`应与settings.json中的位置一致！

#### 2. 启用AirSim API

编辑 `config/multi_uav.yaml`：

```yaml
uav1_airsim_adapter:
  ros__parameters:
    use_airsim_api: true
    vehicle_name: "Drone1"

uav2_airsim_adapter:
  ros__parameters:
    use_airsim_api: true
    vehicle_name: "Drone2"

uav3_airsim_adapter:
  ros__parameters:
    use_airsim_api: true
    vehicle_name: "Drone3"
```

### PX4配置（MAVROS）

每架PX4需要独立的MAVROS实例：

```bash
# UAV1
ros2 run mavros mavros_node --ros-args \
  --remap __ns:=/uav1 \
  -p fcu_url:="udp://:14540@127.0.0.1:14557"

# UAV2
ros2 run mavros mavros_node --ros-args \
  --remap __ns:=/uav2 \
  -p fcu_url:="udp://:14541@127.0.0.1:14558"

# UAV3
ros2 run mavros mavros_node --ros-args \
  --remap __ns:=/uav3 \
  -p fcu_url:="udp://:14542@127.0.0.1:14559"
```

## 💡 工作原理详解

### 示例：3机V字编队

#### 配置
```yaml
formation_offsets:
  uav1: {x: 0.0, y: 0.0, z: 0.0}      # 长机
  uav2: {x: -3.0, y: -3.0, z: 0.0}    # 右后方
  uav3: {x: -3.0, y: 3.0, z: 0.0}     # 左后方
```

#### 真实情况（PX4 ENU）
```
UAV1起飞点A: GPS(47.123°, 8.456°)
  └─ PX4本地坐标: (0, 0, 0) ENU

UAV2起飞点B: GPS(47.124°, 8.457°)（距离A约10米）
  └─ PX4本地坐标: (0, 0, 0) ENU

UAV3起飞点C: GPS(47.122°, 8.455°)（距离A约10米）
  └─ PX4本地坐标: (0, 0, 0) ENU
```

#### 同步过程

**Step 1**: 读取长机UAV1位置
```
UAV1_PX4_ENU: (0, 0, 0)
↓ 转换
UAV1_NED: (0, 0, 0)
```

**Step 2**: 计算编队位置（NED）
```
UAV1编队位置 = UAV1_NED + offset(0, 0, 0) = (0, 0, 0)
UAV2编队位置 = UAV1_NED + offset(-3, -3, 0) = (-3, -3, 0)
UAV3编队位置 = UAV1_NED + offset(-3, 3, 0) = (-3, 3, 0)
```

**Step 3**: 映射到AirSim全局坐标
```
假设airsim_spawn_offsets:
  uav1: (0, 0, 0)
  uav2: (0, -10, 0)
  uav3: (0, 10, 0)

AirSim位置：
  Drone1 = spawn(0,0,0) + formation(0,0,0) = (0, 0, 0)
  Drone2 = spawn(0,-10,0) + formation(-3,-3,0) = (-3, -13, 0)
  Drone3 = spawn(0,10,0) + formation(-3,3,0) = (-3, 13, 0)
```

**Step 4**: 验证相对位置
```
Distance(Drone1, Drone2) = sqrt(9+169) ≈ 4.24米 ✓
Distance(Drone1, Drone3) = sqrt(9+169) ≈ 4.24米 ✓
相对位置关系正确！
```

## 📊 监控编队状态

### 查看同步命令
```bash
ros2 topic echo /uav/formation_sync
```

**输出**：
```json
{
  "type": "formation_sync",
  "timestamp": 1762247600.123,
  "leader_uav": "uav1",
  "uavs": {
    "uav1": {
      "px4_position_enu": {"x": 0, "y": 0, "z": 0},
      "px4_position_ned": {"x": 0, "y": 0, "z": 0},
      "airsim_target_position": {"x": 0, "y": 0, "z": 0},
      "formation_offset": {"x": 0, "y": 0, "z": 0}
    },
    "uav2": {...},
    "uav3": {...}
  }
}
```

### 监控编队偏差

Formation Sync节点会自动监控，如果实际相对位置偏离标准超过1米：

```
[formation_sync]: 编队偏差: uav2 相对uav1 偏离标准位置 1.5米
```

## 🧪 测试（无需真实硬件）

创建测试脚本模拟多机PX4位置：

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class MockPX4Publisher(Node):
    def __init__(self, uav_id):
        super().__init__(f'mock_px4_{uav_id}')
        self.pub = self.create_publisher(
            PoseStamped,
            f'/{uav_id}/mavros/local_position/pose',
            10
        )
        self.timer = self.create_timer(0.1, self.publish_position)
        self.uav_id = uav_id
    
    def publish_position(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        
        # 模拟位置（ENU）
        msg.pose.position.x = 0.0
        msg.pose.position.y = 0.0
        msg.pose.position.z = 0.0
        
        msg.pose.orientation.w = 1.0
        
        self.pub.publish(msg)

def main():
    rclpy.init()
    # 启动3个模拟PX4发布器
    nodes = [
        MockPX4Publisher('uav1'),
        MockPX4Publisher('uav2'),
        MockPX4Publisher('uav3')
    ]
    
    try:
        executor = rclpy.executors.MultiThreadedExecutor()
        for node in nodes:
            executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
```

运行测试：
```bash
# 终端1: 启动多机系统
ros2 launch uav_decision_arbiter multi_uav.launch.py

# 终端2: 模拟PX4位置
python3 mock_px4_positions.py

# 观察formation_sync节点的输出
```

## 🎮 实际使用场景

### 场景1：3架真机编队飞行

```
准备：
1. 3架PX4无人机分别起飞并悬停
2. 各自位置不同（GPS不同）
3. 启动MAVROS连接

启动系统：
ros2 launch uav_decision_arbiter multi_uav.launch.py

效果：
- AirSim中显示3架无人机按标准队形排列
- RL平台收到编队信息
- 后续飞行时保持队形同步
```

### 场景2：仿真测试

```
准备：
1. 启动AirSim（虚幻引擎）
2. 配置3架Drone在settings.json
3. PX4 SITL × 3

启动：
ros2 launch uav_decision_arbiter multi_uav.launch.py

效果：
- AirSim仿真无人机自动调整到标准编队位置
- 可视化编队效果
```

### 场景3：混合部署

```
- UAV1: 真机PX4 + AirSim仿真
- UAV2: 真机PX4 + AirSim仿真  
- UAV3: SITL + AirSim仿真

所有位置同步，实现真实+仿真混合测试
```

## ⚙️ 高级配置

### 动态调整编队

```bash
# 在运行时手动发布编队同步命令
ros2 topic pub /uav/formation_sync std_msgs/msg/String \
  'data: "{\"type\": \"formation_sync\", \"uavs\": {...}}"' --once
```

### 禁用编队同步

```yaml
formation_sync:
  ros__parameters:
    sync_on_startup: false  # 禁用启动时同步
```

### 2机编队

```yaml
arbiter:
  ros__parameters:
    uav_ids: ["uav1", "uav2"]  # 只有2架

formation_sync:
  ros__parameters:
    formation_offsets:
      uav1: {x: 0.0, y: 0.0, z: 0.0}
      uav2: {x: 0.0, y: -5.0, z: 0.0}
```

## 📐 坐标系转换示例

### ENU → NED转换
```python
from uav_decision_arbiter.coordinate_utils import enu_to_ned

enu = {'x': 10.0, 'y': 5.0, 'z': 2.0}  # East=10, North=5, Up=2
ned = enu_to_ned(enu)
# 结果: {'x': 5.0, 'y': 10.0, 'z': -2.0}  # North=5, East=10, Down=-2
```

### NED → ENU转换
```python
from uav_decision_arbiter.coordinate_utils import ned_to_enu

ned = {'x': 5.0, 'y': 10.0, 'z': -2.0}
enu = ned_to_enu(ned)
# 结果: {'x': 10.0, 'y': 5.0, 'z': 2.0}
```

## 🐛 故障排除

### 问题1：AirSim位置未设置

**检查**：
1. `use_airsim_api: true`
2. AirSim正在运行
3. vehicle_name匹配settings.json

### 问题2：编队偏差大

**原因**：
- PX4位置数据未就绪
- 配置的偏移量不正确
- 坐标系转换问题

**解决**：
- 增加 `startup_delay` （给MAVROS更多时间）
- 检查 `formation_offsets` 配置
- 查看日志中的转换结果

### 问题3：某架无人机位置缺失

**检查MAVROS话题**：
```bash
ros2 topic list | grep mavros
ros2 topic echo /uav1/mavros/local_position/pose
```

## 📚 相关文档

- `CENTRALIZED_ARCHITECTURE.md` - 集中仲裁架构
- `INITIAL_POSITION_SYNC.md` - 单机位置同步
- `README.md` - 系统总体文档

## 总结

该功能实现了**智能的编队相对位置同步**：

- ✅ 自动坐标系转换（ENU ↔ NED）
- ✅ 配置化编队队形
- ✅ 启动时自动同步
- ✅ 支持任意数量无人机
- ✅ 实时偏差监控

让多机在三个平台中保持完美队形！🚁🚁🚁

