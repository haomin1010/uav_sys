# AirSim集成指南

## 📌 快速概览

**好消息**：AirSim Adapter代码**已100%完成**，只需**3步配置**即可对接！

### 数据流

```
你的中央算力代码
    ↓ 发布ROS2消息: /{uav_id}/central/decision_output
AirSim Adapter (已实现)
    ↓ 自动调用AirSim API
AirSim仿真环境
```

---

## ⚡ 3步快速对接

### 步骤1：安装airsim包（30秒）

```bash
conda activate uav_sys
pip install airsim
```

### 步骤2：配置AirSim（1分钟）

```bash
# 复制配置文件
cp airsim_config/settings.json ~/Documents/AirSim/

# 重启AirSim（虚幻引擎）使配置生效
```

### 步骤3：启用API（30秒）

编辑 `config/multi_uav.yaml`，将所有adapter的API开关改为true：

```yaml
uav1_airsim_adapter:
  ros__parameters:
    use_airsim_api: true      # ← 改为true

uav2_airsim_adapter:
  ros__parameters:
    use_airsim_api: true      # ← 改为true

uav3_airsim_adapter:
  ros__parameters:
    use_airsim_api: true      # ← 改为true
```

---

## 🧪 验证对接

### 自动测试（推荐）

```bash
./test_airsim_integration.sh
```

### 手动测试

```bash
# 1. 启动AirSim（虚幻引擎），选择Multirotor模式

# 2. 测试连接
python3 airsim_config/test_airsim_connection.py
```

**成功标志**：
```
✓ 成功连接到AirSim
✓ Drone1: 位置=(0.00, 0.00, 0.00)
✓ Drone2: 位置=(0.00, -10.00, 0.00)
✓ Drone3: 位置=(0.00, 10.00, 0.00)
```

---

## 🚀 运行系统

### 启动完整系统

```bash
# 终端1: 启动主系统
ros2 launch uav_decision_arbiter multi_uav.launch.py

# 终端2: 启动中央算力示例
python3 examples/central_planner_airsim.py
```

**观察AirSim窗口**：无人机会按waypoints飞行，每3秒切换目标。

---

## 💻 如何编写中央算力代码

### 上游接口（你的代码 → Adapter）

你只需发布ROS2消息，**非常简单**：

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class MyCentralPlanner(Node):
    def __init__(self):
        super().__init__('my_planner')
        
        # 为每架无人机创建发布器
        self.pub = self.create_publisher(
            String, 
            '/uav1/central/decision_output',  # uav1的决策话题
            10
        )
        
        # 定时发布决策
        self.timer = self.create_timer(0.1, self.send_decision)
    
    def send_decision(self):
        # 你的规划算法
        target = your_algorithm()  # 你的逻辑
        
        # 构建决策消息（NED坐标系）
        decision = {
            "type": "position",  # 位置控制
            "position": {
                "x": target[0],   # North (米)
                "y": target[1],   # East (米)
                "z": -5.0,        # Up 5米 (注意：负值向上)
                "yaw": 0.0        # 弧度
            }
        }
        
        # 发布
        msg = String()
        msg.data = json.dumps(decision)
        self.pub.publish(msg)

# 运行
rclpy.init()
node = MyCentralPlanner()
rclpy.spin(node)
```

**就这么简单！** Adapter会自动处理后续所有工作。

### 下游接口（Adapter → AirSim）

**无需编写**，Adapter已自动实现：

```python
# 在airsim_adapter.py中已实现
def execute_airsim_command(self, cmd):
    if cmd.body.velocity:
        # 速度控制
        self.airsim_client.moveByVelocityAsync(vx, vy, vz, ...)
    
    elif cmd.body.setpoint:
        # 位置控制
        self.airsim_client.moveToPositionAsync(x, y, z, ...)
    
    elif cmd.body.trajectory:
        # 轨迹控制
        self.airsim_client.moveOnPathAsync(path, ...)
```

---

## 📊 消息格式

### 位置控制

```json
{
  "type": "position",
  "position": {
    "x": 10.0,   // North (米)
    "y": 5.0,    // East (米)
    "z": -5.0,   // Up 5米 (负值向上，NED坐标系)
    "yaw": 0.0   // 弧度
  }
}
```

### 速度控制

```json
{
  "type": "velocity",
  "velocity": {
    "vx": 2.0,      // North方向 m/s
    "vy": 1.0,      // East方向 m/s
    "vz": -0.5,     // Up方向 m/s (负值向上)
    "yaw_rate": 0.1 // rad/s
  }
}
```

### 轨迹控制

```json
{
  "type": "trajectory",
  "trajectory": [
    {"t": 0.0, "x": 0, "y": 0, "z": -5, "yaw": 0},
    {"t": 1.0, "x": 5, "y": 0, "z": -5, "yaw": 0},
    {"t": 2.0, "x": 5, "y": 5, "z": -5, "yaw": 0}
  ]
}
```

**注意**：AirSim使用**NED坐标系**（North-East-Down），Z负值表示向上。

---

## ⚙️ 配置说明

### settings.json（AirSim配置）

位置：`~/Documents/AirSim/settings.json`

```json
{
  "SettingsVersion": 1.2,
  "SimMode": "Multirotor",
  "Vehicles": {
    "Drone1": {
      "VehicleType": "SimpleFlight",
      "X": 0, "Y": 0, "Z": 0      // 对应uav1
    },
    "Drone2": {
      "VehicleType": "SimpleFlight",
      "X": 0, "Y": -10, "Z": 0    // 对应uav2
    },
    "Drone3": {
      "VehicleType": "SimpleFlight",
      "X": 0, "Y": 10, "Z": 0     // 对应uav3
    }
  }
}
```

**关键**：Vehicle位置必须与 `config/multi_uav.yaml` 中的 `airsim_spawn_offsets` 一致！

### multi_uav.yaml（系统配置）

```yaml
# 编队初始位置（与settings.json一致）
formation_sync:
  ros__parameters:
    airsim_spawn_offsets:
      uav1: {x: 0.0, y: 0.0, z: 0.0}
      uav2: {x: 0.0, y: -10.0, z: 0.0}
      uav3: {x: 0.0, y: 10.0, z: 0.0}

# 启用AirSim API
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

---

## 🐛 故障排除

### 问题1：连接失败

**原因**：AirSim未运行或IP地址错误

**解决**：
1. 确保AirSim（虚幻引擎）正在运行
2. 检查IP地址（默认127.0.0.1）
3. 运行测试：`python3 airsim_config/test_airsim_connection.py`

### 问题2：无人机不移动

**原因**：API未启用或无人机未解锁

**解决**：
1. 确认 `use_airsim_api: true`
2. 检查日志是否有"成功连接到AirSim"
3. 测试脚本会自动解锁无人机

### 问题3：settings.json不生效

**原因**：配置文件位置错误或AirSim未重启

**解决**：
1. 确认文件在 `~/Documents/AirSim/settings.json`
2. 重启AirSim（虚幻引擎）
3. 检查日志确认加载了配置

### 问题4：坐标不对

**原因**：坐标系混淆

**解决**：
- AirSim使用**NED坐标系**
- Z = -5 表示向上5米（不是Z = 5）
- 确认你的代码使用负Z值

---

## 📁 文件清单

```
uav_sys/
├── AIRSIM_INTEGRATION.md          ← 本文档（唯一的AirSim文档）
├── test_airsim_integration.sh     ← 一键测试脚本
│
├── airsim_config/
│   ├── settings.json              ← AirSim配置（复制到~/Documents/AirSim/）
│   └── test_airsim_connection.py  ← 连接测试脚本
│
├── examples/
│   └── central_planner_airsim.py  ← 中央算力示例
│
├── config/
│   └── multi_uav.yaml             ← 系统配置（启用use_airsim_api）
│
└── src/uav_decision_arbiter/uav_decision_arbiter/
    └── airsim_adapter.py          ← 核心代码（已完成）
```

---

## ✅ 完成清单

对接成功的标志：

- [ ] airsim包已安装 (`pip list | grep airsim`)
- [ ] settings.json已复制到 `~/Documents/AirSim/`
- [ ] AirSim（虚幻引擎）正在运行
- [ ] test_airsim_connection.py 运行成功
- [ ] use_airsim_api: true 已启用
- [ ] 系统启动无错误
- [ ] 日志显示"成功连接到AirSim"
- [ ] 发布决策后无人机在AirSim中移动

---

## 🎯 快速参考

| 需求 | 命令/操作 |
|------|----------|
| 安装 | `pip install airsim` |
| 配置 | 复制 `settings.json` 到 `~/Documents/AirSim/` |
| 启用 | 编辑 `multi_uav.yaml`，设置 `use_airsim_api: true` |
| 测试 | `./test_airsim_integration.sh` |
| 运行 | `ros2 launch ...` + `python3 examples/central_planner_airsim.py` |
| 话题 | `/{uav_id}/central/decision_output` |
| 坐标系 | NED（Z负值向上） |

---

**完整对接时间：5分钟**  
**代码编写量：0行（Adapter已完成）**  
**配置修改量：3处**

🎉 **就这么简单！**

