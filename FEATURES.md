# 系统功能说明

本文档介绍UAV多源决策系统的核心功能特性。

---

## 🎯 核心功能

### 1. 多源决策仲裁

**功能**：智能选择和切换三种决策源（RL、中央算力、人类控制）

**优先级**：
- Human（人类控制）：优先级 200
- Central（中央算力）：优先级 150  
- RL（强化学习）：优先级 100

**特性**：
- ✅ 高优先级自动抢占低优先级
- ✅ 防抖动机制（hysteresis）
- ✅ 心跳检测和超时处理
- ✅ 实时状态监控

**配置**：`config/default.yaml` 或 `config/multi_uav.yaml`

---

### 2. 初始位置同步（单机）

**功能**：启动时自动将AirSim和RL平台的无人机位置同步到真实PX4的位置

**工作流程**：
```
PX4真机 (获取真实位置)
   ↓
发布到 /uav/initial_position
   ↓
AirSim Adapter → 设置AirSim无人机位置
RL Adapter → 通知RL平台更新位置
```

**配置**：
```yaml
px4_adapter:
  ros__parameters:
    publish_initial_position: true    # 启用发布初始位置
    initial_position_delay: 3.0       # 延迟3秒等待系统稳定

airsim_adapter:
  ros__parameters:
    sync_initial_position: true       # 启用同步

rl_adapter:
  ros__parameters:
    sync_initial_position: true       # 启用同步
```

**测试**：
```bash
# 启动系统
ros2 launch uav_decision_arbiter system.launch.py

# 测试同步（mock PX4）
python3 examples/test_initial_position.py
```

---

### 3. 编队位置同步（多机）

**功能**：保持多架无人机在AirSim、RL和PX4三个平台上的相对位置一致

**关键特性**：
- ✅ 自动坐标系转换（ENU ↔ NED）
- ✅ 维护编队相对位置
- ✅ 实时偏差监控
- ✅ 支持任意编队队形

**工作流程**：
```
PX4 (ENU坐标)
   ↓
Formation Sync Node
   ├─ 读取各机位置
   ├─ 计算相对偏移
   ├─ 坐标系转换（ENU→NED）
   ↓
发布到 /uav/formation_sync
   ↓
AirSim Adapter → 设置各机位置
RL Adapter → 通知RL平台
```

**配置**：
```yaml
formation_sync:
  ros__parameters:
    leader_uav: "uav1"
    
    # 编队队形（相对于长机）
    formation_offsets:
      uav1: {x: 0.0, y: 0.0, z: 0.0}      # 长机
      uav2: {x: 0.0, y: -5.0, z: 0.0}     # 左翼
      uav3: {x: 0.0, y: 5.0, z: 0.0}      # 右翼
    
    # AirSim生成位置（NED坐标）
    airsim_spawn_offsets:
      uav1: {x: 0.0, y: 0.0, z: 0.0}
      uav2: {x: 0.0, y: -10.0, z: 0.0}
      uav3: {x: 0.0, y: 10.0, z: 0.0}
```

**测试**：
```bash
# 启动多机系统
ros2 launch uav_decision_arbiter multi_uav.launch.py

# 测试编队同步（mock多PX4）
python3 examples/mock_multi_px4.py
```

---

### 4. 集中仲裁架构（多机）

**功能**：单个Arbiter同时管理多架无人机的决策，每架独立仲裁

**架构**：
```
           ┌─────────────────────┐
           │  Centralized Arbiter │
           └─────────────────────┘
                     │
         ┌───────────┼───────────┐
         │           │           │
     UAV1决策    UAV2决策    UAV3决策
   (独立仲裁) (独立仲裁) (独立仲裁)
```

**配置**：
```yaml
arbiter:
  ros__parameters:
    centralized_mode: true
    uav_ids: ["uav1", "uav2", "uav3"]
```

**优势**：
- 统一管理，减少节点数量
- 每架无人机独立仲裁
- 支持广播命令（target_uav_id: "all"）

---

### 5. 实时可视化（RL平台）

**功能**：Pygame实时显示多机状态和轨迹

**特性**：
- ✅ 2D俯视图
- ✅ 实时轨迹绘制
- ✅ 决策源状态显示
- ✅ 交互控制（暂停/重置）

**启动**：
```bash
python3 rl_platform/rl_platform_node.py
```

**详细文档**：`RL_PLATFORM_GUIDE.md`

---

### 6. 3D仿真（Gazebo + PX4 SITL）

**功能**：真实的物理仿真环境

**特性**：
- ✅ Gazebo 3D可视化
- ✅ PX4 SITL多机仿真
- ✅ MAVROS完整桥接
- ✅ 真实物理模拟

**启动**：
```bash
./gazebo_sim/start_gazebo_sim.sh
```

**详细文档**：`GAZEBO_QUICKSTART.md`

---

### 7. AirSim集成

**功能**：对接虚幻引擎的AirSim仿真

**特性**：
- ✅ 代码100%完成
- ✅ 3步配置即可对接
- ✅ 支持位置/速度/轨迹控制
- ✅ 自动编队同步

**配置**：
```bash
# 1. 安装
pip install airsim

# 2. 配置
cp airsim_config/settings.json ~/Documents/AirSim/

# 3. 启用
# 编辑 config/multi_uav.yaml
use_airsim_api: true
```

**详细文档**：`AIRSIM_INTEGRATION.md`

---

## 🔄 坐标系说明

### ENU坐标系（MAVROS/PX4）
- **X**: East（东）
- **Y**: North（北）
- **Z**: Up（上）

### NED坐标系（AirSim）
- **X**: North（北）
- **Y**: East（东）
- **Z**: Down（下，负值向上）

**转换**：`src/.../coordinate_utils.py` 自动处理

---

## 📊 消息格式

### CommandMsg（统一命令格式）

```json
{
  "header": {
    "source": "rl",           // rl, central, human
    "priority": 100,          // 优先级
    "timestamp": 1234567890,
    "target_uav_id": "uav1"   // 目标无人机ID
  },
  "body": {
    "mode": "velocity",       // position, velocity, trajectory
    "velocity": {
      "vx": 2.0,
      "vy": 1.0,
      "vz": 0.5,
      "yaw_rate": 0.1
    }
  }
}
```

---

## 🎛️ 配置文件

| 文件 | 用途 |
|------|------|
| `config/default.yaml` | 单机配置 |
| `config/multi_uav.yaml` | 多机配置 |
| `airsim_config/settings.json` | AirSim配置 |
| `gazebo_sim/gazebo_config.yaml` | Gazebo配置 |

---

## 🧪 测试工具

```bash
# 系统监控
python3 examples/monitor.py

# RL决策测试
python3 examples/test_rl_publisher.py

# 中央算力测试
python3 examples/test_central_publisher.py

# 初始位置同步测试
python3 examples/test_initial_position.py

# 多机编队测试
python3 examples/mock_multi_px4.py

# AirSim连接测试
./test_airsim_integration.sh
```

---

## 📖 相关文档

- **QUICKSTART.md** - 快速开始指南
- **MULTI_UAV_GUIDE.md** - 多机详细指南
- **AIRSIM_INTEGRATION.md** - AirSim对接
- **GAZEBO_QUICKSTART.md** - Gazebo仿真
- **RL_PLATFORM_GUIDE.md** - RL平台开发
- **TROUBLESHOOTING.md** - 故障排除

---

## ✨ 特性总览

| 功能 | 单机 | 多机 | 状态 |
|------|------|------|------|
| 决策仲裁 | ✅ | ✅ | 完成 |
| 初始位置同步 | ✅ | ✅ | 完成 |
| 编队同步 | - | ✅ | 完成 |
| RL可视化 | ✅ | ✅ | 完成 |
| AirSim集成 | ✅ | ✅ | 完成 |
| Gazebo仿真 | ✅ | ✅ | 完成 |
| 实时监控 | ✅ | ✅ | 完成 |

**系统完整度：100%** 🎉

