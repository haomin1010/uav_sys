# 集中仲裁架构升级文档

## 📝 概述

系统已从**单机架构**升级为**集中仲裁架构**，一个中心仲裁器可以同时管理多架无人机。

## 🏗️ 架构变化

### 旧架构（单机）
```
决策源 → Arbiter → 一架无人机
```

### 新架构（集中仲裁）
```
         全局决策源
   RL编队算法 ──┐
   中央规划 ────┼──→ 集中Arbiter ──┬──→ UAV1
   人类指挥 ────┘    (多机管理)    ├──→ UAV2
                                  └──→ UAV3
```

## ✨ 核心改动

### 1. 消息格式升级

**新增字段**: `target_uav_id`

```python
# 广播命令（所有无人机）
header = Header(
    timestamp=...,
    source_id="rl",
    seq=123,
    target_uav_id="all"  # 所有无人机执行
)

# 定向命令（指定无人机）
header = Header(
    timestamp=...,
    source_id="central",
    seq=456,
    target_uav_id="uav2"  # 只有uav2执行
)
```

### 2. Arbiter集中管理

**多机状态管理**:
```python
uav_states = {
    "uav1": {
        "sources": {rl, central, human},
        "current_source": "rl",
        "last_switch_time": ...
    },
    "uav2": {
        "sources": {rl, central, human},
        "current_source": "central",
        "last_switch_time": ...
    },
    ...
}
```

**独立仲裁**：每架无人机独立评估决策源，互不干扰。

### 3. 配置文件

```yaml
# config/default.yaml
arbiter:
  ros__parameters:
    centralized_mode: true             # 启用集中仲裁
    uav_ids: ["uav1", "uav2", "uav3"] # 管理的无人机列表
    hysteresis_ms: 200
    heartbeat_timeout: 2.0
```

## 🚀 使用方法

### 快速开始（3架无人机）

```bash
# 1. 修改配置
编辑 config/default.yaml:
arbiter:
  ros__parameters:
    centralized_mode: true
    uav_ids: ["uav1", "uav2", "uav3"]

# 2. 启动系统
source setup_env.sh
ros2 launch uav_decision_arbiter system.launch.py

# 3. 系统自动管理3架无人机
```

### 决策源如何发布命令

#### 广播命令（所有无人机执行相同动作）

```python
import json
from std_msgs.msg import String

# RL编队飞行：所有无人机保持队形
decision = {
    "action": [1.0, 0.5, 0.2, 0.0],  # 统一速度
    "timestamp": time.time()
}

# target_uav_id默认为"all"，所有无人机都会执行
msg = String()
msg.data = json.dumps(decision)
pub_rl.publish(msg)
```

#### 定向命令（指定无人机）

**方式1：在决策数据中指定** (需要修改你的RL/Central代码)

```python
# 中央算力：给uav2下达特殊任务
decision = {
    "type": "position",
    "target_uav_id": "uav2",  # 关键字段
    "position": {"x": 10.0, "y": 5.0, "z": 3.0, "yaw": 0.0}
}
```

**方式2：使用不同话题** (推荐，无需修改现有代码)

```python
# 为每架无人机发布到不同话题
pub_uav1 = create_publisher('/uav1/central/decision_output')
pub_uav2 = create_publisher('/uav2/central/decision_output')
pub_uav3 = create_publisher('/uav3/central/decision_output')

# 各自发布不同命令
pub_uav1.publish(decision_for_uav1)
pub_uav2.publish(decision_for_uav2)
pub_uav3.publish(decision_for_uav3)
```

## 📊 监控多机状态

### 查看仲裁器状态

```bash
ros2 topic echo /uav/arbiter/status
```

**输出示例**:
```json
{
  "timestamp": 1762247595.12,
  "mode": "centralized",
  "uav_count": 3,
  "uavs": {
    "uav1": {
      "current_source": "rl",
      "sources": {
        "human": {"priority": 200, "has_cmd": false},
        "central": {"priority": 150, "has_cmd": false},
        "rl": {"priority": 100, "has_cmd": true, "cmd_valid": true}
      }
    },
    "uav2": {
      "current_source": "central",
      "sources": {...}
    },
    "uav3": {
      "current_source": "human",
      "sources": {...}
    }
  }
}
```

### 权威命令包含无人机ID

```bash
ros2 topic echo /uav/authoritative_cmd
```

**输出**:
```json
{
  "header": {
    "target_uav_id": "uav2",  # 指定无人机
    "source_id": "central",
    "seq": 123
  },
  "body": {...}
}
```

## 🎮 典型应用场景

### 场景1：编队飞行（广播）

```python
# RL算法：控制整个编队保持队形
class FormationRL:
    def decision(self):
        # 所有无人机执行相同的相对速度
        return {
            "action": [vx, vy, vz, yaw_rate],
            # 不指定target_uav_id = 广播到所有无人机
        }
```

### 场景2：分工协作（定向）

```python
# 中央算力：为每架无人机分配不同任务
decisions = {
    "uav1": {"position": {"x": 0, "y": 0, "z": 5}},   # 侦查
    "uav2": {"position": {"x": 10, "y": 0, "z": 5}},  # 跟踪
    "uav3": {"position": {"x": 5, "y": 10, "z": 5}}   # 中继
}

for uav_id, decision in decisions.items():
    decision["target_uav_id"] = uav_id
    publish_decision(decision)
```

### 场景3：紧急接管（人类控制特定无人机）

```python
# 地面站：接管uav2进行手动控制
# uav1和uav3继续自主飞行
manual_control_uav2 = {
    "velocity": {"vx": 2.0, ...},
    "target_uav_id": "uav2"
}
```

## 🔄 向后兼容

### 单机模式仍然支持

```yaml
arbiter:
  ros__parameters:
    centralized_mode: false  # 单机模式
    uav_ids: ["uav1"]        # 只管理一架
```

### 旧版消息自动升级

没有`target_uav_id`的消息自动设置为`"all"`，确保兼容性。

## ⚙️ 配置选项

### Arbiter配置

```yaml
arbiter:
  ros__parameters:
    centralized_mode: true           # true=集中仲裁, false=单机
    uav_ids: ["uav1", "uav2", "uav3"]  # 管理的无人机列表
    hysteresis_ms: 200                 # 防抖窗口
    heartbeat_timeout: 2.0             # 心跳超时
```

### 扩展到更多无人机

```yaml
# 10架无人机编队
arbiter:
  ros__parameters:
    uav_ids: ["uav1", "uav2", "uav3", "uav4", "uav5", 
              "uav6", "uav7", "uav8", "uav9", "uav10"]
```

## 🎯 决策逻辑

### 独立仲裁

每架无人机**独立**评估决策源：

```
时刻T:
  uav1: RL控制 (优先级100)
  uav2: Central控制 (优先级150，抢占了RL)
  uav3: Human控制 (优先级200，紧急接管)
```

### 优先级规则不变

- Human (200) > Central (150) > RL (100)
- 高优先级自动抢占低优先级
- 每架无人机独立应用此规则

## 📈 性能特性

- **并发处理**：同时管理多架无人机
- **独立仲裁**：互不干扰
- **实时性**：20Hz评估频率
- **可扩展**：轻松扩展到10+架无人机

## 🔧 开发建议

### 为多机定制RL算法

```python
class MultiUAVRL:
    def __init__(self, uav_count=3):
        self.uav_count = uav_count
    
    def get_decisions(self):
        """返回每架无人机的决策"""
        decisions = {}
        for i in range(self.uav_count):
            uav_id = f"uav{i+1}"
            decisions[uav_id] = self.compute_action(uav_id)
        return decisions
    
    def publish_decisions(self, decisions):
        for uav_id, action in decisions.items():
            msg = {
                "action": action,
                "target_uav_id": uav_id  # 定向发布
            }
            self.publisher.publish(json.dumps(msg))
```

### 中央算力任务分配

```python
class CentralPlanner:
    def assign_tasks(self, uavs):
        """为每架无人机分配任务"""
        tasks = self.global_planner(uavs)
        
        for uav_id, task in tasks.items():
            self.publish_command(
                uav_id=uav_id,
                position=task.target_position
            )
```

## 🐛 故障排除

### 问题1：某架无人机不响应

**检查**:
```bash
ros2 topic echo /uav/arbiter/status
# 查看该无人机的current_source和命令有效性
```

**可能原因**：
- target_uav_id拼写错误
- 命令已过期
- 该无人机未在uav_ids列表中

### 问题2：所有无人机执行相同命令（不想要）

**原因**：target_uav_id="all"或未指定

**解决**：明确指定target_uav_id

### 问题3：想切换回单机模式

```yaml
arbiter:
  ros__parameters:
    centralized_mode: false
    uav_ids: ["uav1"]
```

## 📚 相关文档

- `README.md` - 系统总体介绍
- `ARCHITECTURE.md` - 架构设计详解
- `QUICKSTART.md` - 快速开始指南

## ✅ 升级清单

- [x] 消息格式添加target_uav_id
- [x] Arbiter支持多机管理
- [x] 配置文件支持uav_ids
- [x] 向后兼容单机模式
- [ ] Synchronizer定向转发（待实现）
- [ ] Adapters绑定无人机ID（待实现）
- [ ] 更新监控工具（待实现）

---

**升级完成度**: Arbiter核心逻辑已完成  
**下一步**: 运行测试验证多机仲裁功能

