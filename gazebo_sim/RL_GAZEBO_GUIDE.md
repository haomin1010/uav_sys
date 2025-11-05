# RL平台与Gazebo集成指南

## 概述

本指南介绍如何让RL平台获取Gazebo无人机状态，并控制RL决策的启动/停止。

## 功能特性

### 1. RL适配器状态同步
- ✅ **自动订阅Gazebo状态**：位置、速度、姿态
- ✅ **实时转发给RL平台**：通过 `/{uav_id}/rl/gazebo_state` 话题
- ✅ **启动控制**：等待明确指令后才开始下达RL决策
- ✅ **状态输出**：实时显示RL启用状态

### 2. Gazebo自动起飞
- ✅ **一键起飞**：所有无人机自动解锁并起飞到指定高度
- ✅ **顺序起飞**：避免碰撞，间隔起飞
- ✅ **高度可配置**：默认2米，可自定义

## 快速开始

### 步骤1：启动Gazebo仿真

```bash
cd /home/lihaomin/project/uav_sys
source setup_env.sh
./gazebo_sim/start_gazebo_sim.sh
```

等待所有无人机（3架）启动完成。

### 步骤2：启动多机决策系统

```bash
cd /home/lihaomin/project/uav_sys
source setup_env.sh
ros2 launch uav_decision_arbiter multi_uav.launch.py
```

此时RL适配器已启动，但处于**等待状态**，不会发送决策命令。

### 步骤3：让无人机起飞

```bash
./gazebo_sim/auto_takeoff.sh 3 2.0
```

参数说明：
- `3`：无人机数量
- `2.0`：起飞高度（米）

### 步骤4：监控RL接收的Gazebo状态

在新终端中：

```bash
cd /home/lihaomin/project/uav_sys
source setup_env.sh
./gazebo_sim/monitor_rl_state.sh uav1
```

你会看到类似输出：

```json
{
  "timestamp": 1762312000.123,
  "uav_id": "uav1",
  "pose": {
    "x": 0.01,
    "y": -0.02,
    "z": -2.0,
    "qw": 1.0,
    "qx": 0.0,
    "qy": 0.0,
    "qz": 0.0
  },
  "velocity": {
    "vx": 0.0,
    "vy": 0.0,
    "vz": 0.0,
    "wx": 0.0,
    "wy": 0.0,
    "wz": 0.0
  },
  "state": {
    "connected": true,
    "armed": true,
    "mode": "OFFBOARD"
  },
  "rl_enabled": false
}
```

### 步骤5：启动RL决策

当RL平台准备好后，启动RL决策：

```bash
./gazebo_sim/control_rl.sh start
```

✓ 现在RL平台的决策命令会被转发到仲裁器。

### 步骤6：停止RL决策（可选）

如果需要暂停RL决策：

```bash
./gazebo_sim/control_rl.sh stop
```

切换状态：

```bash
./gazebo_sim/control_rl.sh toggle
```

## 话题说明

### RL平台订阅的话题（输入）

| 话题 | 类型 | 说明 |
|------|------|------|
| `/{uav_id}/rl/gazebo_state` | `std_msgs/String` | Gazebo状态（位置、速度、姿态） |
| `/uav/sync/cmd` | `std_msgs/String` | 仲裁后的权威命令（可视化） |
| `/uav/initial_position` | `std_msgs/String` | 初始位置同步 |

### RL平台发布的话题（输出）

| 话题 | 类型 | 说明 |
|------|------|------|
| `/rl/decision_output` | `std_msgs/String` | RL决策输出 |
| `/rl/state_feedback` | `std_msgs/String` | RL平台状态反馈 |

### RL控制话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/rl/control_command` | `std_msgs/String` | 启动/停止RL决策 |

## RL决策输出格式

### 速度控制模式（默认）

```json
{
  "action": [vx, vy, vz, yaw_rate]
}
```

示例：

```bash
ros2 topic pub /rl/decision_output std_msgs/msg/String \
  "{data: '{\"action\": [0.5, 0.0, 0.0, 0.0]}'}"
```

### 位置控制模式

配置 `rl_input_format: "position"` 后：

```json
{
  "position": [x, y, z, yaw]
}
```

## 配置说明

在 `config/multi_uav.yaml` 中：

```yaml
uav1_rl_adapter:
  ros__parameters:
    uav_id: "uav1"                    # 绑定的无人机ID
    sync_gazebo_state: true           # 是否同步Gazebo状态
    wait_for_start_command: true      # 是否等待启动指令
    rl_input_format: "velocity"       # RL输出格式：velocity/position
    command_timeout: 0.5              # 命令超时时间（秒）
```

## 常用命令

### 查看所有话题

```bash
ros2 topic list | grep rl
```

### 查看RL状态

```bash
ros2 topic echo /uav/rl/state --once
```

### 手动发送RL决策（测试）

```bash
# 速度命令（前进0.5m/s）
ros2 topic pub --once /rl/decision_output std_msgs/msg/String \
  "{data: '{\"action\": [0.5, 0.0, 0.0, 0.0]}'}"

# 停止
ros2 topic pub --once /rl/decision_output std_msgs/msg/String \
  "{data: '{\"action\": [0.0, 0.0, 0.0, 0.0]}'}"
```

### 检查无人机状态

```bash
ros2 topic echo /uav1/state --once
```

### 降落所有无人机

```bash
for i in {1..3}; do
  ros2 service call /uav$i/cmd/land mavros_msgs/srv/CommandTOL "{}"
done
```

## 故障排查

### RL决策不生效

1. 检查RL是否启用：
   ```bash
   ros2 topic echo /{uav_id}/rl/gazebo_state --once | grep rl_enabled
   ```

2. 如果显示 `false`，启动RL：
   ```bash
   ./gazebo_sim/control_rl.sh start
   ```

### 无人机不起飞

1. 检查MAVROS连接：
   ```bash
   ros2 topic echo /uav1/state --once | grep connected
   ```

2. 检查PX4状态：
   ```bash
   cat /tmp/px4_uav1.log | tail -20
   ```

### Gazebo状态话题无数据

1. 检查MAVROS话题：
   ```bash
   ros2 topic hz /uav1/local_position/pose
   ```

2. 检查QoS设置（应为BEST_EFFORT）

## 进阶使用

### 自定义起飞高度

```bash
./gazebo_sim/auto_takeoff.sh 3 5.0  # 起飞到5米
```

### 监控特定无人机

```bash
./gazebo_sim/monitor_rl_state.sh uav2  # 监控UAV2
```

### 分别控制多个RL实例

每个无人机有独立的RL适配器：
- UAV1: `/uav1/rl/gazebo_state`
- UAV2: `/uav2/rl/gazebo_state`
- UAV3: `/uav3/rl/gazebo_state`

可以为每架无人机运行独立的RL策略。

## 完整工作流程示例

```bash
# 1. 启动Gazebo（终端1）
cd /home/lihaomin/project/uav_sys && source setup_env.sh
./gazebo_sim/start_gazebo_sim.sh

# 2. 启动决策系统（终端2）
cd /home/lihaomin/project/uav_sys && source setup_env.sh
ros2 launch uav_decision_arbiter multi_uav.launch.py

# 等待5秒系统稳定

# 3. 让无人机起飞（终端3）
cd /home/lihaomin/project/uav_sys && source setup_env.sh
./gazebo_sim/auto_takeoff.sh

# 4. 监控状态（终端4）
cd /home/lihaomin/project/uav_sys && source setup_env.sh
./gazebo_sim/monitor_rl_state.sh uav1

# 5. 启动RL决策（终端3）
./gazebo_sim/control_rl.sh start

# 6. RL平台现在可以发送决策了
# 在你的RL代码中发布到 /rl/decision_output
```

## 注意事项

1. **RL适配器默认等待启动指令**：修改 `wait_for_start_command: false` 可以自动启用
2. **每个UAV独立配置**：可以为不同UAV设置不同的RL参数
3. **Gazebo状态是本地坐标**：相对于各自的home position
4. **命令超时**：RL决策有0.5秒超时，需要持续发布
5. **优先级**：人类控制(200) > 中央算力(150) > RL(100)

## 相关文档

- [GAZEBO_INTEGRATION.md](./GAZEBO_INTEGRATION.md) - Gazebo多机仿真指南
- [QUICK_TEST.md](./QUICK_TEST.md) - 快速测试指南
- [multi_uav.yaml](../src/uav_decision_arbiter/config/multi_uav.yaml) - 配置文件


