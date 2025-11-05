# Gazebo仿真系统集成说明

## 🎯 核心问题：Gazebo如何与UAV决策系统联动？

### 简单回答

**Gazebo通过MAVROS作为桥梁，与你的系统完全集成**。

```
RL决策 → Arbiter仲裁 → Synchronizer → PX4 Adapter → MAVROS → PX4 SITL → Gazebo
  ↑                                                                        ↓
  └────────────────── 位置反馈 ← MAVROS ← PX4 SITL ← Gazebo ←──────────────┘
```

## 📊 完整数据流

### 1. 启动层级

```
第1层: Gazebo仿真环境
├─ gzserver (物理仿真后端)
└─ gzclient (3D可视化GUI)

第2层: PX4 SITL (飞控仿真)
├─ px4 instance 0 (uav1)
├─ px4 instance 1 (uav2)
└─ px4 instance 2 (uav3)

第3层: MAVROS (ROS2桥接)
├─ /uav1/mavros/* (ROS2话题)
├─ /uav2/mavros/* (ROS2话题)
└─ /uav3/mavros/* (ROS2话题)

第4层: UAV决策系统
├─ PX4 Adapter (订阅MAVROS话题)
├─ Arbiter (仲裁决策)
├─ RL Platform (发布决策)
└─ AirSim Adapter (可选并行)
```

### 2. 控制命令流向

```
┌─────────────────┐
│   RL Platform   │ 你的AI决策
│   (rl_platform) │
└────────┬────────┘
         │ 发布: /uav1/rl/decision_output
         │ 格式: {"x": 10, "y": 5, "z": 3, ...}
         ↓
┌────────────────┐
│  RL Adapter    │ 接收RL决策
│                │
└────────┬───────┘
         │ 发布: /uav/source/rl/cmd
         │ 添加: priority=100
         ↓
┌────────────────┐
│   Arbiter      │ 决策仲裁
│   (仲裁器)      │ 比较优先级: RL(100) vs 中央(150) vs 人类(200)
└────────┬───────┘
         │ 发布: /uav/authoritative_cmd
         │ 选择: 当前最高优先级的命令
         ↓
┌────────────────┐
│ Synchronizer   │ 同步分发
│  (同步器)       │
└────────┬───────┘
         │ 发布: /uav/sync/cmd
         │ 转发给所有适配器
         ↓
┌────────────────┐
│  PX4 Adapter   │ 执行控制 ⭐关键
│                │
└────────┬───────┘
         │ 发布: /uav1/mavros/setpoint_position/local
         │      /uav2/mavros/setpoint_position/local
         │      /uav3/mavros/setpoint_position/local
         │ 格式: PoseStamped消息
         ↓
┌────────────────┐
│     MAVROS     │ MAVLink协议转换
│                │
└────────┬───────┘
         │ UDP通信: 14540, 14541, 14542
         │ 协议: MAVLink
         ↓
┌────────────────┐
│   PX4 SITL     │ 飞控仿真
│  (软件飞控)     │ 执行位置控制算法
└────────┬───────┘
         │ Gazebo插件通信
         │ 发送: 电机指令
         ↓
┌────────────────┐
│    Gazebo      │ 物理仿真
│ (gzserver)     │ 计算: 空气动力学、重力、碰撞
└────────┬───────┘
         │ 模型更新
         ↓
┌────────────────┐
│    Gazebo      │ 3D可视化
│  (gzclient)    │ 显示: 无人机飞行画面
└────────────────┘
```

### 3. 位置反馈流向

```
Gazebo仿真 → PX4 SITL → MAVROS → ROS2话题
                                    ↓
                              /uav1/mavros/local_position/pose
                              /uav2/mavros/local_position/pose
                              /uav3/mavros/local_position/pose
                                    ↓
                              PX4 Adapter订阅
                                    ↓
                              更新系统状态
                                    ↓
                              RL Platform接收(用于决策)
```

## 🔌 关键ROS2话题

### RL Platform发布（决策输出）

```bash
# RL平台为每架无人机发布决策
/uav1/rl/decision_output  # JSON格式
/uav2/rl/decision_output
/uav3/rl/decision_output
```

**消息格式**:
```json
{
  "x": 10.0,      // 目标位置 (米)
  "y": 5.0,
  "z": 3.0,
  "yaw": 1.57,    // 目标航向 (弧度)
  "priority": 100,
  "timestamp": 1234567890.123
}
```

### MAVROS话题（Gazebo交互）

```bash
# 位置反馈（Gazebo → 系统）
/uav1/mavros/local_position/pose  # PoseStamped
/uav2/mavros/local_position/pose
/uav3/mavros/local_position/pose

# 控制命令（系统 → Gazebo）
/uav1/mavros/setpoint_position/local  # PoseStamped
/uav2/mavros/setpoint_position/local
/uav3/mavros/setpoint_position/local

# 状态监控
/uav1/mavros/state  # 连接状态、飞行模式
/uav2/mavros/state
/uav3/mavros/state
```

## 🚀 完整启动流程

### 步骤1: 启动Gazebo仿真

```bash
# 终端1: 启动Gazebo + PX4 SITL + MAVROS
cd ~/project/uav_sys/gazebo_sim
export DISPLAY=:0
./start_gazebo_sim.sh

# 这会启动:
# - Gazebo (gzserver + gzclient)
# - 3个PX4 SITL实例
# - 3个MAVROS节点 (ROS2)
```

**等待30秒，确保：**
- ✅ Gazebo窗口显示3架无人机
- ✅ 终端显示"Gazebo仿真完全启动"

### 步骤2: 验证MAVROS连接

```bash
# 终端2: 验证ROS2话题
ros2 topic list | grep mavros

# 应该看到:
# /uav1/mavros/local_position/pose
# /uav1/mavros/setpoint_position/local
# /uav1/mavros/state
# ... (uav2, uav3 同样)

# 测试位置反馈
ros2 topic echo /uav1/mavros/local_position/pose

# 应该看到实时位置数据:
# pose:
#   position:
#     x: 0.001
#     y: -0.002
#     z: 0.0
```

### 步骤3: 启动UAV决策系统

```bash
# 终端3: 启动仲裁和适配器
cd ~/project/uav_sys
source setup_env.sh
ros2 launch uav_decision_arbiter multi_uav.launch.py

# 这会启动:
# - Arbiter (仲裁器)
# - Synchronizer (同步器)  
# - PX4 Adapter (Gazebo控制器) ⭐
# - AirSim Adapter (可选)
# - Formation Sync (编队同步)
```

**检查启动日志：**
```
[px4_adapter]: 订阅 /uav1/mavros/local_position/pose
[px4_adapter]: 发布到 /uav1/mavros/setpoint_position/local
[arbiter]: 集中仲裁模式已启动
```

### 步骤4: 启动RL决策平台

```bash
# 终端4: 启动RL平台（带Pygame可视化）
cd ~/project/uav_sys
source setup_env.sh
python3 rl_platform/rl_platform_node.py

# 应该看到:
# - Pygame窗口打开
# - 显示3架无人机的2D轨迹
# - RL策略开始发布决策
```

### 步骤5: 观察联动效果！

现在你应该同时看到：

1. **Gazebo 3D窗口** - 无人机在3D场景中真实飞行
2. **Pygame 2D窗口** - RL平台的决策可视化
3. **终端日志** - 实时的决策和控制信息

**无人机会：**
- RL平台生成导航决策 → Arbiter仲裁 → Gazebo执行
- Gazebo的位置反馈 → RL平台更新 → 继续决策
- **完整闭环控制！**

## 🔍 调试和验证

### 验证1: 检查RL决策是否发布

```bash
# 查看RL决策输出
ros2 topic echo /uav1/rl/decision_output

# 应该看到JSON格式的目标位置
```

### 验证2: 检查权威命令

```bash
# 查看仲裁后的命令
ros2 topic echo /uav/authoritative_cmd

# 应该看到:
# source: "rl"
# priority: 100
# target_uav_id: "uav1"
# command: {"x": ..., "y": ..., ...}
```

### 验证3: 检查MAVROS控制命令

```bash
# 查看发送给Gazebo的命令
ros2 topic echo /uav1/mavros/setpoint_position/local

# 应该看到geometry_msgs/PoseStamped消息
```

### 验证4: 监控Gazebo执行

在Gazebo窗口中，你应该看到无人机根据RL决策移动！

## 🎮 手动控制测试

### 方法1: 直接发送RL决策

```bash
# 发送一个导航命令
ros2 topic pub /uav1/rl/decision_output std_msgs/String \
  'data: "{\"x\": 5.0, \"y\": 3.0, \"z\": 2.0, \"yaw\": 0.0, \"priority\": 100}"'

# Gazebo中的uav1应该飞向 (5, 3, 2)
```

### 方法2: 直接控制MAVROS（绕过系统）

```bash
# 直接发送位置命令到Gazebo
ros2 topic pub /uav1/mavros/setpoint_position/local \
  geometry_msgs/PoseStamped \
  "{header: {frame_id: 'map'}, 
    pose: {position: {x: 10.0, y: 0.0, z: 3.0}}}"

# 无人机立即响应
```

## 📈 性能指标

### 正常运行时的指标

```bash
# 控制频率
ros2 topic hz /uav1/mavros/setpoint_position/local
# 应该: ~20 Hz

# 位置反馈频率  
ros2 topic hz /uav1/mavros/local_position/pose
# 应该: ~30 Hz

# RL决策频率
ros2 topic hz /uav1/rl/decision_output
# 应该: ~10 Hz
```

### 延迟检查

```bash
# 端到端延迟（RL决策 → Gazebo响应）
# 正常: < 100ms
# 可接受: < 200ms
```

## 🔧 常见问题

### 问题1: RL决策发布了，但Gazebo中无人机不动

**检查：**
```bash
# 1. MAVROS是否连接
ros2 topic echo /uav1/mavros/state
# connected: True

# 2. PX4 Adapter是否订阅
ros2 node info /px4_adapter
# Subscriptions: /uav/sync/cmd

# 3. 是否有权威命令
ros2 topic echo /uav/authoritative_cmd
```

**原因：**
- PX4需要先解锁（arm）和切换到OFFBOARD模式
- 启动脚本应该已经处理，但可以手动：

```bash
# 解锁
ros2 service call /uav1/mavros/cmd/arming \
  mavros_msgs/CommandBool "{value: true}"

# 切换到OFFBOARD模式
ros2 service call /uav1/mavros/set_mode \
  mavros_msgs/SetMode "{custom_mode: 'OFFBOARD'}"
```

### 问题2: Gazebo显示但位置反馈为0

**原因**: PX4 SITL与Gazebo插件未正确连接

**检查**:
```bash
cat /tmp/px4_uav1.log | grep "Connected"
# 应该看到: INFO [mavlink] Connected to PX4 SITL
```

### 问题3: 多架无人机只有一架在动

**原因**: target_uav_id匹配问题或MAVROS端口配置错误

**检查**:
```bash
# 查看所有MAVROS实例
ps aux | grep mavros_node

# 应该有3个进程，端口: 14540, 14541, 14542
```

## 🎯 最佳实践

### 1. 启动顺序很重要

```
Gazebo → MAVROS → UAV系统 → RL平台
```

每个组件等待前一个完全启动（30秒）

### 2. 调试时分离组件

```bash
# 只启动Gazebo，不启动RL
./start_gazebo_sim.sh

# 手动测试MAVROS控制
ros2 topic pub ...
```

### 3. 使用可视化工具

```bash
# rqt查看话题关系
rqt_graph

# 实时监控
ros2 topic list
ros2 topic echo <topic>
```

## 📚 相关文档

- [GAZEBO_QUICKSTART.md](./GAZEBO_QUICKSTART.md) - Gazebo快速启动
- [RL_PLATFORM_GUIDE.md](./RL_PLATFORM_GUIDE.md) - RL平台详解
- [MULTI_UAV_GUIDE.md](./MULTI_UAV_GUIDE.md) - 多机系统
- [README.md](./README.md) - 系统总览

## 🎉 总结

**Gazebo如何联动？**

1. **物理层**: Gazebo提供真实的物理仿真和3D可视化
2. **接口层**: MAVROS提供ROS2话题桥接
3. **控制层**: PX4 Adapter订阅MAVROS，接收位置，发送命令
4. **决策层**: RL Platform生成决策 → Arbiter仲裁 → 发送到Gazebo
5. **反馈层**: Gazebo位置 → MAVROS → RL Platform → 更新决策

**完整闭环**：
```
RL决策 → Gazebo执行 → 位置反馈 → RL更新 → 新决策 → ...
```

现在你的系统有了**真实的物理仿真环境**，可以测试复杂的飞行任务！🚁✨



