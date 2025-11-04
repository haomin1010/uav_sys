# UAV多源决策仲裁与同步系统

一个用于无人机控制的多源决策融合平台，支持RL决策、中央算力（AirSim）和人类控制三种决策源的智能仲裁与轨迹同步。

## 系统架构

### 工作模式

系统支持两种工作模式：

1. **单机模式**（默认）- 控制一架无人机
2. **集中仲裁模式** - 同时管理多架无人机编队

### 核心模块

1. **Arbiter（仲裁器）**
   - 接收三路决策源命令
   - 根据优先级和时效性选择当前生效命令
   - 支持高优先级抢占低优先级
   - 防抖动和心跳检测
   - **集中仲裁模式**：同时管理多架无人机的独立决策

2. **Synchronizer（同步器）**
   - 转发仲裁后的权威命令
   - 确保三个平台轨迹同步
   - 监控同步状态

3. **Adapters（适配器）**
   - **RL Adapter**: 对接RL决策平台（带Pygame可视化）
   - **AirSim Adapter**: 对接中央算力（通过AirSim仿真）
   - **PX4 Adapter**: 对接真实飞控、PX4 SITL或Gazebo仿真

4. **Formation Sync（编队同步器）** ⭐新功能
   - 自动坐标系转换（ENU ↔ NED）
   - 维护多机相对位置一致性
   - 启动时同步编队队形
   - 实时监控编队偏差

5. **RL Decision Platform（RL决策平台）** ⭐⭐完整实现
   - Pygame实时可视化界面
   - 多机环境模拟
   - 可运行的RL策略（2种）
   - ROS2完整集成（发送/接收）
   - 支持替换为神经网络策略

6. **Gazebo Simulation（Gazebo仿真）** ⭐新增
   - PX4 SITL多机仿真
   - Gazebo 3D可视化
   - 真实物理模拟
   - MAVROS完整桥接

### 优先级设置

- 人类控制：200（最高）
- 中央算力：150
- RL决策：100（最低）

高优先级决策源会自动抢占低优先级源的控制权。

### 通信架构

```
RL平台 ──→ RL Adapter ──┐
                        │
中央算力 → AirSim Adapter ─┼──→ Arbiter ──→ Synchronizer ──┬──→ RL Adapter ──→ RL可视化
                        │                                │
人类控制 → PX4 Adapter ──┘                                ├──→ AirSim Adapter ──→ AirSim
                                                         │
                                                         └──→ PX4 Adapter ──→ PX4飞控
```

## 安装与配置

### 前置要求

- Ubuntu 20.04/22.04
- ROS 2 (Humble/Foxy)
- Python 3.8+
- Conda

### 环境设置

1. **创建并激活Conda环境**：
```bash
cd /home/lihaomin/project/uav_sys
conda activate uav_sys
```

2. **安装Python依赖**：
```bash
pip install -r requirements.txt
```

3. **配置ROS2环境**：
```bash
source /opt/ros/humble/setup.zsh  # 或您的ROS2版本
```

4. **编译ROS2工作空间**：
```bash
colcon build --symlink-install
source install/setup.zsh
```

### 可选依赖

- **AirSim**: 如需实际连接AirSim仿真
  ```bash
  pip install airsim
  ```

- **MAVROS**: 如需连接PX4飞控
  ```bash
  sudo apt install ros-humble-mavros ros-humble-mavros-extras
  ```

## 使用方法

### 单机模式（一架无人机）

```bash
# 确保已激活conda环境和source ROS2
conda activate uav_sys
source /opt/ros/humble/setup.zsh
source install/setup.zsh

# 启动单机系统
ros2 launch uav_decision_arbiter system.launch.py
```

### 多机模式（编队飞行）⭐

```bash
# 启动多机编队系统（3架无人机）
source setup_env.sh
ros2 launch uav_decision_arbiter multi_uav.launch.py
```

**详细文档**：`MULTI_UAV_GUIDE.md`

### 2. 单独启动各节点

```bash
# 仲裁器
ros2 run uav_decision_arbiter arbiter

# 同步器
ros2 run uav_decision_arbiter synchronizer

# RL适配器
ros2 run uav_decision_arbiter rl_adapter

# AirSim适配器
ros2 run uav_decision_arbiter airsim_adapter

# PX4适配器
ros2 run uav_decision_arbiter px4_adapter
```

### 3. 监控系统状态

```bash
# 查看仲裁器状态
ros2 topic echo /uav/arbiter/status

# 查看同步状态
ros2 topic echo /uav/sync/status

# 查看当前权威命令
ros2 topic echo /uav/authoritative_cmd
```

## 话题接口

### 决策源输入话题

- `/rl/decision_output` - RL平台输出决策
- `/central/decision_output` - 中央算力输出决策
- 人类控制通过MAVROS自动检测

### 内部话题

- `/uav/source/rl/cmd` - RL命令（统一格式）
- `/uav/source/central/cmd` - 中央算力命令（统一格式）
- `/uav/source/human/cmd` - 人类控制命令（统一格式）
- `/uav/authoritative_cmd` - 仲裁后的权威命令
- `/uav/sync/cmd` - 同步命令

### 状态反馈话题

- `/uav/rl/state` - RL平台状态
- `/uav/airsim/state` - AirSim平台状态
- `/uav/px4/state` - PX4平台状态

## 配置说明

配置文件位置：`src/uav_decision_arbiter/config/default.yaml`

主要配置项：
- `arbiter.hysteresis_ms`: 防抖窗口（毫秒）
- `arbiter.heartbeat_timeout`: 心跳超时（秒）
- `airsim_adapter.use_airsim_api`: 是否实际连接AirSim
- `px4_adapter.use_mavros`: 是否使用MAVROS连接PX4

## 消息格式

### CommandMsg 统一命令消息

```json
{
  "header": {
    "timestamp": 1699123456.789,
    "source_id": "rl|central|human",
    "seq": 123
  },
  "meta": {
    "priority": 100,
    "mode": "velocity|position|trajectory",
    "expires_at": 1699123457.789
  },
  "body": {
    "velocity": {"vx": 1.0, "vy": 0.0, "vz": 0.5, "yaw_rate": 0.1},
    // 或
    "setpoint": {"x": 10.0, "y": 5.0, "z": 3.0, "yaw": 0.0},
    // 或
    "trajectory": [
      {"t_offset": 0.0, "x": 0.0, "y": 0.0, "z": 1.0, "yaw": 0.0},
      {"t_offset": 1.0, "x": 1.0, "y": 0.0, "z": 1.0, "yaw": 0.0}
    ]
  }
}
```

## 开发指南

### 添加自定义决策源

1. 在 `uav_decision_arbiter` 包中创建新的适配器文件
2. 继承基础的ROS2 Node类
3. 订阅您的决策源输出话题
4. 将输出转换为统一的 CommandMsg 格式
5. 发布到对应的 `/uav/source/<source_id>/cmd` 话题

### 调试技巧

```bash
# 查看所有活动话题
ros2 topic list

# 查看话题消息类型
ros2 topic info /uav/authoritative_cmd

# 手动发布测试命令
ros2 topic pub /rl/decision_output std_msgs/msg/String \
  'data: "{\"action\": [1.0, 0.0, 0.5, 0.0]}"'

# 记录所有话题用于回放
ros2 bag record -a
```

## 测试场景

### 场景1：单一RL决策
1. 启动系统
2. 发布RL决策到 `/rl/decision_output`
3. 观察仲裁器选择RL源
4. 三个平台同步执行RL命令

### 场景2：中央算力抢占
1. RL正在控制
2. 发布中央算力决策到 `/central/decision_output`
3. 仲裁器切换到中央算力源（优先级更高）
4. 所有平台执行中央算力命令

### 场景3：人类紧急控制
1. 任意决策源在控制
2. 通过遥控器操作（或发布到人类控制话题）
3. 仲裁器立即切换到人类控制（最高优先级）
4. 所有平台响应人类命令

## 故障排除

### 问题1：节点无法启动
- 检查conda环境是否激活
- 检查ROS2环境是否source
- 检查工作空间是否编译：`colcon build`

### 问题2：AirSim连接失败
- 确认AirSim正在运行
- 检查IP地址配置是否正确
- 如不需要实际连接，设置 `use_airsim_api: false`

### 问题3：MAVROS无法连接PX4
- 确认PX4 SITL或真机正在运行
- 检查MAVROS是否正确启动：`ros2 node list | grep mavros`
- 检查连接参数

### 问题4：没有决策生效
- 检查各决策源是否发布消息：`ros2 topic echo /rl/decision_output`
- 查看仲裁器状态：`ros2 topic echo /uav/arbiter/status`
- 检查命令是否过期（expires_at时间戳）

## 高级功能

### 1. 初始位置同步（单机）

系统支持启动时自动同步初始位置，让AirSim仿真和RL平台与真实PX4保持一致。

**快速开始**：
```bash
# 默认已启用，无需额外配置
ros2 launch uav_decision_arbiter system.launch.py
```

**详细文档**：`INITIAL_POSITION_SYNC.md`

**主要特性**：
- ✅ 启动时自动从PX4读取位置
- ✅ 自动设置AirSim仿真无人机位置
- ✅ 通知RL平台初始位置信息
- ✅ 支持手动触发位置同步
- ✅ 可配置启用/禁用

### 2. 编队相对位置同步（多机）⭐新功能

支持多架无人机的编队飞行，自动保持AirSim、RL和PX4三个平台的相对位置一致。

**快速开始**：
```bash
# 启动多机编队系统
ros2 launch uav_decision_arbiter multi_uav.launch.py

# 模拟PX4位置（测试用）
python3 examples/mock_multi_px4.py
```

**详细文档**：`FORMATION_SYNC.md` 和 `MULTI_UAV_GUIDE.md`

**主要特性**：
- ✅ 集中仲裁架构（一个Arbiter管理多机）
- ✅ 自动坐标系转换（ENU ↔ NED）
- ✅ 配置化编队队形（V字、纵队、横向等）
- ✅ 启动时自动同步编队
- ✅ 实时编队偏差监控
- ✅ 支持2-10+架无人机

### 3. 集中仲裁架构

支持一个中心仲裁器同时管理多架无人机的决策。

**详细文档**：`CENTRALIZED_ARCHITECTURE.md`

**主要特性**：
- ✅ 统一决策协调
- ✅ 支持广播和定向命令
- ✅ 每架无人机独立仲裁
- ✅ 灵活的任务分配

### 4. Gazebo 3D仿真⭐新增

使用Gazebo + PX4 SITL提供真实的物理仿真和3D可视化。

**快速开始**：
```bash
cd gazebo_sim
./start_gazebo_sim.sh
```

**详细文档**：`GAZEBO_QUICKSTART.md` 和 `gazebo_sim/README.md`

**主要特性**：
- ✅ Gazebo 3D可视化（真实渲染）
- ✅ PX4 SITL多机仿真（3架）
- ✅ 真实物理模拟（重力、空气动力学）
- ✅ MAVROS完整桥接
- ✅ 与系统无缝对接
- ✅ 一键启动脚本

### 5. RL决策平台⭐⭐完整实现

完整的RL开发平台，包含可视化界面、环境模拟和ROS2通信。

**快速开始**：
```bash
python3 rl_platform/rl_platform_node.py
```

**详细文档**：`RL_PLATFORM_GUIDE.md` 和 `rl_platform/README.md`

**主要特性**：
- ✅ Pygame实时可视化（2D俯视图）
- ✅ 多机环境模拟（位置/速度/奖励）
- ✅ 2种决策策略（简单导航/圆形编队）
- ✅ ROS2完整集成（发送决策/接收同步）
- ✅ 交互控制（暂停/重置/退出）
- ✅ 易于替换为神经网络策略

## 许可证

MIT License

## 作者

UAV Team

## 参考资料

- [ROS2 Documentation](https://docs.ros.org/en/humble/)
- [AirSim Documentation](https://microsoft.github.io/AirSim/)
- [PX4 Documentation](https://docs.px4.io/)
- [MAVROS Documentation](http://wiki.ros.org/mavros)

