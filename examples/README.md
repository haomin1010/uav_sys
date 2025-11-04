# 测试示例

本目录包含用于测试UAV多源决策系统的示例脚本。

## 测试脚本说明

### 1. test_rl_publisher.py
模拟RL平台发布决策命令（速度控制）。

运行方法：
```bash
python3 examples/test_rl_publisher.py
```

功能：
- 以10Hz频率发布速度命令
- 模拟圆形轨迹运动
- 发布到话题 `/rl/decision_output`

### 2. test_central_publisher.py
模拟中央算力发布决策命令（位置控制）。

运行方法：
```bash
python3 examples/test_central_publisher.py
```

功能：
- 每2秒发布一个目标位置
- 循环4个路点（正方形轨迹）
- 发布到话题 `/central/decision_output`

### 3. monitor.py
实时监控系统状态。

运行方法：
```bash
python3 examples/monitor.py
```

功能：
- 显示仲裁器状态（当前生效源、各源状态）
- 显示同步状态（各平台在线情况）
- 显示权威命令内容

### 4. test_initial_position.py
模拟PX4发布初始位置（测试初始位置同步功能）。

运行方法：
```bash
python3 examples/test_initial_position.py
```

功能：
- 模拟PX4发布初始位置到 `/uav/initial_position`
- 测试AirSim和RL平台的位置同步
- 无需真实PX4/MAVROS即可测试

**使用场景**：在没有PX4硬件时测试初始位置同步功能

## 完整测试流程

### 场景1：测试RL单独控制

1. 启动系统：
```bash
ros2 launch uav_decision_arbiter system.launch.py
```

2. 在新终端启动监控：
```bash
python3 examples/monitor.py
```

3. 在新终端启动RL发布器：
```bash
python3 examples/test_rl_publisher.py
```

观察：
- 监控显示当前生效源为 `rl`
- 权威命令显示速度控制指令

### 场景2：测试优先级抢占

1. 在场景1基础上，启动中央算力发布器：
```bash
python3 examples/test_central_publisher.py
```

观察：
- 监控显示当前生效源从 `rl` 切换到 `central`
- 权威命令显示位置控制指令
- RL仍在发布，但被抢占

2. 停止中央算力发布器（Ctrl+C）

观察：
- 监控显示当前生效源回退到 `rl`
- 系统自动回退到可用的低优先级源

### 场景3：测试人类控制最高优先级

人类控制需要MAVROS和PX4连接，如果没有实际硬件，可以：

1. 手动发布人类控制命令：
```bash
ros2 topic pub /mavros/setpoint_velocity/cmd_vel_unstamped geometry_msgs/msg/Twist \
  "{linear: {x: 2.0, y: 1.0, z: 0.5}, angular: {z: 0.2}}"
```

观察：
- 监控显示当前生效源立即切换到 `human`
- 所有其他决策源被抢占

## 调试命令

查看所有话题：
```bash
ros2 topic list
```

查看特定话题内容：
```bash
ros2 topic echo /uav/authoritative_cmd
ros2 topic echo /uav/arbiter/status
```

记录所有数据用于回放：
```bash
ros2 bag record -a
```

查看节点图：
```bash
ros2 run rqt_graph rqt_graph
```

