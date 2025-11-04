# 快速开始指南

## 环境激活

每次使用前，执行：

```bash
cd /home/lihaomin/project/uav_sys
source setup_env.sh
```

或手动执行：

```bash
conda activate uav_sys
source /opt/ros/humble/setup.zsh
source install/setup.zsh
```

## 快速测试

### 1. 启动完整系统

在终端1中启动所有核心节点：

```bash
ros2 launch uav_decision_arbiter system.launch.py
```

### 2. 启动监控

在终端2中启动监控器：

```bash
python3 examples/monitor.py
```

### 3. 测试RL控制

在终端3中启动RL测试发布器：

```bash
python3 examples/test_rl_publisher.py
```

观察监控器输出，应该看到：
- 当前生效源：`rl`
- 权威命令显示速度控制

### 4. 测试优先级抢占

在终端4中启动中央算力测试发布器：

```bash
python3 examples/test_central_publisher.py
```

观察监控器，应该看到：
- 当前生效源从 `rl` 切换到 `central`
- 权威命令变为位置控制

### 5. 测试回退

按 `Ctrl+C` 停止中央算力发布器，观察监控器：
- 系统自动回退到 `rl` 源

## 常用命令

### 查看系统状态

```bash
# 查看所有节点
ros2 node list

# 查看所有话题
ros2 topic list

# 查看仲裁器状态
ros2 topic echo /uav/arbiter/status

# 查看权威命令
ros2 topic echo /uav/authoritative_cmd
```

### 手动发布测试命令

```bash
# 发布RL决策（速度控制）
ros2 topic pub /rl/decision_output std_msgs/msg/String \
  'data: "{\"action\": [1.0, 0.5, 0.2, 0.1]}"'

# 发布中央决策（位置控制）
ros2 topic pub /central/decision_output std_msgs/msg/String \
  'data: "{\"type\": \"position\", \"position\": {\"x\": 5.0, \"y\": 3.0, \"z\": 2.0, \"yaw\": 0.0}}"'
```

## 项目结构

```
uav_sys/
├── src/
│   └── uav_decision_arbiter/          # ROS2包
│       ├── uav_decision_arbiter/       # Python模块
│       │   ├── command_msg.py         # 统一消息格式
│       │   ├── arbiter_node.py        # 仲裁器
│       │   ├── synchronizer_node.py   # 同步器
│       │   ├── rl_adapter.py          # RL适配器
│       │   ├── airsim_adapter.py      # AirSim适配器
│       │   └── px4_adapter.py         # PX4适配器
│       ├── config/                    # 配置文件
│       ├── launch/                    # 启动文件
│       ├── package.xml
│       └── setup.py
├── examples/                          # 测试示例
│   ├── test_rl_publisher.py
│   ├── test_central_publisher.py
│   ├── monitor.py
│   └── README.md
├── README.md                          # 完整文档
├── QUICKSTART.md                      # 本文件
├── setup_env.sh                       # 环境设置脚本
└── requirements.txt                   # Python依赖
```

## 配置修改

配置文件位置：`src/uav_decision_arbiter/config/default.yaml`

修改后需要重新编译：

```bash
colcon build --symlink-install
source install/setup.zsh
```

## 连接实际平台

### 连接AirSim

1. 确保AirSim正在运行
2. 修改配置文件 `config/default.yaml`：
   ```yaml
   airsim_adapter:
     use_airsim_api: true
     airsim_ip: "127.0.0.1"  # AirSim地址
   ```
3. 安装AirSim Python包：
   ```bash
   pip install airsim
   ```

### 连接PX4

1. 启动PX4 SITL或连接真实飞控
2. 启动MAVROS：
   ```bash
   ros2 run mavros mavros_node --ros-args \
     -p fcu_url:="udp://:14540@127.0.0.1:14557"
   ```
3. 确保配置文件中 `px4_adapter.use_mavros: true`

### 连接RL平台

您的RL平台需要：
1. 发布决策到话题 `/rl/decision_output`
2. 订阅可视化命令从 `/rl/visualization_cmd`
3. （可选）发布状态到 `/rl/state_feedback`

消息格式参考：`examples/test_rl_publisher.py`

## 故障排除

### 编译失败

```bash
# 清理后重新编译
rm -rf build/ install/ log/
colcon build --symlink-install
```

### ROS2环境未找到

```bash
# 确认ROS2安装
ls /opt/ros/

# 如果是其他版本（如foxy），修改setup_env.sh中的ROS_DISTRO
```

### Conda环境问题

```bash
# 重新创建环境
conda deactivate
conda remove -n uav_sys --all
conda create -n uav_sys python=3.10 -y
conda activate uav_sys
pip install -r requirements.txt
```

## 下一步

- 阅读完整文档：`README.md`
- 查看测试示例：`examples/README.md`
- 根据需求修改配置：`src/uav_decision_arbiter/config/default.yaml`
- 集成您的RL平台、AirSim和PX4

## 技术支持

如有问题，请检查：
1. ROS2话题是否正常：`ros2 topic list`
2. 节点是否运行：`ros2 node list`
3. 日志输出中的错误信息

