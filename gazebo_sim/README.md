# Gazebo + PX4 SITL 多机仿真平台

## 📝 概述

这是一个完整的Gazebo多机仿真方案，提供**3D可视化**和**真实的PX4物理仿真**，与系统完美对接。

## ✨ 功能特性

- ✅ **Gazebo 3D可视化** - 真实的物理仿真环境
- ✅ **多架PX4 SITL** - 同时运行3个完整的PX4实例
- ✅ **MAVROS连接** - 每架独立的ROS2接口
- ✅ **编队spawn** - 按配置队形生成
- ✅ **与系统对接** - 无缝接入PX4 Adapter

## 🏗️ 架构

```
┌─────────────────────────────────────────────────────┐
│              Gazebo仿真环境                          │
├─────────────────────────────────────────────────────┤
│                                                       │
│  ┌──────┐      ┌──────┐      ┌──────┐              │
│  │ Iris1│      │ Iris2│      │ Iris3│              │
│  │ (3D) │      │ (3D) │      │ (3D) │              │
│  └───┬──┘      └───┬──┘      └───┬──┘              │
│      │             │             │                   │
└──────┼─────────────┼─────────────┼───────────────────┘
       │             │             │
       ▼             ▼             ▼
┌──────────────────────────────────────────────────────┐
│          PX4 SITL实例 (软件在环)                      │
├──────────────────────────────────────────────────────┤
│  ┌─────────┐  ┌─────────┐  ┌─────────┐             │
│  │ PX4_1   │  │ PX4_2   │  │ PX4_3   │             │
│  │ ID=1    │  │ ID=2    │  │ ID=3    │             │
│  │:14540   │  │:14541   │  │:14542   │             │
│  └────┬────┘  └────┬────┘  └────┬────┘             │
└───────┼────────────┼─────────────┼───────────────────┘
        │            │             │
        ▼            ▼             ▼
┌──────────────────────────────────────────────────────┐
│              MAVROS (ROS2桥接)                       │
├──────────────────────────────────────────────────────┤
│  /uav1/mavros  /uav2/mavros  /uav3/mavros           │
└───────┬────────────┬─────────────┬───────────────────┘
        │            │             │
        ▼            ▼             ▼
┌──────────────────────────────────────────────────────┐
│         你的UAV决策系统                               │
│  (Arbiter + Adapters + RL Platform)                  │
└──────────────────────────────────────────────────────┘
```

## 📋 前置要求

### 1. 安装PX4-Autopilot

```bash
# 克隆PX4源码
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot

# 安装依赖
bash ./Tools/setup/ubuntu.sh

# 编译（首次会下载Gazebo模型）
make px4_sitl_default gazebo

# 测试单机SITL
make px4_sitl gazebo
# 应该看到Gazebo窗口和一架iris无人机
# Ctrl+C停止
```

### 2. 设置环境变量

```bash
# 添加到 ~/.zshrc 或 ~/.bashrc
export PX4_AUTOPILOT_DIR=$HOME/PX4-Autopilot

# 生效
source ~/.zshrc
```

### 3. 安装MAVROS

```bash
# ROS2 Humble
sudo apt install ros-humble-mavros ros-humble-mavros-extras

# 安装GeographicLib数据集
sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh
```

## 🚀 使用方法

### 方式1：一键启动（推荐） ⭐

```bash
cd /home/lihaomin/project/uav_sys/gazebo_sim

# 启动Gazebo + SITL + MAVROS（all-in-one）
./start_gazebo_sim.sh
```

**你会看到**：
1. Gazebo窗口打开
2. 3架iris无人机出现在编队位置
3. MAVROS连接到每架SITL
4. 日志显示启动信息

### 方式2：分步启动（调试用）

```bash
# 终端1: 启动SITL + Gazebo
cd /home/lihaomin/project/uav_sys/gazebo_sim
./simple_multi_sitl.sh

# 等待Gazebo完全启动（看到无人机）

# 终端2: 启动MAVROS
./mavros_launch.sh 3

# 等待MAVROS连接成功
```

### 方式3：使用PX4官方脚本

```bash
cd ~/PX4-Autopilot

# PX4官方多机启动
Tools/gazebo_multi_vehicle.sh -n 3 -m iris -w empty

# 在另一个终端启动MAVROS
cd /home/lihaomin/project/uav_sys/gazebo_sim
./mavros_launch.sh 3
```

## 🔗 与UAV系统对接

### 完整流程（5个终端）

```bash
# ========== 终端1: Gazebo仿真 ==========
cd /home/lihaomin/project/uav_sys/gazebo_sim
./start_gazebo_sim.sh

# 等待看到Gazebo中的3架无人机

# ========== 终端2: UAV决策系统 ==========
cd /home/lihaomin/project/uav_sys
source setup_env.sh
ros2 launch uav_decision_arbiter multi_uav.launch.py

# 等待系统启动完成

# ========== 终端3: RL平台 ==========
cd /home/lihaomin/project/uav_sys  
source setup_env.sh
python3 rl_platform/rl_platform_node.py

# 看到Pygame界面

# ========== 终端4: 监控（可选）==========
cd /home/lihaomin/project/uav_sys
source setup_env.sh
python3 examples/monitor.py

# ========== 终端5: 测试命令（可选）==========
# 解锁并起飞UAV1
ros2 topic pub /uav1/mavros/cmd/arming mavros_msgs/msg/CommandBool \
  "{value: true}" --once

# 切换到OFFBOARD模式
ros2 service call /uav1/mavros/set_mode mavros_msgs/srv/SetMode \
  "{custom_mode: 'OFFBOARD'}"
```

## 🎮 Gazebo操作

### 视角控制
- **鼠标左键拖动** - 旋转视角
- **鼠标右键拖动** - 平移视角
- **滚轮** - 缩放
- **Shift+鼠标** - 倾斜视角

### 查看无人机状态
- 左侧面板可以选择每架无人机
- 查看位置、速度等信息
- 实时物理仿真

### 暂停/继续
- 工具栏的暂停按钮
- 或按 `空格键`

## 📊 端口分配

| UAV | 系统ID | MAVLINK UDP | GCS UDP | MAVROS命名空间 |
|-----|--------|-------------|---------|---------------|
| UAV1 | 1 | 14540 | 14557 | /uav1 |
| UAV2 | 2 | 14541 | 14558 | /uav2 |
| UAV3 | 3 | 14542 | 14559 | /uav3 |

## 🛠️ 配置说明

### 编队spawn位置

编辑 `gazebo_config.yaml`:

```yaml
uavs:
  - id: 1
    spawn_position:
      x: 0.0    # North
      y: 0.0    # East
      z: 0.0    # Down
      
  - id: 2  
    spawn_position:
      x: 0.0
      y: -5.0  # 右边5米
      z: 0.0
      
  - id: 3
    spawn_position:
      x: 0.0
      y: 5.0   # 左边5米
      z: 0.0
```

### Gazebo世界选择

```bash
# 空世界
./start_gazebo_sim.sh 3 empty

# 带障碍物
./start_gazebo_sim.sh 3 obstacle

# 自定义世界
./start_gazebo_sim.sh 3 my_world
```

## ✈️ 无人机控制测试

### 测试1：解锁和起飞

```bash
# 解锁UAV1
ros2 topic pub /uav1/mavros/cmd/arming \
  mavros_msgs/msg/CommandBool "{value: true}" --once

# 切换OFFBOARD模式
ros2 service call /uav1/mavros/set_mode \
  mavros_msgs/srv/SetMode "{custom_mode: 'OFFBOARD'}"

# 发送位置命令
ros2 topic pub /uav1/mavros/setpoint_position/local \
  geometry_msgs/msg/PoseStamped \
  "{pose: {position: {x: 0, y: 0, z: 5}}}" --rate 10
```

### 测试2：查看位置

```bash
# 查看UAV1位置
ros2 topic echo /uav1/mavros/local_position/pose

# 查看所有无人机状态
ros2 topic echo /uav1/mavros/state
ros2 topic echo /uav2/mavros/state
ros2 topic echo /uav3/mavros/state
```

### 测试3：与RL平台联动

```bash
# RL平台运行后，无人机会自动：
# 1. 被RL算法控制
# 2. 在Gazebo中飞行
# 3. 在RL界面中同步显示
```

## 🎯 完整系统集成

### 启动顺序

```
1. Gazebo仿真 (终端1)
   ./start_gazebo_sim.sh
   ↓ 等待Gazebo窗口显示3架无人机

2. UAV系统 (终端2)
   ros2 launch uav_decision_arbiter multi_uav.launch.py
   ↓ 等待所有节点启动

3. RL平台 (终端3)
   python3 rl_platform/rl_platform_node.py
   ↓ 等待Pygame窗口

4. 观察
   - Gazebo中无人机开始飞行
   - RL界面显示相同轨迹
   - 编队队形保持一致
```

### 数据流

```
RL平台决策
    ↓ /{uav_id}/rl/decision_output
RL Adapter转换
    ↓ /uav/source/rl/cmd
Arbiter仲裁
    ↓ /uav/authoritative_cmd
PX4 Adapter转发
    ↓ /{uav_id}/mavros/setpoint_velocity/cmd_vel
MAVROS通信
    ↓ MAVLINK
PX4 SITL执行
    ↓ 
Gazebo渲染
```

## 🔧 故障排除

### 问题1：Gazebo无法启动

**检查PX4编译**：
```bash
cd ~/PX4-Autopilot
make px4_sitl_default gazebo
```

**检查Gazebo安装**：
```bash
which gazebo
gazebo --version  # 应该是Gazebo 11
```

### 问题2：MAVROS连接失败

**检查SITL是否运行**：
```bash
ps aux | grep px4
netstat -an | grep 14540  # 应该有监听
```

**检查MAVROS日志**：
```bash
tail -f /tmp/mavros_uav1.log
```

### 问题3：无人机不出现在Gazebo

**手动spawn**：
```bash
gz model --spawn-file=~/PX4-Autopilot/Tools/sitl_gazebo/models/iris/iris.sdf \
  --model-name=iris_0 -x 0 -y 0 -z 0.1
```

### 问题4：OFFBOARD模式无法切换

**原因**：需要先发送setpoint

**解决**：
```bash
# 先持续发送setpoint（10Hz），再切换模式
ros2 topic pub /uav1/mavros/setpoint_position/local \
  geometry_msgs/msg/PoseStamped \
  "{pose: {position: {x: 0, y: 0, z: 5}}}" --rate 10 &

# 等待2秒
sleep 2

# 解锁
ros2 topic pub /uav1/mavros/cmd/arming \
  mavros_msgs/msg/CommandBool "{value: true}" --once

# 切换OFFBOARD
ros2 service call /uav1/mavros/set_mode \
  mavros_msgs/srv/SetMode "{custom_mode: 'OFFBOARD'}"
```

## 📐 坐标系

### Gazebo/PX4 SITL
- 使用 **ENU** (East-North-Up)  
- 与MAVROS一致
- 原点：spawn位置

### 编队配置
```
Gazebo spawn位置（ENU）:
  UAV1: (0, 0, 0)
  UAV2: (0, -5, 0)  - 右边5米（东边为负）
  UAV3: (0, 5, 0)   - 左边5米

与formation_sync配置对应:
  formation_offsets (NED):
    uav1: (0, 0, 0)
    uav2: (0, -5, 0)
    uav3: (0, 5, 0)
  
  ENU → NED转换后位置一致 ✓
```

## 🎬 完整演示脚本

创建 `demo_gazebo.sh`：

```bash
#!/bin/bash
# 完整Gazebo仿真演示

echo "=== Gazebo多机仿真完整演示 ==="

# 清理
pkill -9 px4; pkill -9 gz; pkill -f mavros; pkill -f ros2

# 终端1: Gazebo + SITL
gnome-terminal --title="Gazebo仿真" -- bash -c "
    cd /home/lihaomin/project/uav_sys/gazebo_sim
    ./start_gazebo_sim.sh
    exec bash"

sleep 10

# 终端2: UAV系统
gnome-terminal --title="UAV系统" -- bash -c "
    cd /home/lihaomin/project/uav_sys
    source setup_env.sh
    ros2 launch uav_decision_arbiter multi_uav.launch.py
    exec bash"

sleep 5

# 终端3: RL平台
gnome-terminal --title="RL平台" -- bash -c "
    cd /home/lihaomin/project/uav_sys
    source setup_env.sh
    python3 rl_platform/rl_platform_node.py
    exec bash"

echo "✓ 所有组件已启动！"
echo ""
echo "你现在应该看到:"
echo "  1. Gazebo窗口 - 3架iris无人机"
echo "  2. Pygame窗口 - RL决策可视化"
echo "  3. 终端输出 - 系统运行日志"
```

## 📊 监控和调试

### 查看MAVROS话题

```bash
# 查看所有MAVROS话题
ros2 topic list | grep mavros

# 查看UAV1位置
ros2 topic echo /uav1/mavros/local_position/pose

# 查看UAV1状态
ros2 topic echo /uav1/mavros/state
```

### 查看系统仲裁

```bash
# 查看仲裁器状态（多机）
ros2 topic echo /uav/arbiter/status

# 查看编队同步
ros2 topic echo /uav/formation_sync
```

### Gazebo中观察

- **位置轨迹**：无人机应该按RL算法移动
- **编队队形**：保持相对位置
- **实时同步**：与RL界面一致

## 🎨 自定义配置

### 修改spawn位置

编辑 `gazebo_config.yaml` 或直接修改 `start_gazebo_sim.sh` 中的位置计算：

```bash
# 修改编队队形
case $i in
    0) PX_X=0; PX_Y=0 ;;      # UAV1: 中心
    1) PX_X=-5; PX_Y=-3 ;;    # UAV2: V字右后
    2) PX_X=-5; PX_Y=3 ;;     # UAV3: V字左后
esac
```

### 修改无人机数量

```bash
# 启动5架无人机
./start_gazebo_sim.sh  # 修改NUM_UAVS=5

# 同时修改系统配置
编辑 config/multi_uav.yaml:
  uav_ids: ["uav1", "uav2", "uav3", "uav4", "uav5"]
```

### 修改Gazebo世界

```bash
# 使用不同世界
cd ~/PX4-Autopilot/Tools/sitl_gazebo/worlds
ls *.world

# 启动时指定
./start_gazebo_sim.sh 3 windy  # 有风环境
./start_gazebo_sim.sh 3 warehouse  # 仓库环境
```

## 🧪 测试场景

### 场景1：编队起飞

```bash
# 系统运行后，RL算法会自动控制无人机起飞和飞行
# 在Gazebo中观察编队保持
```

### 场景2：单机接管

```bash
# RL控制全部无人机时，中央算力接管UAV2
ros2 topic pub /uav2/central/decision_output std_msgs/msg/String \
  'data: "{\"type\": \"position\", \"position\": {\"x\": 10, \"y\": 0, \"z\": 5, \"yaw\": 0}}"' \
  --rate 2

# 观察Gazebo中UAV2飞向不同位置
# 其他无人机继续编队飞行
```

### 场景3：紧急降落

```bash
# 人类控制UAV1紧急降落
ros2 topic pub /uav1/mavros/setpoint_velocity/cmd_vel \
  geometry_msgs/msg/Twist \
  "{linear: {x: 0, y: 0, z: -0.5}}" --rate 10

# 观察Gazebo中UAV1下降
```

## 📈 性能优化

### 提高仿真速度

```bash
# 设置速度因子
export PX4_SIM_SPEED_FACTOR=2  # 2倍速

# 关闭Gazebo GUI（更快）
./start_gazebo_sim.sh 3 empty headless
```

### 降低资源占用

```bash
# 减少MAVROS插件
编辑脚本中的plugin_allowlist，只保留必要插件

# 降低MAVROS频率
修改stream rate: -r 20（从50降到20Hz）
```

## 🔗 与其他组件对接

### Gazebo → RL平台

```
Gazebo显示位置
    ↓ MAVROS
/{uav_id}/mavros/local_position/pose
    ↓ Formation Sync
/uav/formation_sync
    ↓ RL Platform
更新RL环境初始位置
```

### RL平台 → Gazebo

```
RL决策
    ↓ /{uav_id}/rl/decision_output
RL Adapter
    ↓ /uav/source/rl/cmd  
Arbiter仲裁
    ↓ /uav/authoritative_cmd
PX4 Adapter
    ↓ /{uav_id}/mavros/setpoint_velocity/cmd_vel
MAVROS
    ↓ MAVLINK
PX4 SITL
    ↓ 
Gazebo渲染
```

## 📝 完整系统验证

### 验证清单

- [ ] Gazebo窗口显示3架iris
- [ ] MAVROS连接成功（ros2 topic list | grep mavros）
- [ ] FormationSync读取位置
- [ ] RL平台收到编队同步
- [ ] RL决策发送到Arbiter
- [ ] Gazebo中无人机开始飞行
- [ ] Pygame界面同步显示
- [ ] 编队队形保持

### 验证命令

```bash
# 1. 检查Gazebo
gz model --list  # 应该显示3个iris

# 2. 检查MAVROS连接
ros2 topic hz /uav1/mavros/local_position/pose  # 应该~50Hz

# 3. 检查系统仲裁
ros2 topic echo /uav/arbiter/status --once

# 4. 检查RL决策
ros2 topic hz /uav1/rl/decision_output  # 应该~10Hz
```

## 🎓 学习建议

### 新手路径
1. 先测试单机SITL：`make px4_sitl gazebo`
2. 理解MAVROS：`ros2 topic list | grep mavros`
3. 运行多机脚本：`./start_gazebo_sim.sh`
4. 集成完整系统

### 进阶使用
1. 修改编队队形
2. 添加障碍物
3. 实现路径规划
4. 性能优化

## 🐛 常见错误

### make: *** No rule to make target 'px4_sitl_default'

**解决**：
```bash
cd ~/PX4-Autopilot
make distclean
make px4_sitl_default gazebo
```

### Gazebo models下载慢

**解决**：
```bash
# 手动下载模型
cd ~/.gazebo/models
wget -r -np -nH --cut-dirs=2 \
  http://models.gazebosim.org/
```

### MAVROS: FCU connection timeout

**检查**：
```bash
# PX4是否运行
ps aux | grep px4

# 端口是否监听
netstat -an | grep 14540

# 重启MAVROS
pkill -f mavros
./mavros_launch.sh
```

## 📚 参考资料

- [PX4 SITL文档](https://docs.px4.io/main/en/simulation/)
- [Gazebo教程](http://gazebosim.org/tutorials)
- [MAVROS文档](https://github.com/mavlink/mavros)

## 总结

你现在拥有：
- ✅ **Gazebo 3D仿真** - 真实物理环境
- ✅ **多机PX4 SITL** - 完整飞控仿真
- ✅ **MAVROS连接** - ROS2桥接
- ✅ **一键启动** - 简化部署
- ✅ **系统集成** - 无缝对接

**Gazebo仿真 + RL可视化 = 双重展示！** 🎮🚁

