# 快速开始指南

5分钟快速启动UAV多源决策系统。

---

## 📋 前置要求

- Ubuntu 20.04/22.04
- ROS2 Humble
- Conda (Python 3.10)

---

## ⚡ 快速启动（3步）

### 步骤1：环境配置（一次性）

```bash
# 克隆项目
cd ~/project
git clone <repo_url> uav_sys
cd uav_sys

# 创建conda环境
conda create -n uav_sys python=3.10 -y
conda activate uav_sys

# 安装依赖
pip install -r requirements.txt

# 编译ROS2包
source /opt/ros/humble/setup.zsh
colcon build --symlink-install
```

### 步骤2：启动系统

```bash
# 使用便捷脚本（自动source环境）
source setup_env.sh

# 方式A：单机模式
ros2 launch uav_decision_arbiter system.launch.py

# 方式B：多机模式（推荐）
ros2 launch uav_decision_arbiter multi_uav.launch.py
```

### 步骤3：验证运行

```bash
# 新终端：启动监控
python3 examples/monitor.py

# 应该看到：
# ✓ 仲裁器在线
# ✓ 同步器在线
# ✓ 3个适配器在线
```

---

## 🎮 完整演示

### 演示1：RL平台可视化

```bash
# 终端1：系统
ros2 launch uav_decision_arbiter multi_uav.launch.py

# 终端2：RL平台
python3 rl_platform/rl_platform_node.py

# 效果：Pygame窗口显示3架无人机飞行
```

### 演示2：Gazebo 3D仿真

```bash
# 一键启动（Gazebo + 系统 + RL）
./run_full_system.sh

# 效果：
# - Gazebo窗口：3D无人机模型
# - Pygame窗口：2D轨迹
# - 终端：系统日志
```

### 演示3：AirSim集成

```bash
# 1. 配置AirSim（一次性）
pip install airsim
cp airsim_config/settings.json ~/Documents/AirSim/
# 编辑 config/multi_uav.yaml: use_airsim_api: true

# 2. 启动AirSim（虚幻引擎）

# 3. 启动系统
ros2 launch uav_decision_arbiter multi_uav.launch.py

# 4. 启动中央算力
python3 examples/central_planner_airsim.py

# 效果：AirSim中无人机按waypoints飞行
```

---

## 📊 快速参考

### 常用命令

```bash
# 环境激活
source setup_env.sh

# 启动系统
ros2 launch uav_decision_arbiter multi_uav.launch.py

# 监控状态
python3 examples/monitor.py

# RL平台
python3 rl_platform/rl_platform_node.py

# Gazebo仿真
./gazebo_sim/start_gazebo_sim.sh

# 完整演示
./run_full_system.sh
```

### 重要话题

| 话题 | 说明 |
|------|------|
| `/uav/arbiter/status` | 仲裁器状态 |
| `/uav/authoritative_cmd` | 当前生效命令 |
| `/{uav_id}/rl/decision_output` | RL决策输出 |
| `/{uav_id}/central/decision_output` | 中央算力决策 |
| `/uav/formation_sync` | 编队同步 |

### 配置文件

| 文件 | 用途 |
|------|------|
| `config/default.yaml` | 单机配置 |
| `config/multi_uav.yaml` | 多机配置 |
| `airsim_config/settings.json` | AirSim配置 |

### 优先级设置

| 决策源 | 优先级 | 用途 |
|--------|--------|------|
| Human | 200 | 紧急接管 |
| Central | 150 | 中央算力 |
| RL | 100 | 自主决策 |

---

## 🧪 测试工具

```bash
# RL决策测试
python3 examples/test_rl_publisher.py

# 中央算力测试
python3 examples/test_central_publisher.py

# 多机PX4测试
python3 examples/mock_multi_px4.py

# AirSim连接测试
./test_airsim_integration.sh
```

---

## 🎯 使用场景

### 场景1：只运行仲裁系统

```bash
ros2 launch uav_decision_arbiter multi_uav.launch.py
python3 examples/monitor.py
```

### 场景2：系统 + RL可视化

```bash
# 终端1
ros2 launch uav_decision_arbiter multi_uav.launch.py

# 终端2
python3 rl_platform/rl_platform_node.py
```

### 场景3：完整仿真（推荐）

```bash
./run_full_system.sh
```

### 场景4：AirSim对接

```bash
# 1. 启动AirSim
# 2. 配置：config/multi_uav.yaml (use_airsim_api: true)
ros2 launch uav_decision_arbiter multi_uav.launch.py
python3 examples/central_planner_airsim.py
```

---

## 🔧 常见问题

### 问题1：环境变量未设置

```bash
# 解决：使用便捷脚本
source setup_env.sh
```

### 问题2：rclpy导入失败

```bash
# 解决：检查libstdc++
conda install -c conda-forge libstdcxx-ng
```

### 问题3：colcon build失败

```bash
# 解决：清理后重新编译
rm -rf build install log
colcon build --symlink-install
```

### 问题4：节点无法通信

```bash
# 检查：ROS_DOMAIN_ID
echo $ROS_DOMAIN_ID  # 应该相同（默认0）

# 检查：ROS2守护进程
ros2 daemon stop
ros2 daemon start
```

**更多故障排除**：`TROUBLESHOOTING.md`

---

## 📖 详细文档

| 文档 | 说明 |
|------|------|
| **README.md** | 系统概述和架构 |
| **FEATURES.md** | 功能详细说明 |
| **MULTI_UAV_GUIDE.md** | 多机开发指南 |
| **AIRSIM_INTEGRATION.md** | AirSim对接 |
| **GAZEBO_QUICKSTART.md** | Gazebo仿真 |
| **RL_PLATFORM_GUIDE.md** | RL平台开发 |
| **TROUBLESHOOTING.md** | 故障排除 |

---

## 🎓 学习路径

### 新手路径（30分钟）

1. **快速启动**（5分钟）
   ```bash
   source setup_env.sh
   ros2 launch uav_decision_arbiter multi_uav.launch.py
   python3 examples/monitor.py
   ```

2. **运行演示**（10分钟）
   ```bash
   # 启动RL可视化
   python3 rl_platform/rl_platform_node.py
   ```

3. **测试决策**（10分钟）
   ```bash
   # 发布RL决策
   python3 examples/test_rl_publisher.py
   
   # 发布中央决策
   python3 examples/test_central_publisher.py
   ```

4. **阅读文档**（5分钟）
   - README.md - 系统概述
   - FEATURES.md - 功能说明

### 进阶路径（2小时）

1. **多机编队**（30分钟）
   - 阅读：MULTI_UAV_GUIDE.md
   - 测试：`python3 examples/mock_multi_px4.py`

2. **Gazebo仿真**（30分钟）
   - 阅读：GAZEBO_QUICKSTART.md
   - 启动：`./run_full_system.sh`

3. **AirSim集成**（30分钟）
   - 阅读：AIRSIM_INTEGRATION.md
   - 测试：`./test_airsim_integration.sh`

4. **RL开发**（30分钟）
   - 阅读：RL_PLATFORM_GUIDE.md
   - 修改：`rl_platform/rl_policy.py`

### 开发路径（1天）

1. 自定义决策源
2. 修改仲裁逻辑
3. 添加新功能
4. 扩展到更多无人机

**开发文档**：README.md - 开发指南部分

---

## 🚀 下一步

- **基础使用** → 阅读 `FEATURES.md`
- **多机开发** → 阅读 `MULTI_UAV_GUIDE.md`
- **3D仿真** → 运行 `./run_full_system.sh`
- **AirSim对接** → 阅读 `AIRSIM_INTEGRATION.md`
- **RL开发** → 阅读 `RL_PLATFORM_GUIDE.md`

**遇到问题？** 查看 `TROUBLESHOOTING.md`

---

**开始你的UAV开发之旅！** 🎉
