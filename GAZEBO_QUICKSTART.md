# Gazebo仿真快速开始指南

## 🎯 目标

使用**Gazebo + PX4 SITL**作为真实的物理仿真平台，提供3D可视化展示，与RL平台和AirSim并行运行。

## 📊 系统架构

```
你的完整系统：

┌─────────────┐   ┌─────────────┐   ┌─────────────┐
│  RL平台     │   │ 中央算力    │   │ 人类控制    │
│  (Pygame)   │   │ (AirSim)    │   │  (遥控)     │
└──────┬──────┘   └──────┬──────┘   └──────┬──────┘
       │                 │                 │
       └─────────┬───────┴─────────┬───────┘
                 ▼                 ▼
          ┌─────────────────────────────┐
          │    Arbiter仲裁器             │
          └────────┬────────────────────┘
                   │
      ┌────────────┴────────────┬──────────────┐
      ▼                         ▼              ▼
┌──────────┐             ┌───────────┐   ┌──────────┐
│ PX4真机  │             │ AirSim    │   │ Gazebo   │⭐
│  (可选)  │             │   仿真     │   │  仿真     │
└──────────┘             └───────────┘   └──────────┘
                                              ↑
                                         3D物理仿真
```

**Gazebo的位置**：
- 作为**PX4 Adapter之后**的执行平台
- 提供真实的物理仿真和3D可视化
- 与AirSim并行，可替代或互补

## ⚡ 10分钟快速开始

### 步骤1：安装PX4（首次）

```bash
# 1. 克隆PX4
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive

# 2. 安装依赖
cd PX4-Autopilot
bash ./Tools/setup/ubuntu.sh

# 3. 编译
make px4_sitl_default gazebo

# 4. 测试
make px4_sitl gazebo  # 应该看到Gazebo和一架无人机

# Ctrl+C停止测试
```

### 步骤2：设置环境变量

```bash
# 添加到 ~/.zshrc
echo 'export PX4_AUTOPILOT_DIR=$HOME/PX4-Autopilot' >> ~/.zshrc
source ~/.zshrc
```

### 步骤3：启动Gazebo仿真（3架无人机）

```bash
cd /home/lihaomin/project/uav_sys/gazebo_sim
./start_gazebo_sim.sh
```

**等待启动**（约10-15秒）：
- Gazebo窗口打开
- 3架iris无人机出现
- MAVROS连接成功

### 步骤4：启动UAV系统

```bash
# 新终端
cd /home/lihaomin/project/uav_sys
source setup_env.sh
ros2 launch uav_decision_arbiter multi_uav.launch.py
```

### 步骤5：启动RL平台

```bash
# 新终端
cd /home/lihaomin/project/uav_sys
source setup_env.sh
python3 rl_platform/rl_platform_node.py
```

### 步骤6：观察结果！

你现在有**3个可视化界面同时运行**：

1. **Gazebo** - 3D物理仿真，真实渲染
2. **Pygame（RL）** - 2D轨迹，决策可视化
3. **终端日志** - 系统状态信息

所有3个界面应该显示**相同的无人机运动**！

## 🎬 一键启动脚本

创建 `run_full_system_gazebo.sh`：

```bash
#!/bin/bash
echo "=== 启动完整系统（Gazebo版）==="

BASE_DIR="/home/lihaomin/project/uav_sys"

# 检查PX4
if [ ! -d "$HOME/PX4-Autopilot" ]; then
    echo "❌ 请先安装PX4-Autopilot"
    exit 1
fi

# 清理
pkill -9 px4; pkill -9 gz; pkill -f mavros; pkill -f ros2
sleep 2

# 终端1: Gazebo
gnome-terminal --title="1. Gazebo仿真" --geometry=80x24+0+0 -- bash -c "
    cd $BASE_DIR/gazebo_sim
    ./start_gazebo_sim.sh
    exec bash"

echo "等待Gazebo启动..."
sleep 12

# 终端2: UAV系统
gnome-terminal --title="2. UAV决策系统" --geometry=80x24+700+0 -- bash -c "
    cd $BASE_DIR
    source setup_env.sh
    ros2 launch uav_decision_arbiter multi_uav.launch.py
    exec bash"

sleep 6

# 终端3: RL平台
gnome-terminal --title="3. RL平台" --geometry=80x24+1400+0 -- bash -c "
    cd $BASE_DIR
    source setup_env.sh
    python3 rl_platform/rl_platform_node.py
    exec bash"

echo ""
echo "✅ 完整系统已启动！"
echo ""
echo "你应该看到:"
echo "  🎮 Gazebo窗口 - 3架iris无人机"
echo "  🖼️  Pygame窗口 - RL决策界面"
echo "  📊 3个终端 - 系统日志"
echo ""
echo "观察:"
echo "  1. Gazebo中无人机圆形编队飞行"
echo "  2. Pygame中相同的轨迹"
echo "  3. 编队队形保持一致"
echo ""
echo "停止系统:"
echo "  pkill -9 px4; pkill -9 gz; pkill -f mavros; pkill -f ros2"
echo ""
```

## 🎮 Gazebo vs AirSim vs RL界面对比

| 特性 | Gazebo | AirSim | RL Pygame |
|------|--------|--------|-----------|
| 3D渲染 | ✅ 真实 | ✅ 虚幻引擎 | ❌ 2D |
| 物理仿真 | ✅ 高精度 | ✅ 中等 | ❌ 简化 |
| 性能开销 | 中 | 高 | 低 |
| 易用性 | ✅ 简单 | 中等 | ✅ 最简单 |
| PX4集成 | ✅ 原生 | 需要插件 | N/A |
| 实时性 | ✅ 好 | 好 | ✅ 最好 |
| 适用场景 | 真实仿真 | 视觉算法 | 快速迭代 |

**推荐组合**：
- **开发阶段**：RL Pygame（快速）
- **测试阶段**：Gazebo（真实）
- **视觉算法**：AirSim（高质量渲染）

## 🔧 配置对接

### 确保spawn位置一致

**Gazebo配置**（`gazebo_config.yaml`）：
```yaml
spawn_position:  # NED
  uav1: {x: 0, y: 0, z: 0}
  uav2: {x: 0, y: -5, z: 0}
  uav3: {x: 0, y: 5, z: 0}
```

**系统配置**（`config/multi_uav.yaml`）：
```yaml
formation_offsets:  # NED
  uav1: {x: 0, y: 0, z: 0}
  uav2: {x: 0, y: -5, z: 0}
  uav3: {x: 0, y: 5, z: 0}
```

**保持一致！**这样编队相对位置才会正确。

## 📊 三平台对比展示

启动完整系统后，你可以同时看到：

```
┌────────── Gazebo（真实3D）──────────┐
│                                      │
│   🚁        🚁        🚁            │
│  UAV3      UAV1      UAV2            │
│   ↑         ↑         ↑              │
│   实时物理仿真、光影效果              │
└──────────────────────────────────────┘

┌────────── RL Pygame（决策2D）───────┐
│  Y                                   │
│  ↑                                   │
│  │  ○         ○         ○           │
│  │ uav3      uav1     uav2          │
│  └──→ X                              │
│  轨迹、速度向量、控制信息             │
└──────────────────────────────────────┘

┌────────── AirSim（可选）────────────┐
│  虚幻引擎高质量渲染                   │
│  （如需视觉算法开发）                 │
└──────────────────────────────────────┘

所有界面显示相同的无人机运动！
```

## 🎯 典型工作流

### 工作流1：纯仿真开发

```
Gazebo SITL ← 决策系统 ← RL平台(Pygame)
    ↓
  真实仿真
```

不需要真实硬件，完全在仿真中开发和测试。

### 工作流2：硬件在环

```
真实PX4 ← 决策系统 ← RL平台
    ↓
Gazebo SITL（并行）
    ↓
对比验证
```

真机和仿真同时运行，对比结果。

### 工作流3：多平台验证

```
决策系统
    ├─→ Gazebo SITL（物理仿真）
    ├─→ AirSim（视觉仿真）
    └─→ RL Pygame（快速可视化）
```

三个平台同时验证算法正确性。

## 💻 系统要求

- **操作系统**: Ubuntu 20.04/22.04
- **内存**: 8GB+（推荐16GB）
- **显卡**: 集成显卡即可（独显更好）
- **CPU**: 4核+（多机仿真需要）
- **磁盘**: 10GB+（PX4+Gazebo模型）

## 📝 安装检查清单

- [ ] Ubuntu 20.04/22.04
- [ ] ROS2 Humble已安装
- [ ] PX4-Autopilot已克隆
- [ ] PX4已编译（make px4_sitl_default gazebo）
- [ ] MAVROS已安装
- [ ] Gazebo 11已安装（PX4编译时自动安装）
- [ ] 环境变量PX4_AUTOPILOT_DIR已设置

## 🚀 现在开始！

```bash
# 最简单的方式
cd /home/lihaomin/project/uav_sys/gazebo_sim
./start_gazebo_sim.sh

# 等待Gazebo窗口出现，然后在新终端：
cd /home/lihaomin/project/uav_sys
source setup_env.sh
ros2 launch uav_decision_arbiter multi_uav.launch.py

# 再新终端：
python3 rl_platform/rl_platform_node.py
```

**享受你的3D仿真+RL决策系统吧！** 🎮🚁✨

---

**下一步**：查看 `gazebo_sim/README.md` 了解详细配置和高级用法。

