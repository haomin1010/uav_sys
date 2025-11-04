# 🚀 从这里开始！

## 欢迎使用UAV多源决策融合与编队同步平台！

这是一个**完整**的多机无人机决策平台，已经**100%实现**并可以**立即运行**！

## ⚡ 30秒快速体验

### 选项1：RL可视化平台（最简单）

```bash
# 1. 进入项目
cd /home/lihaomin/project/uav_sys

# 2. 一键环境设置
source setup_env.sh

# 3. 启动RL平台（带可视化界面）
python3 rl_platform/rl_platform_node.py
```

**你会看到**：
- 🖼️ Pygame窗口打开
- 🚁 3架无人机在界面中飞行
- 📊 实时轨迹和状态显示
- 🎮 可交互控制（SPACE暂停，R重置）

### 选项2：Gazebo 3D仿真（需要PX4）⭐

```bash
# 一键启动完整系统（Gazebo + 决策系统 + RL平台）
./run_full_system.sh
```

**你会看到**：
- 🎮 Gazebo窗口 - 3架iris无人机3D仿真
- 🖼️ Pygame窗口 - RL决策2D可视化
- 📊 多个终端 - 系统运行日志

**需要先安装PX4**，详见：`GAZEBO_QUICKSTART.md`

## 🎯 我想做什么？

### 情况1：我想看RL可视化界面

```bash
# 直接运行
python3 rl_platform/rl_platform_node.py
```

→ 阅读：`RL_PLATFORM_GUIDE.md`

### 情况2：我想测试完整系统（多机）

```bash
# 终端1：启动系统
ros2 launch uav_decision_arbiter multi_uav.launch.py

# 终端2：启动RL平台
python3 rl_platform/rl_platform_node.py

# 终端3：模拟PX4（可选）
python3 examples/mock_multi_px4.py
```

→ 阅读：`MULTI_UAV_GUIDE.md`

### 情况3：我想理解系统架构

→ 阅读顺序：
1. `README.md` - 总体介绍
2. `CENTRALIZED_ARCHITECTURE.md` - 集中仲裁
3. `FORMATION_SYNC.md` - 编队同步

### 情况4：我想开发自己的RL算法

→ 阅读：
1. `RL_PLATFORM_GUIDE.md` - RL平台说明
2. 修改：`rl_platform/rl_policy.py`
3. 参考：`rl_platform/README.md`

### 情况5：我遇到了问题

→ 查看：`TROUBLESHOOTING.md`

## 📚 完整文档列表

| 📄 文档 | 🎯 用途 | ⭐ 重要性 |
|---------|---------|-----------|
| **START_HERE.md** | 👈 **从这开始！** | ⭐⭐⭐⭐⭐ |
| **RL_PLATFORM_GUIDE.md** | RL平台完整指南 | ⭐⭐⭐⭐⭐ |
| **GAZEBO_QUICKSTART.md** | Gazebo仿真快速开始⭐ | ⭐⭐⭐⭐⭐ |
| **MULTI_UAV_GUIDE.md** | 多机编队使用 | ⭐⭐⭐⭐⭐ |
| **QUICK_REFERENCE.md** | 快速参考卡片 | ⭐⭐⭐⭐ |
| **FORMATION_SYNC.md** | 编队同步详解 | ⭐⭐⭐⭐ |
| **QUICKSTART.md** | 快速开始指南 | ⭐⭐⭐⭐ |
| **TROUBLESHOOTING.md** | 故障排除 | ⭐⭐⭐ |
| **CENTRALIZED_ARCHITECTURE.md** | 集中仲裁架构 | ⭐⭐⭐ |
| **README.md** | 完整系统介绍 | ⭐⭐⭐ |
| **PROJECT_COMPLETE.md** | 项目验收清单 | ⭐⭐ |
| **FINAL_SUMMARY.md** | 项目总结 | ⭐⭐ |
| **ARCHITECTURE.md** | 原始设计 | ⭐⭐ |

## 🎮 核心功能演示

### 功能1：RL可视化决策

```bash
python3 rl_platform/rl_platform_node.py
```

效果：看到无人机在界面中自主飞行

### 功能2：多机编队

```bash
ros2 launch uav_decision_arbiter multi_uav.launch.py
python3 examples/mock_multi_px4.py
python3 rl_platform/rl_platform_node.py
```

效果：3架无人机保持编队队形飞行

### 功能3：优先级抢占

```bash
# RL运行时，发布中央决策
ros2 topic pub /uav2/central/decision_output std_msgs/msg/String \
  'data: "{\"type\": \"position\", \"position\": {\"x\": 15, \"y\": 5, \"z\": 3, \"yaw\": 0}}"'
```

效果：uav2被中央算力接管，界面上变灰

## 🔑 关键文件

### 配置文件
- `config/default.yaml` - 单机配置
- `config/multi_uav.yaml` - 多机配置（修改编队队形在这里）

### 启动文件
- `launch/system.launch.py` - 单机启动
- `launch/multi_uav.launch.py` - 多机启动

### RL平台
- `rl_platform/rl_platform_node.py` - 主节点
- `rl_platform/rl_policy.py` - 策略（修改算法在这里）
- `rl_platform/rl_visualizer.py` - 界面

### 测试脚本
- `examples/mock_multi_px4.py` - 模拟PX4
- `examples/multi_uav_rl_example.py` - RL控制示例

## 💡 常用命令速查

```bash
# 环境设置
source setup_env.sh

# 启动单机系统
ros2 launch uav_decision_arbiter system.launch.py

# 启动多机系统
ros2 launch uav_decision_arbiter multi_uav.launch.py

# 启动RL平台
python3 rl_platform/rl_platform_node.py

# 启动RL平台（使用脚本）
./run_rl_platform.sh

# 查看系统状态
ros2 topic echo /uav/arbiter/status

# 查看编队同步
ros2 topic echo /uav/formation_sync

# 查看所有话题
ros2 topic list | grep uav
```

## 🎓 学习建议

### 第1天 - 基础
1. ✅ 运行 `python3 rl_platform/rl_platform_node.py`
2. ✅ 体验可视化界面
3. ✅ 理解RL决策流程
4. ✅ 阅读 `RL_PLATFORM_GUIDE.md`

### 第2天 - 集成
1. ✅ 运行完整系统（多机模式）
2. ✅ 理解ROS2通信
3. ✅ 测试优先级抢占
4. ✅ 阅读 `MULTI_UAV_GUIDE.md`

### 第3天 - 定制
1. ✅ 修改编队队形
2. ✅ 修改RL策略
3. ✅ 调整可视化
4. ✅ 阅读代码注释

### 第4天+ - 应用
1. ✅ 集成你的RL算法
2. ✅ 连接实际硬件
3. ✅ 优化性能
4. ✅ 开始研究/应用

## 🆘 需要帮助？

### 问题排查流程
1. 查看 `TROUBLESHOOTING.md`
2. 查看系统日志
3. 查看对应模块文档
4. 检查配置文件

### 常见问题快速解决

**pygame不显示**：
```bash
# WSL需要X Server
export DISPLAY=:0
```

**ROS2导入错误**：
```bash
conda install -c conda-forge libstdcxx-ng -y
```

**编译失败**：
```bash
rm -rf build/ install/ log/
colcon build --symlink-install
```

## 🎁 你拥有的资源

- ✅ 完整的代码库（~3000行）
- ✅ 11份详细文档
- ✅ 6个测试脚本
- ✅ 2个配置文件
- ✅ 可视化界面
- ✅ RL环境和策略
- ✅ 坐标转换工具

## 📋 功能清单

### 核心功能
- [x] 多源决策仲裁（RL/中央/人类）
- [x] 集中仲裁架构（管理多机）
- [x] 轨迹同步（三平台）
- [x] 编队位置同步（相对位置保持）
- [x] 坐标系自动转换（ENU ↔ NED）
- [x] RL决策平台（可视化界面）
- [x] ROS2完整集成

### 测试功能
- [x] 单机测试脚本
- [x] 多机测试脚本
- [x] 监控工具
- [x] 模拟PX4
- [x] RL控制示例

### 文档功能
- [x] 快速开始指南
- [x] 完整使用文档
- [x] 故障排除手册
- [x] 代码示例
- [x] 架构说明

## 🌟 开始你的旅程

选择一个起点：

1. **想快速看效果** → 运行 `./run_rl_platform.sh`
2. **想理解系统** → 阅读 `README.md`
3. **想开发RL算法** → 阅读 `RL_PLATFORM_GUIDE.md`
4. **想多机编队** → 阅读 `MULTI_UAV_GUIDE.md`
5. **遇到问题** → 查看 `TROUBLESHOOTING.md`

---

**项目版本**: 1.1.0  
**完成日期**: 2025-11-04  
**状态**: ✅ 完全可用

## 立即开始！

```bash
source setup_env.sh
python3 rl_platform/rl_platform_node.py
```

祝你使用愉快！🚁✨

