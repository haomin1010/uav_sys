# Gazebo多机仿真 - 快速启动指南

## 🎯 快速启动（3步）

### 1. 配置环境变量（一次性）
```bash
# 添加到~/.zshrc或~/.bashrc
echo 'export DISPLAY=:0' >> ~/.zshrc
echo 'export PX4_AUTOPILOT_DIR=$HOME/project/PX4-Autopilot' >> ~/.zshrc
source ~/.zshrc
```

### 2. 启动仿真
```bash
cd ~/project/uav_sys/gazebo_sim
./启动Gazebo仿真.sh
```

或者直接：
```bash
cd ~/project/uav_sys/gazebo_sim
export DISPLAY=:0
./start_gazebo_sim.sh
```

### 3. 观察结果
- **Gazebo窗口**应该自动弹出，显示3架无人机
- 终端显示所有启动状态

## ❌ 没有GUI显示？

### 快速诊断
```bash
cd ~/project/uav_sys/gazebo_sim
./fix_wsl_display.sh
```

这个脚本会：
- ✅ 检测X11配置
- ✅ 自动修复DISPLAY
- ✅ 给出具体解决方案

### 常见问题

#### 问题1："gzclient未启动" 或 "Gazebo GUI: 未运行"

**原因**：X11显示未配置

**解决**：
```bash
export DISPLAY=:0
xeyes  # 测试，应该显示眼睛窗口
```

如果`xeyes`也不显示，查看[WSL2_GUI_SETUP.md](./WSL2_GUI_SETUP.md)

#### 问题2："Gazebo服务器启动失败"

**解决**：查看日志
```bash
cat /tmp/gzserver.log
cat /tmp/gzclient.log
```

#### 问题3："PX4-Autopilot未找到"

**解决**：
```bash
export PX4_AUTOPILOT_DIR=/path/to/your/PX4-Autopilot
# 或安装PX4
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
make px4_sitl_default gazebo-classic
```

## 📁 文件说明

| 文件 | 说明 |
|------|------|
| `启动Gazebo仿真.sh` | **主启动脚本**（自动配置DISPLAY） |
| `start_gazebo_sim.sh` | 核心启动逻辑 |
| `fix_wsl_display.sh` | X11诊断和修复工具 |
| `WSL2_GUI_SETUP.md` | WSL2 GUI详细配置指南 |

## 🔍 验证系统

启动后，在新终端验证：

```bash
# 1. 检查进程
ps aux | grep gz
# 应该看到gzserver和gzclient

# 2. 检查MAVROS
ros2 topic list | grep mavros

# 3. 检查无人机状态
ros2 topic echo /uav1/mavros/state

# 4. 查看位置
ros2 topic echo /uav1/mavros/local_position/pose
```

## 📊 系统架构

```
┌─────────────────────────────────────────┐
│         Gazebo Classic GUI              │  ← 3D可视化
│     (gzclient + gzserver)               │
└──────────────┬──────────────────────────┘
               │
┌──────────────┴──────────────────────────┐
│      PX4 SITL (3个实例)                 │  ← 飞控仿真
│      - uav1 (0, 0)                      │
│      - uav2 (0, -5)                     │
│      - uav3 (0, 5)                      │
└──────────────┬──────────────────────────┘
               │
┌──────────────┴──────────────────────────┐
│       MAVROS (3个节点)                  │  ← ROS2接口
│       - /uav1/mavros/*                  │
│       - /uav2/mavros/*                  │
│       - /uav3/mavros/*                  │
└─────────────────────────────────────────┘
```

## 🛑 停止仿真

```bash
# 方法1：在启动终端按 Ctrl+C

# 方法2：强制停止所有
pkill -9 px4
pkill -9 gzserver
pkill -9 gzclient
pkill -f mavros
```

## 📝 日志位置

```bash
# Gazebo
/tmp/gzserver.log    # 服务器日志
/tmp/gzclient.log    # GUI日志

# PX4
/tmp/px4_uav1.log    # UAV1日志
/tmp/px4_uav2.log    # UAV2日志
/tmp/px4_uav3.log    # UAV3日志

# MAVROS
/tmp/mavros_uav1.log
/tmp/mavros_uav2.log
/tmp/mavros_uav3.log

# 模型生成
/tmp/spawn_0.log
/tmp/spawn_1.log
/tmp/spawn_2.log
```

## 🔧 高级配置

### 修改无人机数量

编辑 `start_gazebo_sim.sh`：
```bash
NUM_UAVS=5  # 改成你想要的数量
```

### 修改初始位置

编辑 `start_gazebo_sim.sh` 中的位置计算：
```bash
case $i in
    0) PX_X=0; PX_Y=0 ;;
    1) PX_X=0; PX_Y=-5 ;;
    2) PX_X=0; PX_Y=5 ;;
    3) PX_X=5; PX_Y=0 ;;   # 添加更多
    4) PX_X=-5; PX_Y=0 ;;
esac
```

### 使用无头模式（无GUI）

如果不需要Gazebo GUI：
```bash
# 编辑start_gazebo_sim.sh，注释掉这行：
# gzclient --verbose 2>&1 | tee /tmp/gzclient.log &
```

## 📚 更多资源

- [PX4用户指南](https://docs.px4.io/)
- [Gazebo教程](http://gazebosim.org/tutorials)
- [MAVROS文档](https://github.com/mavlink/mavros)
- [WSL GUI配置](./WSL2_GUI_SETUP.md)

## 🆘 获取帮助

1. **查看诊断脚本**：`./fix_wsl_display.sh`
2. **查看详细日志**：`cat /tmp/*.log`
3. **检查进程状态**：`ps aux | grep -E "(gz|px4|mavros)"`

---

**提示**：第一次启动可能需要20-30秒来加载所有模型和连接。请耐心等待！
