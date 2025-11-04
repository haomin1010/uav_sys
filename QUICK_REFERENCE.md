# 快速参考卡片

## 🚀 启动命令

### 单机模式
```bash
source setup_env.sh
ros2 launch uav_decision_arbiter system.launch.py
```

### 多机编队模式
```bash
source setup_env.sh
ros2 launch uav_decision_arbiter multi_uav.launch.py
```

## 📊 关键配置

### 单机 vs 多机

```yaml
# 单机模式 (config/default.yaml)
arbiter:
  ros__parameters:
    centralized_mode: false
    uav_ids: ["uav1"]

# 多机模式 (config/multi_uav.yaml)
arbiter:
  ros__parameters:
    centralized_mode: true
    uav_ids: ["uav1", "uav2", "uav3"]
```

### 编队队形

```yaml
formation_sync:
  ros__parameters:
    formation_offsets:
      uav1: {x: 0.0, y: 0.0, z: 0.0}    # 长机
      uav2: {x: 0.0, y: -5.0, z: 0.0}   # 右翼
      uav3: {x: 0.0, y: 5.0, z: 0.0}    # 左翼
```

## 🎮 发布命令

### 广播（所有无人机）
```bash
ros2 topic pub /rl/decision_output std_msgs/msg/String \
  'data: "{\"action\": [1.0, 0.0, 0.5, 0.0]}"'
```

### 定向（指定无人机）
```bash
ros2 topic pub /uav2/central/decision_output std_msgs/msg/String \
  'data: "{\"type\": \"position\", \"position\": {\"x\": 5, \"y\": 3, \"z\": 2, \"yaw\": 0}}"'
```

## 📡 关键话题

```bash
# 仲裁器状态（显示每架无人机的决策源）
ros2 topic echo /uav/arbiter/status

# 权威命令（包含target_uav_id）
ros2 topic echo /uav/authoritative_cmd

# 编队同步（多机位置信息）
ros2 topic echo /uav/formation_sync

# PX4位置（每架独立）
ros2 topic echo /uav1/mavros/local_position/pose
ros2 topic echo /uav2/mavros/local_position/pose
```

## 🧪 测试脚本

### 单机测试
```bash
# RL控制
python3 examples/test_rl_publisher.py

# 中央算力
python3 examples/test_central_publisher.py

# 监控
python3 examples/monitor.py
```

### 多机测试
```bash
# 模拟多机PX4
python3 examples/mock_multi_px4.py

# 多机RL控制
python3 examples/multi_uav_rl_example.py
```

## 🌐 坐标系

```
MAVROS (PX4): ENU
  X → East
  Y → North
  Z → Up

AirSim: NED
  X → North
  Y → East
  Z → Down

转换：
  NED.x = ENU.y
  NED.y = ENU.x
  NED.z = -ENU.z
```

## 🎯 优先级

```
Human (200) > Central (150) > RL (100)
```

高优先级自动抢占低优先级

## 📁 文档索引

| 文档 | 用途 |
|------|------|
| `QUICKSTART.md` | ⭐ 新手入门 |
| `README.md` | 完整功能介绍 |
| `MULTI_UAV_GUIDE.md` | ⭐ 多机使用指南 |
| `FORMATION_SYNC.md` | 编队同步详解 |
| `CENTRALIZED_ARCHITECTURE.md` | 集中仲裁架构 |
| `INITIAL_POSITION_SYNC.md` | 单机位置同步 |
| `TROUBLESHOOTING.md` | 故障排除 |
| `ARCHITECTURE.md` | 系统架构设计 |

## 🔧 常用调试命令

```bash
# 查看所有节点
ros2 node list

# 查看所有话题
ros2 topic list

# 查看某话题信息
ros2 topic info /uav/authoritative_cmd

# 查看节点信息
ros2 node info /arbiter

# 录制所有话题
ros2 bag record -a

# 可视化节点图
ros2 run rqt_graph rqt_graph
```

## ⚙️ 环境设置

```bash
# 方式1：自动脚本
source setup_env.sh

# 方式2：手动
conda activate uav_sys
source /opt/ros/humble/setup.zsh
source install/setup.zsh
```

## 🐛 常见问题

### libstdc++错误
```bash
conda install -c conda-forge libstdcxx-ng -y
```

### 参数文件错误
检查 `ros__parameters` 格式是否正确

### 编译失败
```bash
rm -rf build/ install/ log/
colcon build --symlink-install
```

## 📞 获取帮助

1. 查看对应文档（见上表）
2. 检查 `TROUBLESHOOTING.md`
3. 查看系统日志：`~/.ros/log/`

---

**版本**: 1.1.0  
**更新**: 2025-11-04  
**特性**: 单机 + 多机编队 + 位置同步

