# 故障排除指南

## 常见问题与解决方案

### 1. ImportError: libstdc++.so.6: version `GLIBCXX_3.4.30' not found

**问题描述**：
```
ImportError: /home/xxx/anaconda3/envs/uav_sys/bin/../lib/libstdc++.so.6: 
version `GLIBCXX_3.4.30' not found
```

**原因**：Conda环境中的`libstdc++`版本过旧，与ROS2要求的版本不兼容。

**解决方案**：

在conda环境中安装更新的libstdc++：
```bash
conda activate uav_sys
conda install -c conda-forge libstdcxx-ng -y
```

验证修复：
```bash
python3 -c "import rclpy; print('成功！')"
```

**已自动修复**：`setup_env.sh`脚本已包含自动检查和修复此问题的代码。

---

### 2. ROS2参数文件解析错误

**问题描述**：
```
Error: Cannot have a value before ros__parameters at line X
```

**原因**：ROS2的YAML参数文件格式要求每个节点配置必须包含`ros__parameters`键。

**解决方案**：

配置文件格式应该是：
```yaml
node_name:
  ros__parameters:
    parameter1: value1
    parameter2: value2
```

而不是：
```yaml
node_name:
  parameter1: value1
  parameter2: value2
```

**已修复**：`config/default.yaml`已使用正确格式。

---

### 3. 节点无法启动

**问题描述**：启动launch文件后节点立即退出。

**检查步骤**：

1. 检查conda环境是否激活：
```bash
echo $CONDA_DEFAULT_ENV  # 应该显示 uav_sys
```

2. 检查ROS2环境是否source：
```bash
echo $ROS_DISTRO  # 应该显示 humble 或其他版本
```

3. 检查工作空间是否编译：
```bash
ls install/  # 应该看到setup.zsh等文件
```

**解决方案**：
```bash
source setup_env.sh  # 自动配置所有环境
```

---

### 4. MAVROS连接失败（PX4适配器）

**问题描述**：PX4适配器无法接收到MAVROS话题数据。

**检查步骤**：

1. 确认MAVROS是否运行：
```bash
ros2 node list | grep mavros
```

2. 检查MAVROS话题：
```bash
ros2 topic list | grep mavros
```

3. 确认PX4是否连接：
```bash
ros2 topic echo /mavros/state
```

**解决方案**：

如果不需要实际PX4连接，在配置文件中设置：
```yaml
px4_adapter:
  ros__parameters:
    use_mavros: false
```

---

### 5. AirSim连接失败

**问题描述**：AirSim适配器无法连接到AirSim。

**解决方案**：

如果不需要实际AirSim连接，在配置文件中设置：
```yaml
airsim_adapter:
  ros__parameters:
    use_airsim_api: false
```

如果需要连接AirSim：
1. 确认AirSim正在运行
2. 检查IP地址配置
3. 安装AirSim Python包：
```bash
conda activate uav_sys
pip install airsim
```

---

### 6. 话题没有数据

**问题描述**：使用`ros2 topic echo`看不到任何数据。

**检查步骤**：

1. 确认话题存在：
```bash
ros2 topic list
```

2. 查看话题信息：
```bash
ros2 topic info /uav/authoritative_cmd
```

3. 查看节点是否运行：
```bash
ros2 node list
```

**常见原因**：
- 没有决策源在发布数据
- 仲裁器判定所有命令都过期/无效
- 节点配置错误导致话题名不匹配

**解决方案**：
- 运行测试发布器：`python3 examples/test_rl_publisher.py`
- 查看仲裁器状态：`ros2 topic echo /uav/arbiter/status`

---

### 7. 编译失败

**问题描述**：`colcon build`失败。

**解决方案**：

1. 清理并重新编译：
```bash
rm -rf build/ install/ log/
colcon build --symlink-install
```

2. 检查Python语法错误：
```bash
python3 -m py_compile src/uav_decision_arbiter/uav_decision_arbiter/*.py
```

3. 确认package.xml和setup.py格式正确

---

### 8. 权限问题

**问题描述**：脚本无法执行。

**解决方案**：
```bash
chmod +x setup_env.sh
chmod +x examples/*.py
```

---

### 9. Python模块找不到

**问题描述**：`ModuleNotFoundError: No module named 'xxx'`

**解决方案**：

1. 确认conda环境激活：
```bash
which python3  # 应该指向anaconda3/envs/uav_sys/bin/python3
```

2. 确认ROS2工作空间已source：
```bash
source install/setup.zsh
```

3. 重新编译（使用--symlink-install）：
```bash
colcon build --symlink-install
```

---

### 10. 时间戳/时钟问题

**问题描述**：命令立即过期或时间不同步。

**解决方案**：

1. 检查系统时间：
```bash
date
```

2. 如果使用仿真时钟，确保配置正确：
```bash
ros2 param get /use_sim_time
```

3. 调整命令超时时间（在config/default.yaml中）：
```yaml
command_timeout: 2.0  # 增加超时时间
```

---

## 调试技巧

### 查看日志

ROS2日志位置：
```bash
ls ~/.ros/log/
```

查看最新日志：
```bash
cat ~/.ros/log/latest/xxx.log
```

### 实时监控

使用提供的监控脚本：
```bash
python3 examples/monitor.py
```

### 话题调试

手动发布测试消息：
```bash
ros2 topic pub /rl/decision_output std_msgs/msg/String \
  'data: "{\"action\": [1.0, 0.0, 0.5, 0.0]}"' --once
```

录制话题用于回放：
```bash
ros2 bag record -a
```

### 可视化

使用rqt_graph查看节点关系：
```bash
ros2 run rqt_graph rqt_graph
```

---

## 获取帮助

如果以上方法都无法解决问题：

1. 查看完整日志输出
2. 检查ROS2和系统版本兼容性
3. 确认所有依赖都已正确安装
4. 参考ROS2官方文档：https://docs.ros.org/

---

## 环境检查清单

运行此脚本检查环境：

```bash
#!/bin/bash
echo "=== 环境检查 ==="
echo "Conda环境: $CONDA_DEFAULT_ENV"
echo "Python路径: $(which python3)"
echo "ROS版本: $ROS_DISTRO"
echo "工作空间: $(pwd)"
echo ""
echo "=== 测试导入 ==="
python3 -c "import rclpy; print('✓ rclpy')" 2>&1
python3 -c "import numpy; print('✓ numpy')" 2>&1
echo ""
echo "=== ROS2节点 ==="
ros2 node list
echo ""
echo "=== ROS2话题 ==="
ros2 topic list | head -10
```

保存为`check_env.sh`并运行：
```bash
chmod +x check_env.sh
./check_env.sh
```

