#!/bin/bash
# 完整系统一键启动脚本
# 启动：Gazebo仿真 + UAV决策系统 + RL平台

cat << "EOF"
╔════════════════════════════════════════════════════════════╗
║     UAV多源决策融合与编队同步平台 - 完整系统启动          ║
╚════════════════════════════════════════════════════════════╝
EOF

echo ""
echo "系统组件："
echo "  1. Gazebo 3D仿真（3架iris无人机）"
echo "  2. PX4 SITL（软件在环仿真）"
echo "  3. MAVROS（ROS2桥接）"
echo "  4. UAV决策系统（Arbiter + Adapters）"
echo "  5. RL决策平台（Pygame可视化）"
echo ""
echo "================================================================"

BASE_DIR="/home/lihaomin/project/uav_sys"

# 检查PX4
if [ ! -d "$HOME/PX4-Autopilot" ]; then
    echo "❌ 未找到PX4-Autopilot"
    echo ""
    echo "请先安装："
    echo "  git clone https://github.com/PX4/PX4-Autopilot.git --recursive"
    echo "  cd PX4-Autopilot"
    echo "  make px4_sitl_default gazebo"
    echo ""
    exit 1
fi

# 清理旧进程
echo "清理旧进程..."
pkill -9 px4 2>/dev/null
pkill -9 gzserver 2>/dev/null
pkill -9 gzclient 2>/dev/null
pkill -f mavros_node 2>/dev/null
pkill -f "ros2 launch" 2>/dev/null
sleep 2
echo "✓ 清理完成"
echo ""

# 询问用户确认
read -p "准备启动完整系统，继续？(y/n) " -n 1 -r
echo ""
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "已取消"
    exit 0
fi

echo "================================================================"
echo "步骤1/5: 启动Gazebo仿真 + PX4 SITL"
echo "================================================================"

gnome-terminal --title="[1] Gazebo仿真" --geometry=100x30+0+0 -- bash -c "
    echo '启动Gazebo + PX4 SITL...'
    cd $BASE_DIR/gazebo_sim
    ./start_gazebo_sim.sh
    exec bash"

echo "等待Gazebo启动..."
echo "（大约需要15秒，请耐心等待Gazebo窗口出现）"
sleep 15

echo ""
echo "================================================================"
echo "步骤2/5: 验证Gazebo"
echo "================================================================"

# 检查Gazebo
if pgrep -x "gzserver" > /dev/null; then
    echo "✓ Gazebo服务器运行中"
else
    echo "❌ Gazebo启动失败，请检查终端1的输出"
    exit 1
fi

# 检查MAVROS
if pgrep -f "mavros_node" > /dev/null; then
    echo "✓ MAVROS运行中"
else
    echo "⚠️  MAVROS未检测到，但继续..."
fi

sleep 2

echo ""
echo "================================================================"
echo "步骤3/5: 启动UAV决策系统"
echo "================================================================"

gnome-terminal --title="[2] UAV决策系统" --geometry=100x30+850+0 -- bash -c "
    echo '启动UAV多源决策系统...'
    cd $BASE_DIR
    source setup_env.sh
    ros2 launch uav_decision_arbiter multi_uav.launch.py
    exec bash"

echo "等待系统初始化..."
sleep 8

echo ""
echo "================================================================"
echo "步骤4/5: 启动RL决策平台"
echo "================================================================"

gnome-terminal --title="[3] RL平台" --geometry=100x30+0+500 -- bash -c "
    echo '启动RL决策平台（Pygame可视化）...'
    cd $BASE_DIR
    source setup_env.sh
    python3 rl_platform/rl_platform_node.py
    exec bash"

sleep 5

echo ""
echo "================================================================"
echo "步骤5/5: 启动监控工具"
echo "================================================================"

gnome-terminal --title="[4] 系统监控" --geometry=80x30+850+500 -- bash -c "
    echo '启动系统监控...'
    cd $BASE_DIR
    source setup_env.sh
    python3 examples/monitor.py
    exec bash"

sleep 2

echo ""
echo "================================================================"
echo "✅ 完整系统启动完成！"
echo "================================================================"
echo ""
echo "你现在应该看到："
echo "  🎮 Gazebo窗口 - 3架iris无人机3D仿真"
echo "  🖼️  Pygame窗口 - RL决策2D可视化"
echo "  📊 4个终端窗口 - 各组件日志"
echo ""
echo "操作指南："
echo "  Gazebo: 鼠标拖动旋转视角，滚轮缩放"
echo "  Pygame: SPACE暂停, R重置, Q退出"
echo ""
echo "观察要点："
echo "  ✓ Gazebo中无人机按RL算法飞行（圆形轨迹）"
echo "  ✓ Pygame显示相同的运动轨迹"
echo "  ✓ 监控显示 Control: rl"
echo "  ✓ 编队队形保持一致"
echo ""
echo "测试优先级抢占："
echo "  ros2 topic pub /uav2/central/decision_output std_msgs/msg/String \\"
echo "    'data: \"{\\\"type\\\": \\\"position\\\", \\\"position\\\": {\\\"x\\\": 10, \\\"y\\\": 5, \\\"z\\\": 5, \\\"yaw\\\": 0}}\"' \\"
echo "    --rate 2"
echo ""
echo "  观察uav2在两个界面中都改变轨迹！"
echo ""
echo "================================================================"
echo "停止所有："
echo "  pkill -9 px4; pkill -9 gz; pkill -f mavros; pkill -f ros2"
echo "================================================================"

