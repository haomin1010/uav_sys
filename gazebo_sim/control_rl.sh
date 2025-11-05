#!/bin/bash
# RL决策控制脚本
# 用于启动、停止或切换RL决策

# Source ROS2环境
source /opt/ros/humble/setup.bash 2>/dev/null || source /opt/ros/humble/setup.zsh 2>/dev/null

COMMAND=${1:-"start"}

case $COMMAND in
    start)
        echo "🚀 启动RL决策..."
        ros2 topic pub --once /rl/control_command std_msgs/msg/String \
            "{data: '{\"command\": \"start\"}'}"
        echo "✓ RL决策已启动"
        ;;
    stop)
        echo "⏸️  停止RL决策..."
        ros2 topic pub --once /rl/control_command std_msgs/msg/String \
            "{data: '{\"command\": \"stop\"}'}"
        echo "✓ RL决策已停止"
        ;;
    toggle)
        echo "🔄 切换RL决策状态..."
        ros2 topic pub --once /rl/control_command std_msgs/msg/String \
            "{data: '{\"command\": \"toggle\"}'}"
        echo "✓ RL决策状态已切换"
        ;;
    *)
        echo "用法: $0 {start|stop|toggle}"
        echo ""
        echo "命令说明:"
        echo "  start  - 启动RL决策，开始下达命令"
        echo "  stop   - 停止RL决策，暂停下达命令"
        echo "  toggle - 切换RL决策状态"
        exit 1
        ;;
esac


