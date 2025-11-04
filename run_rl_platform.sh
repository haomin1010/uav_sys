#!/bin/bash
# RL平台快速启动脚本

echo "========================================"
echo "启动RL决策平台"
echo "========================================"

# 进入项目目录
cd /home/lihaomin/project/uav_sys

# 激活环境
echo "激活conda环境..."
eval "$(conda shell.bash hook)"
conda activate uav_sys

# Source ROS2
echo "加载ROS2环境..."
source /opt/ros/humble/setup.zsh 2>/dev/null || source /opt/ros/humble/setup.bash
source install/setup.zsh 2>/dev/null || source install/setup.bash

echo "========================================"
echo "RL平台启动中..."
echo "========================================"
echo ""
echo "可视化界面控制："
echo "  SPACE - 暂停/继续"
echo "  R - 重置环境"
echo "  Q - 退出"
echo ""
echo "========================================"

# 启动RL平台
python3 rl_platform/rl_platform_node.py

