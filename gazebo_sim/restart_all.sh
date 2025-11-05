#!/bin/bash
# 完全重启 Gazebo 和多机系统

echo "========================================"
echo "重启所有系统"
echo "========================================"

echo "1. 停止所有进程..."
pkill -9 px4 2>/dev/null
pkill -9 gzserver 2>/dev/null
pkill -9 gzclient 2>/dev/null
pkill -f mavros_node 2>/dev/null
pkill -f "multi_uav.launch" 2>/dev/null
pkill -f arbiter 2>/dev/null
pkill -f synchronizer 2>/dev/null
pkill -f formation_sync 2>/dev/null
pkill -f adapter 2>/dev/null

sleep 3

echo "✓ 所有进程已停止"
echo ""

# 清理共享内存文件
echo "2. 清理共享内存..."
rm -rf /dev/shm/fastrtps_* 2>/dev/null || true
rm -rf /tmp/.fastrtps_* 2>/dev/null || true
echo "✓ 共享内存已清理"
echo ""

echo "========================================"
echo "✓ 清理完成！"
echo "========================================"
echo ""
echo "现在可以重新启动系统："
echo "  1. 启动 Gazebo: ./gazebo_sim/start_gazebo_sim.sh"
echo "  2. 启动决策系统: ros2 launch uav_decision_arbiter multi_uav.launch.py"
echo ""


