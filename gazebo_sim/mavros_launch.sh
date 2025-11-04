#!/bin/bash
# MAVROS 多机启动脚本
# 为每架PX4 SITL启动独立的MAVROS实例

echo "========================================"
echo "MAVROS 多机启动"
echo "========================================"

# Source ROS2环境
source /opt/ros/humble/setup.zsh 2>/dev/null || source /opt/ros/humble/setup.bash

# 参数
NUM_UAVS=${1:-3}

echo "启动 $NUM_UAVS 个MAVROS实例..."
echo "========================================"

# 清理旧进程
pkill -f mavros_node 2>/dev/null
sleep 1

# 为每架无人机启动MAVROS
for i in $(seq 0 $(($NUM_UAVS - 1))); do
    UAV_ID=$((i + 1))
    UAV_NAME="uav$UAV_ID"
    
    # 端口配置（与SITL对应）
    FCU_UDP_PORT=$((14540 + i))
    GCS_UDP_PORT=$((14557 + i))
    
    # FCU URL
    FCU_URL="udp://:${FCU_UDP_PORT}@127.0.0.1:${GCS_UDP_PORT}"
    
    echo "启动 $UAV_NAME MAVROS..."
    echo "  命名空间: /$UAV_NAME"
    echo "  系统ID: $UAV_ID"
    echo "  FCU URL: $FCU_URL"
    
    # 启动MAVROS（使用ROS2命名空间）
    ros2 run mavros mavros_node \
        --ros-args \
        -r __ns:=/$UAV_NAME \
        -p fcu_url:="$FCU_URL" \
        -p system_id:=$UAV_ID \
        -p target_system_id:=$UAV_ID \
        -p target_component_id:=1 \
        -p gcs_url:="" \
        > /tmp/mavros_${UAV_NAME}.log 2>&1 &
    
    MAVROS_PID=$!
    echo "  PID: $MAVROS_PID"
    echo "  日志: /tmp/mavros_${UAV_NAME}.log"
    echo ""
    
    sleep 2
done

echo "========================================"
echo "✓ 所有MAVROS实例已启动"
echo "========================================"
echo ""
echo "话题检查:"
for i in $(seq 1 $NUM_UAVS); do
    echo "  UAV$i: /uav$i/mavros/..."
done
echo ""
echo "验证命令:"
echo "  ros2 topic list | grep mavros"
echo "  ros2 topic echo /uav1/mavros/state"
echo ""
echo "停止所有MAVROS:"
echo "  pkill -f mavros_node"
echo "========================================"

# 等待用户中断
echo ""
echo "按Ctrl+C停止所有MAVROS实例..."
wait

