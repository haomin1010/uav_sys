#!/bin/bash
# 一键启动Gazebo多机仿真 + MAVROS
# 这是最简单的启动方式

echo "========================================"
echo "Gazebo多机仿真 - 一键启动"
echo "========================================"

# 配置
NUM_UAVS=3
PX4_DIR="${PX4_AUTOPILOT_DIR:-$HOME/PX4-Autopilot}"

# 检查PX4
if [ ! -d "$PX4_DIR" ]; then
    echo "❌ PX4-Autopilot未找到"
    echo ""
    echo "请先安装PX4:"
    echo "  git clone https://github.com/PX4/PX4-Autopilot.git --recursive"
    echo "  cd PX4-Autopilot"
    echo "  make px4_sitl_default gazebo"
    echo ""
    echo "或设置环境变量:"
    echo "  export PX4_AUTOPILOT_DIR=/path/to/PX4-Autopilot"
    exit 1
fi

# 清理旧进程
echo "清理旧进程..."
pkill -9 px4 2>/dev/null
pkill -9 gz 2>/dev/null
pkill -f mavros_node 2>/dev/null
sleep 2

# 启动Gazebo + PX4 SITL（使用PX4的脚本）
echo "========================================"
echo "步骤1: 启动Gazebo + PX4 SITL"
echo "========================================"

cd $PX4_DIR

# 使用PX4的multi-vehicle启动脚本
echo "启动 $NUM_UAVS 架无人机..."

# 设置环境变量
export PX4_SIM_SPEED_FACTOR=1
export HEADLESS=0  # 显示GUI

# 启动多机仿真
# PX4提供的多机启动方式
DONT_RUN=1 make px4_sitl_default gazebo

# 手动启动每个实例
for i in $(seq 0 $(($NUM_UAVS - 1))); do
    UAV_ID=$((i + 1))
    
    # 计算位置（编队队形）
    case $i in
        0) PX_X=0; PX_Y=0 ;;
        1) PX_X=0; PX_Y=-5 ;;
        2) PX_X=0; PX_Y=5 ;;
        *) PX_X=$((i * 3)); PX_Y=0 ;;
    esac
    
    echo "启动UAV$UAV_ID at ($PX_X, $PX_Y)..."
    
    # 使用PX4的multi_uav_mavros_sitl.sh（如果存在）
    # 或手动启动
    DONT_RUN=1 PX4_SIM_MODEL=iris_${i} PX4_SIMULATOR=gazebo \
        make px4_sitl_default gazebo___iris_${i} \
        > /tmp/px4_uav${UAV_ID}.log 2>&1 &
    
    sleep 3
done

echo "✓ PX4 SITL实例已启动"
sleep 5

# 启动MAVROS
echo "========================================"
echo "步骤2: 启动MAVROS连接"
echo "========================================"

# Source ROS2
source /opt/ros/humble/setup.zsh 2>/dev/null || source /opt/ros/humble/setup.bash

for i in $(seq 0 $(($NUM_UAVS - 1))); do
    UAV_ID=$((i + 1))
    UAV_NAME="uav$UAV_ID"
    
    FCU_UDP_PORT=$((14540 + i))
    GCS_UDP_PORT=$((14557 + i))
    FCU_URL="udp://:${FCU_UDP_PORT}@127.0.0.1:${GCS_UDP_PORT}"
    
    echo "启动 $UAV_NAME MAVROS (FCU: $FCU_URL)..."
    
    ros2 run mavros mavros_node \
        --ros-args \
        -r __ns:=/$UAV_NAME \
        -p fcu_url:="$FCU_URL" \
        -p system_id:=$UAV_ID \
        -p target_system_id:=$UAV_ID \
        > /tmp/mavros_${UAV_NAME}.log 2>&1 &
    
    sleep 2
done

echo "✓ MAVROS连接已启动"

echo "========================================"
echo "✓ Gazebo仿真完全启动！"
echo "========================================"
echo ""
echo "系统信息:"
echo "  无人机数量: $NUM_UAVS"
echo "  Gazebo世界: 已运行"
echo "  MAVROS命名空间: /uav1, /uav2, /uav3"
echo ""
echo "验证命令:"
echo "  ros2 topic list | grep mavros"
echo "  ros2 topic echo /uav1/mavros/state"
echo "  ros2 topic echo /uav1/mavros/local_position/pose"
echo ""
echo "日志位置:"
echo "  PX4: /tmp/px4_uav*.log"
echo "  MAVROS: /tmp/mavros_uav*.log"
echo ""
echo "停止仿真:"
echo "  pkill -9 px4; pkill -9 gz; pkill -f mavros"
echo ""
echo "========================================"
echo "Gazebo仿真正在运行..."
echo "按Ctrl+C停止"
echo "========================================"

# 保持运行
wait

