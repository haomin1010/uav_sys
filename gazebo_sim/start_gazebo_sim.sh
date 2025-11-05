#!/bin/bash
# Gazebo多机仿真 - 正确的启动方式

echo "========================================"
echo "Gazebo多机仿真 - 一键启动"
echo "========================================"

# 配置
NUM_UAVS=3
PX4_DIR="${PX4_AUTOPILOT_DIR:-$HOME/project/PX4-Autopilot}"

# 检查PX4
if [ ! -d "$PX4_DIR" ]; then
    echo "❌ PX4-Autopilot未找到: $PX4_DIR"
    exit 1
fi

# 检查WSL GUI
echo "检查GUI环境..."
if [ -z "$DISPLAY" ]; then
    export DISPLAY=:0
fi
echo "DISPLAY=$DISPLAY"

# 清理旧进程
echo "清理旧进程..."
pkill -9 px4 2>/dev/null
pkill -9 gzclient 2>/dev/null  
pkill -9 gzserver 2>/dev/null
pkill -f mavros_node 2>/dev/null
sleep 2

echo "========================================"
echo "步骤1: 启动Gazebo GUI"
echo "========================================"

cd $PX4_DIR

# 设置Gazebo环境
source Tools/simulation/gazebo-classic/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default

# 确保编译过
DONT_RUN=1 make px4_sitl_default gazebo-classic 2>&1 | grep -v "ninja: no work to do" || true

# 启动Gazebo服务器和客户端（带GUI）
echo "启动Gazebo服务器和客户端（使用灰色地面）..."

# 使用自定义世界文件（灰色地面）
CUSTOM_WORLD="/home/lihaomin/project/uav_sys/gazebo_sim/worlds/multi_uav.world"
if [ -f "$CUSTOM_WORLD" ]; then
    echo "  使用自定义世界: $CUSTOM_WORLD"
    gzserver --verbose "$CUSTOM_WORLD" 2>&1 | tee /tmp/gzserver.log &
else
    echo "  使用默认世界（空白）"
    gzserver --verbose Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/empty.world 2>&1 | tee /tmp/gzserver.log &
fi
sleep 3

gzclient --verbose 2>&1 | tee /tmp/gzclient.log &
sleep 3

# 检查Gazebo是否真的启动了
if ! pgrep -f gzserver > /dev/null; then
    echo "❌ Gazebo服务器启动失败！"
    echo "查看日志: cat /tmp/gzserver.log"
    exit 1
fi

if ! pgrep -f gzclient > /dev/null; then
    echo "⚠️  Gazebo客户端（GUI）启动失败！"
    echo "可能X11未配置。查看日志: cat /tmp/gzclient.log"
    echo ""
    echo "WSL2用户请确保："
    echo "1. Windows 11使用WSLg（自带）"
    echo "2. 或安装X Server（VcXsrv/X410）"
else
    echo "✓ Gazebo GUI已启动"
fi

echo "========================================"
echo "步骤2: 启动PX4 SITL实例"
echo "========================================"

# 启动多个PX4实例
for i in $(seq 0 $(($NUM_UAVS - 1))); do
    UAV_ID=$((i + 1))
    
    # 计算位置
    case $i in
        0) PX_X=0; PX_Y=0 ;;
        1) PX_X=0; PX_Y=-5 ;;
        2) PX_X=0; PX_Y=5 ;;
        *) PX_X=$((i * 5)); PX_Y=0 ;;
    esac
    
    echo "启动UAV$UAV_ID at ($PX_X, $PX_Y)..."
    
    # 创建工作目录（参考PX4官方脚本）
    WORK_DIR="$PX4_DIR/build/px4_sitl_default/rootfs/$i"
    mkdir -p "$WORK_DIR"
    
    # 启动PX4实例（在工作目录中启动）
    (
        cd "$WORK_DIR"
        export PX4_SIM_MODEL=gazebo-classic_iris
        export PX4_SIM_SPEED_FACTOR=1
        
        $PX4_DIR/build/px4_sitl_default/bin/px4 \
            -i $i \
            -d "$PX4_DIR/build/px4_sitl_default/etc" \
            > /tmp/px4_uav${UAV_ID}.log 2>&1 &
    )
    
    PX4_PID=$!
    echo "  PX4 PID: $PX4_PID"
    
    sleep 3
    
    # 生成带有正确MAVLink配置的SDF文件
    echo "  生成无人机模型配置..."
    MAVLINK_TCP_PORT=$((4560 + i))
    MAVLINK_UDP_PORT=$((14560 + i))
    MAVLINK_ID=$((1 + i))
    
    python3 $PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic/scripts/jinja_gen.py \
        $PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris/iris.sdf.jinja \
        $PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic \
        --mavlink_tcp_port $MAVLINK_TCP_PORT \
        --mavlink_udp_port $MAVLINK_UDP_PORT \
        --mavlink_id $MAVLINK_ID \
        --gst_udp_port $((5600 + i)) \
        --video_uri $((5600 + i)) \
        --mavlink_cam_udp_port $((14530 + i)) \
        --output-file /tmp/iris_${i}.sdf \
        > /tmp/jinja_$i.log 2>&1
    
    if [ $? -ne 0 ]; then
        echo "  ⚠️  SDF生成失败，查看: /tmp/jinja_$i.log"
    fi
    
    # 在Gazebo中生成模型
    echo "  在Gazebo中生成无人机..."
    gz model \
        -f /tmp/iris_${i}.sdf \
        -m iris_$i \
        -x $PX_X -y $PX_Y -z 0.83 \
        > /tmp/spawn_$i.log 2>&1
    
    if [ $? -eq 0 ]; then
        echo "  ✓ 模型iris_$i已生成"
    else
        echo "  ⚠️  模型生成失败，查看: /tmp/spawn_$i.log"
    fi
    
    sleep 2
done

echo "✓ PX4 SITL实例已启动"
sleep 3

echo "========================================"
echo "步骤3: 启动MAVROS连接"
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
echo "  Gazebo GUI: $(pgrep -f gzclient > /dev/null && echo '运行中' || echo '未运行')"
echo "  MAVROS命名空间: /uav1, /uav2, /uav3"
echo ""
echo "进程状态:"
echo "  gzserver: $(pgrep -f gzserver || echo '未运行')"
echo "  gzclient: $(pgrep -f gzclient || echo '未运行')" 
echo "  px4: $(pgrep px4 | wc -l) 个实例"
echo ""
echo "验证命令:"
echo "  ros2 topic list | grep mavros"
echo "  ros2 topic echo /uav1/mavros/state"
echo ""
echo "日志位置:"
echo "  Gazebo: /tmp/gzserver.log, /tmp/gzclient.log"
echo "  PX4: /tmp/px4_uav*.log"
echo "  MAVROS: /tmp/mavros_uav*.log"
echo ""
echo "停止仿真:"
echo "  pkill -9 px4; pkill -9 gzserver; pkill -9 gzclient; pkill -f mavros"
echo ""
echo "========================================"

# 保持运行
wait
