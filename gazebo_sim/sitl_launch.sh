#!/bin/bash
# PX4 SITL 多机启动脚本
# 用于启动多个PX4实例进行仿真

echo "========================================"
echo "PX4 SITL 多机仿真启动"
echo "========================================"

# 检查PX4固件路径
PX4_DIR="${PX4_AUTOPILOT_DIR:-$HOME/PX4-Autopilot}"

if [ ! -d "$PX4_DIR" ]; then
    echo "❌ 未找到PX4-Autopilot目录"
    echo "请设置环境变量: export PX4_AUTOPILOT_DIR=/path/to/PX4-Autopilot"
    echo "或将PX4-Autopilot克隆到: $HOME/PX4-Autopilot"
    exit 1
fi

echo "✓ PX4目录: $PX4_DIR"

# 参数配置
NUM_UAVS=${1:-3}  # 默认3架无人机
WORLD=${2:-empty}  # Gazebo世界

echo "启动配置:"
echo "  无人机数量: $NUM_UAVS"
echo "  Gazebo世界: $WORLD"
echo "========================================"

# 清理旧的进程
echo "清理旧进程..."
pkill -9 px4 2>/dev/null
pkill -9 gzserver 2>/dev/null
pkill -9 gzclient 2>/dev/null
sleep 1

# 设置Gazebo环境
cd $PX4_DIR
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo

# 创建临时工作目录
WORK_DIR="/tmp/px4_sitl_multi"
rm -rf $WORK_DIR
mkdir -p $WORK_DIR

echo "启动Gazebo服务器..."
# 启动Gazebo（无头模式可选）
if [ "$3" == "headless" ]; then
    gzserver --verbose $(pwd)/Tools/sitl_gazebo/worlds/${WORLD}.world &
    GAZEBO_PID=$!
    echo "✓ Gazebo服务器已启动（无头模式）PID=$GAZEBO_PID"
else
    gazebo --verbose $(pwd)/Tools/sitl_gazebo/worlds/${WORLD}.world &
    GAZEBO_PID=$!
    echo "✓ Gazebo已启动 PID=$GAZEBO_PID"
fi

sleep 5

echo "========================================"
echo "启动PX4 SITL实例..."
echo "========================================"

# 为每架无人机启动SITL
for i in $(seq 0 $(($NUM_UAVS - 1))); do
    UAV_ID=$((i + 1))
    MAV_SYS_ID=$UAV_ID
    
    # 端口配置
    # PX4 simulator UDP端口
    SIM_UDP_PORT=$((14560 + i))
    
    # MAVLINK端口（用于MAVROS连接）
    MAVLINK_UDP_PORT=$((14540 + i))
    MAVLINK_TCP_PORT=$((4560 + i))
    
    # 工作目录
    INSTANCE_DIR="$WORK_DIR/instance_$i"
    mkdir -p $INSTANCE_DIR
    cd $INSTANCE_DIR
    
    # 生成SITL配置
    cat > rcS <<EOF
#!/bin/sh
uorb start
param select $INSTANCE_DIR/parameters
param load
dataman start
param set MAV_SYS_ID $MAV_SYS_ID
param set MAV_TYPE 2
param set SYS_AUTOSTART 4001
param set SIM_UDP_PORT $SIM_UDP_PORT

# 位置偏移（Gazebo中的spawn位置）
# X(North), Y(East), Z(Down), Yaw
EOF

    # 根据ID设置不同的spawn位置（编队队形）
    case $i in
        0)  # UAV1 - 中心
            echo "param set PX4_SIM_INIT_LOC_X 0.0" >> rcS
            echo "param set PX4_SIM_INIT_LOC_Y 0.0" >> rcS
            ;;
        1)  # UAV2 - 右边5米
            echo "param set PX4_SIM_INIT_LOC_X 0.0" >> rcS
            echo "param set PX4_SIM_INIT_LOC_Y -5.0" >> rcS
            ;;
        2)  # UAV3 - 左边5米
            echo "param set PX4_SIM_INIT_LOC_X 0.0" >> rcS
            echo "param set PX4_SIM_INIT_LOC_Y 5.0" >> rcS
            ;;
        *)  # 其他 - 网格排列
            ROW=$((i / 3))
            COL=$((i % 3))
            echo "param set PX4_SIM_INIT_LOC_X $((ROW * 5))" >> rcS
            echo "param set PX4_SIM_INIT_LOC_Y $(((COL - 1) * 5))" >> rcS
            ;;
    esac
    
    cat >> rcS <<EOF

replay tryapplyparams
simulator start -c \$SIMULATOR_UDP_PORT
tone_alarm start
gyrosim start
accelsim start
barosim start
gpssim start
ekf2 start
mc_pos_control start
mc_att_control start
land_detector start multicopter
navigator start
mavlink start -x -u $MAVLINK_UDP_PORT -r 4000000
mavlink start -x -u $MAVLINK_TCP_PORT -r 4000000 -m onboard -o 14580
mavlink stream -r 50 -s POSITION_TARGET_LOCAL_NED -u $MAVLINK_UDP_PORT
mavlink stream -r 50 -s LOCAL_POSITION_NED -u $MAVLINK_UDP_PORT
mavlink stream -r 50 -s GLOBAL_POSITION_INT -u $MAVLINK_UDP_PORT
mavlink stream -r 50 -s ATTITUDE -u $MAVLINK_UDP_PORT
mavlink boot_complete
EOF

    chmod +x rcS
    
    # 启动PX4实例
    echo "启动 UAV$UAV_ID (MAV_SYS_ID=$MAV_SYS_ID, UDP=$MAVLINK_UDP_PORT)..."
    
    $PX4_DIR/build/px4_sitl_default/bin/px4 \
        -i $i \
        -s $INSTANCE_DIR/rcS \
        -d "$PX4_DIR/build/px4_sitl_default/etc" \
        > $INSTANCE_DIR/px4_${i}.log 2>&1 &
    
    PX4_PID=$!
    echo "  PID: $PX4_PID"
    echo "  UDP端口: $MAVLINK_UDP_PORT"
    echo "  日志: $INSTANCE_DIR/px4_${i}.log"
    
    sleep 2
done

echo "========================================"
echo "✓ 所有PX4 SITL实例已启动"
echo "========================================"
echo ""
echo "端口分配:"
for i in $(seq 0 $(($NUM_UAVS - 1))); do
    UAV_ID=$((i + 1))
    MAVLINK_UDP_PORT=$((14540 + i))
    echo "  UAV$UAV_ID: UDP $MAVLINK_UDP_PORT (MAVROS连接: udp://:${MAVLINK_UDP_PORT}@127.0.0.1:$((14557 + i)))"
done

echo ""
echo "Gazebo进程: $GAZEBO_PID"
echo ""
echo "工作目录: $WORK_DIR"
echo ""
echo "========================================"
echo "使用说明:"
echo "  - 查看Gazebo: 窗口应该已打开"
echo "  - 停止所有: pkill -9 px4; pkill -9 gz"
echo "  - 查看日志: tail -f $WORK_DIR/instance_*/px4_*.log"
echo "========================================"

# 保持脚本运行
echo ""
echo "按Ctrl+C停止所有SITL实例..."
wait

