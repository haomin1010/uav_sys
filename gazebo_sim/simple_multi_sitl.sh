#!/bin/bash
# 简化版多机SITL启动（推荐使用）
# 使用PX4官方的multi-vehicle脚本

echo "========================================"
echo "PX4多机SITL启动（简化版）"
echo "========================================"

PX4_DIR="${PX4_AUTOPILOT_DIR:-$HOME/PX4-Autopilot}"

if [ ! -d "$PX4_DIR" ]; then
    echo "❌ 未找到PX4-Autopilot"
    echo "请安装: git clone https://github.com/PX4/PX4-Autopilot.git"
    exit 1
fi

cd $PX4_DIR

# 清理
pkill -9 px4 2>/dev/null
pkill -9 gz 2>/dev/null
sleep 1

echo "启动3架iris无人机..."
echo ""

# 方法1: 使用PX4的Tools/gazebo_multi_vehicle.sh
if [ -f "Tools/gazebo_multi_vehicle.sh" ]; then
    echo "使用PX4官方多机脚本..."
    Tools/gazebo_multi_vehicle.sh -n 3 -m iris -w empty
else
    # 方法2: 手动启动
    echo "手动启动多机SITL..."
    
    # 启动第一架（会自动打开Gazebo）
    DONT_RUN=1 make px4_sitl gazebo_iris__empty &
    sleep 5
    
    # 启动后续实例
    for i in 1 2; do
        cd $PX4_DIR
        source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
        
        # 创建实例目录
        INSTANCE_DIR="/tmp/px4_instance_$i"
        mkdir -p $INSTANCE_DIR
        
        # 启动PX4实例
        ./build/px4_sitl_default/bin/px4 \
            -i $i \
            -d "$PX4_DIR/build/px4_sitl_default/etc" \
            > /tmp/px4_$i.log 2>&1 &
        
        echo "✓ PX4实例$i已启动"
        sleep 3
    done
fi

echo "========================================"
echo "✓ PX4 SITL启动完成"
echo "========================================"
echo ""
echo "下一步:"
echo "  1. 检查Gazebo窗口（应该显示3架无人机）"
echo "  2. 在新终端运行: ./mavros_launch.sh"
echo "  3. 然后启动主系统"
echo ""
echo "按Ctrl+C停止..."
wait

