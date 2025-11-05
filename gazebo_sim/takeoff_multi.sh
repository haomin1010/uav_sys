#!/bin/bash
# 多机起飞脚本 - 依次起飞所有无人机

NUM_UAVS=${1:-3}
TAKEOFF_ALT=${2:-2.0}
DELAY_BETWEEN=${3:-3}  # 每架之间的延迟（秒）

# Source ROS2环境
source /opt/ros/humble/setup.bash 2>/dev/null
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_BUILTIN_TRANSPORTS=UDPv4

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 用于存储所有 setpoint PID
declare -a SETPOINT_PIDS

echo "========================================"
echo "多机起飞系统"
echo "========================================"
echo "无人机数量: $NUM_UAVS"
echo "起飞高度: ${TAKEOFF_ALT}米"
echo "间隔时间: ${DELAY_BETWEEN}秒"
echo ""

# 清理函数
cleanup() {
    echo ""
    echo "========================================"
    echo "停止所有 setpoint 发送器..."
    echo "========================================"
    
    # 停止记录的PID
    for pid in "${SETPOINT_PIDS[@]}"; do
        if kill -0 $pid 2>/dev/null; then
            kill $pid 2>/dev/null
            sleep 0.5
            # 如果还在运行，强制杀掉
            if kill -0 $pid 2>/dev/null; then
                kill -9 $pid 2>/dev/null
            fi
            echo "  已停止 PID: $pid"
        fi
    done
    
    # 额外保险：杀掉所有send_setpoint进程
    pkill -f "send_setpoint.py" 2>/dev/null
    
    echo "✓ 清理完成"
    exit 0
}

# 捕获 Ctrl+C
trap cleanup SIGINT SIGTERM

# 依次起飞每架无人机
for i in $(seq 1 $NUM_UAVS); do
    UAV_NAME="uav$i"
    
    echo "========================================"
    echo "[$UAV_NAME] 开始起飞流程 ($i/$NUM_UAVS)"
    echo "========================================"
    
    # 1. 检查MAVROS连接
    echo "1. 检查MAVROS连接..."
    timeout 5 bash -c "until ros2 topic echo /$UAV_NAME/state --once 2>/dev/null | grep -q 'connected: true'; do sleep 0.5; done" || {
        echo "   ✗ MAVROS连接失败，跳过此架"
        continue
    }
    echo "   ✓ MAVROS已连接"
    
    # 2. 等待传感器稳定（第一架等久一点）
    if [ $i -eq 1 ]; then
        echo ""
        echo "2. 等待传感器稳定（10秒）..."
        sleep 10
    else
        echo ""
        echo "2. 等待传感器稳定（3秒）..."
        sleep 3
    fi
    
    # 3. 启动 Python setpoint 发送器（使用独立的进程和环境）
    echo ""
    echo "3. 启动 setpoint 发送器..."
    # 使用 setsid 创建新的会话，避免信号传播
    setsid python3 "$SCRIPT_DIR/send_setpoint.py" $UAV_NAME $TAKEOFF_ALT </dev/null > /tmp/setpoint_${UAV_NAME}.log 2>&1 &
    SETPOINT_PID=$!
    SETPOINT_PIDS+=($SETPOINT_PID)
    echo "   PID: $SETPOINT_PID"
    
    # 验证进程是否真的启动了
    sleep 2
    if kill -0 $SETPOINT_PID 2>/dev/null; then
        echo "   ✓ Setpoint发送器进程运行中"
    else
        echo "   ✗ Setpoint发送器启动失败"
        echo "   查看日志: tail /tmp/setpoint_${UAV_NAME}.log"
        continue
    fi
    
    # 验证setpoint是否真的在发送
    sleep 1
    SETPOINT_COUNT=$(timeout 2 ros2 topic hz /$UAV_NAME/setpoint_position/local 2>&1 | grep "average rate" | awk '{print $3}')
    if [ ! -z "$SETPOINT_COUNT" ]; then
        echo "   ✓ Setpoint发送频率: ${SETPOINT_COUNT}Hz"
    else
        echo "   ⚠️  无法检测到setpoint，但继续尝试..."
    fi
    
    sleep 1
    
    # 4. 切换到 OFFBOARD 模式
    echo ""
    echo "4. 切换到 OFFBOARD 模式..."
    ros2 service call /$UAV_NAME/set_mode mavros_msgs/srv/SetMode \
        "{custom_mode: 'OFFBOARD'}" 2>&1 | grep -v "RTPS\|XMLPARSER\|waiting for" | grep "response" -A1
    
    sleep 2
    
    CURRENT_MODE=$(ros2 topic echo /$UAV_NAME/state --once 2>&1 | grep "mode:" | awk '{print $2}')
    if [ "$CURRENT_MODE" = "OFFBOARD" ]; then
        echo "   ✓ OFFBOARD 模式设置成功"
    else
        echo "   ✗ OFFBOARD 模式设置失败，跳过此架"
        kill $SETPOINT_PID 2>/dev/null
        continue
    fi
    
    # 5. 解锁
    echo ""
    echo "5. 解锁无人机..."
    ros2 service call /$UAV_NAME/cmd/arming mavros_msgs/srv/CommandBool \
        "{value: true}" 2>&1 | grep -v "RTPS\|XMLPARSER\|waiting for" | grep "response" -A1
    
    sleep 2
    
    ARMED=$(ros2 topic echo /$UAV_NAME/state --once 2>&1 | grep "armed:" | awk '{print $2}')
    if [ "$ARMED" = "true" ]; then
        echo "   ✓ 解锁成功"
    else
        echo "   ⚠️  正常解锁失败，尝试强制解锁..."
        
        ros2 service call /$UAV_NAME/cmd/command mavros_msgs/srv/CommandLong \
            "{broadcast: false, command: 400, confirmation: 0, param1: 1.0, param2: 21196.0}" \
            2>&1 | grep -v "RTPS\|XMLPARSER\|waiting for" | grep "response" -A1
        
        sleep 2
        
        ARMED=$(ros2 topic echo /$UAV_NAME/state --once 2>&1 | grep "armed:" | awk '{print $2}')
        if [ "$ARMED" = "true" ]; then
            echo "   ✓ 强制解锁成功"
        else
            echo "   ✗ 解锁失败，跳过此架"
            kill $SETPOINT_PID 2>/dev/null
            continue
        fi
    fi
    
    # 6. 监控起飞
    echo ""
    echo "6. 🚁 起飞中..."
    REACHED_ALT=false
    for j in {1..3}; do
        sleep 1
        
        ARMED=$(ros2 topic echo /$UAV_NAME/state --once 2>&1 | grep "armed:" | awk '{print $2}')
        
        ALT_Z=$(ros2 topic echo /$UAV_NAME/local_position/pose --once 2>&1 | \
            grep -A1 "position:" | grep "z:" | awk '{print $2}')
        
        if [ ! -z "$ALT_Z" ]; then
            ALT=$(echo "$ALT_Z" | awk '{printf "%.2f", -$1}')
        else
            ALT="N/A"
        fi
        
        echo "   [$j/10] Armed: $ARMED, 高度: ${ALT}米"
        
        if [ "$ARMED" = "false" ]; then
            echo "   ✗ 失去解锁"
            kill $SETPOINT_PID 2>/dev/null
            break
        fi
        
        if [ "$ALT" != "N/A" ] && (( $(echo "$ALT > $(echo "$TAKEOFF_ALT * 0.8" | bc)" | bc -l) )); then
            echo "   ✓ 达到目标高度！"
            REACHED_ALT=true
            break
        fi
    done
    
    # 7. 停止 setpoint 发送器，让 RL 接管控制
    echo ""
    if [ "$REACHED_ALT" = "true" ]; then
        echo "[$UAV_NAME] 停止 setpoint 发送器，准备接受 RL 控制..."
        kill $SETPOINT_PID 2>/dev/null
        sleep 0.5
        # 从数组中移除这个 PID
        SETPOINT_PIDS=(${SETPOINT_PIDS[@]/$SETPOINT_PID})
        echo "[$UAV_NAME] ✓ 已准备好接受外部控制"
    else
        echo "[$UAV_NAME] ⚠️  未达到目标高度，保持 setpoint 发送"
    fi
    
    echo ""
    echo "[$UAV_NAME] ✓ 起飞流程完成"
    echo ""
    
    # 延迟后起飞下一架
    if [ $i -lt $NUM_UAVS ]; then
        echo "等待 ${DELAY_BETWEEN}秒 后起飞下一架..."
        echo ""
        sleep $DELAY_BETWEEN
    fi
done

echo "========================================"
echo "✓ 所有无人机起飞完成！"
echo "========================================"
echo ""
echo "当前状态:"
for i in $(seq 1 $NUM_UAVS); do
    UAV_NAME="uav$i"
    STATE=$(ros2 topic echo /$UAV_NAME/state --once 2>&1)
    ARMED=$(echo "$STATE" | grep "armed:" | awk '{print $2}')
    MODE=$(echo "$STATE" | grep "mode:" | awk '{print $2}')
    
    ALT_Z=$(ros2 topic echo /$UAV_NAME/local_position/pose --once 2>&1 | \
        grep -A1 "position:" | grep "z:" | awk '{print $2}')
    if [ ! -z "$ALT_Z" ]; then
        ALT=$(echo "$ALT_Z" | awk '{printf "%.2f", -$1}')
    else
        ALT="N/A"
    fi
    
    echo "  [$UAV_NAME] Armed: $ARMED, Mode: $MODE, 高度: ${ALT}米"
done

echo ""
echo "Setpoint 发送器正在运行（保持悬停）:"
for pid in "${SETPOINT_PIDS[@]}"; do
    if kill -0 $pid 2>/dev/null; then
        echo "  PID: $pid"
    fi
done

echo ""
echo "命令:"
echo "  降落所有: for i in {1..3}; do ros2 service call /uav\$i/cmd/land mavros_msgs/srv/CommandTOL '{}' & done"
echo "  停止 setpoint: kill ${SETPOINT_PIDS[@]}"
echo "  按 Ctrl+C 停止所有 setpoint 并退出"
echo ""

# 保持运行
echo "保持脚本运行以维持悬停..."
wait

