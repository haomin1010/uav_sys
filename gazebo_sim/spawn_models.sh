#!/bin/bash
# Gazebo模型spawn脚本
# 在Gazebo中生成多架无人机模型

echo "========================================"
echo "Gazebo模型Spawn"
echo "========================================"

NUM_UAVS=${1:-3}
PX4_DIR="${PX4_AUTOPILOT_DIR:-$HOME/PX4-Autopilot}"

# 检查PX4目录
if [ ! -d "$PX4_DIR" ]; then
    echo "❌ 未找到PX4目录: $PX4_DIR"
    exit 1
fi

# 模型路径
MODEL_PATH="$PX4_DIR/Tools/sitl_gazebo/models"
MODEL_NAME="iris"

echo "模型路径: $MODEL_PATH"
echo "模型名称: $MODEL_NAME"
echo "生成数量: $NUM_UAVS"
echo "========================================"

# 等待Gazebo启动
echo "等待Gazebo服务器启动..."
sleep 5

# Spawn每架无人机
for i in $(seq 0 $(($NUM_UAVS - 1))); do
    UAV_ID=$((i + 1))
    MODEL_INSTANCE="${MODEL_NAME}_$i"
    
    # 位置配置（NED坐标系）
    case $i in
        0)  # UAV1 - 中心
            X=0.0
            Y=0.0
            ;;
        1)  # UAV2 - 右边
            X=0.0
            Y=-5.0
            ;;
        2)  # UAV3 - 左边
            X=0.0
            Y=5.0
            ;;
        *)  # 其他 - 网格
            ROW=$((i / 3))
            COL=$((i % 3))
            X=$((ROW * 5))
            Y=$(((COL - 1) * 5))
            ;;
    esac
    
    Z=0.1  # 稍微离地
    YAW=0.0
    
    echo "Spawning UAV$UAV_ID ($MODEL_INSTANCE) at ($X, $Y, $Z)..."
    
    # 使用gz model命令spawn
    gz model --spawn-file="${MODEL_PATH}/${MODEL_NAME}/${MODEL_NAME}.sdf" \
        --model-name="$MODEL_INSTANCE" \
        -x $X -y $Y -z $Z -Y $YAW \
        > /tmp/spawn_${MODEL_INSTANCE}.log 2>&1 &
    
    sleep 1
done

echo "========================================"
echo "✓ 所有模型已生成"
echo "========================================"
echo ""
echo "Gazebo中应该可以看到 $NUM_UAVS 架无人机"
echo ""
echo "检查模型:"
echo "  gz model --list"
echo ""

