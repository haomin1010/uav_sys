#!/bin/bash
# UAV多源决策系统 - 环境设置脚本
# 使用方法：source setup_env.sh

echo "========================================"
echo "UAV多源决策系统 - 环境初始化"
echo "========================================"

# 检查是否在conda环境中
if [[ -z "${CONDA_DEFAULT_ENV}" ]]; then
    echo "❌ 未检测到Conda环境，正在激活uav_sys环境..."
    eval "$(conda shell.bash hook)"
    conda activate uav_sys
else
    if [[ "${CONDA_DEFAULT_ENV}" != "uav_sys" ]]; then
        echo "⚠️  当前在 ${CONDA_DEFAULT_ENV} 环境，切换到uav_sys..."
        conda activate uav_sys
    else
        echo "✓ Conda环境uav_sys已激活"
    fi
fi

# 检查libstdc++版本（ROS2兼容性）
if ! strings ${CONDA_PREFIX}/lib/libstdc++.so.6 | grep GLIBCXX_3.4.30 > /dev/null 2>&1; then
    echo "⚠️  检测到libstdc++版本过旧，正在更新..."
    conda install -c conda-forge libstdcxx-ng -y -q
    echo "✓ libstdc++已更新"
fi

# 设置ROS2环境（根据您的ROS2版本调整）
ROS_DISTRO="humble"
if [ -f "/opt/ros/${ROS_DISTRO}/setup.zsh" ]; then
    source /opt/ros/${ROS_DISTRO}/setup.zsh
    echo "✓ ROS2 ${ROS_DISTRO} 环境已加载"
elif [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
    source /opt/ros/${ROS_DISTRO}/setup.bash
    echo "✓ ROS2 ${ROS_DISTRO} 环境已加载"
else
    echo "❌ 未找到ROS2安装，请检查ROS2是否正确安装"
fi

# 设置工作空间
WORKSPACE_DIR="/home/lihaomin/project/uav_sys"
if [ -f "${WORKSPACE_DIR}/install/setup.zsh" ]; then
    source ${WORKSPACE_DIR}/install/setup.zsh
    echo "✓ 工作空间已加载"
elif [ -f "${WORKSPACE_DIR}/install/setup.bash" ]; then
    source ${WORKSPACE_DIR}/install/setup.bash
    echo "✓ 工作空间已加载"
else
    echo "⚠️  工作空间尚未编译，请运行: colcon build"
fi

# 禁用ROS2共享内存传输（避免RTPS_TRANSPORT_SHM警告）
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=""
export RMW_FASTRTPS_USE_QOS_FROM_XML=0
# 使用UDP传输替代共享内存
export FASTRTPS_BUILTIN_TRANSPORTS=UDPv4

echo "========================================"
echo "环境设置完成！"
echo ""
echo "可用命令："
echo "  - 编译工作空间: colcon build --symlink-install"
echo "  - 启动完整系统: ros2 launch uav_decision_arbiter system.launch.py"
echo "  - 查看话题列表: ros2 topic list"
echo "  - 查看节点状态: ros2 node list"
echo "========================================"

