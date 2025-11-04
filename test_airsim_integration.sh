#!/bin/bash
# AirSim集成快速测试脚本

echo "╔══════════════════════════════════════════════════════════╗"
echo "║         AirSim集成测试                                    ║"
echo "╚══════════════════════════════════════════════════════════╝"
echo ""

# 检查conda环境
if [[ "$CONDA_DEFAULT_ENV" != "uav_sys" ]]; then
    echo "❌ 请先激活conda环境:"
    echo "   conda activate uav_sys"
    exit 1
fi
echo "✓ Conda环境: $CONDA_DEFAULT_ENV"

# 检查airsim包
echo ""
echo "检查airsim包..."
python3 -c "import airsim" 2>/dev/null
if [ $? -eq 0 ]; then
    echo "✓ airsim包已安装"
else
    echo "❌ airsim包未安装"
    echo ""
    echo "安装方法:"
    echo "  pip install airsim"
    exit 1
fi

# 检查settings.json
echo ""
echo "检查settings.json..."
if [ -f ~/Documents/AirSim/settings.json ]; then
    echo "✓ settings.json已存在"
else
    echo "⚠️  settings.json不存在"
    echo ""
    echo "是否自动复制？(y/n)"
    read -r answer
    if [ "$answer" = "y" ]; then
        mkdir -p ~/Documents/AirSim
        cp airsim_config/settings.json ~/Documents/AirSim/
        echo "✓ 已复制settings.json"
        echo "⚠️  请重启AirSim使配置生效"
    else
        echo "❌ 请手动复制:"
        echo "   cp airsim_config/settings.json ~/Documents/AirSim/"
        exit 1
    fi
fi

# 检查multi_uav.yaml配置
echo ""
echo "检查配置文件..."
if grep -q "use_airsim_api: true" config/multi_uav.yaml; then
    echo "✓ AirSim API已启用"
else
    echo "⚠️  AirSim API未启用"
    echo ""
    echo "请编辑 config/multi_uav.yaml，设置:"
    echo "  use_airsim_api: true"
fi

# 测试AirSim连接
echo ""
echo "════════════════════════════════════════════════════════════"
echo "测试AirSim连接..."
echo "════════════════════════════════════════════════════════════"
echo ""
echo "⚠️  请确保AirSim（虚幻引擎）正在运行！"
echo ""
echo "继续测试？(y/n)"
read -r answer

if [ "$answer" = "y" ]; then
    python3 airsim_config/test_airsim_connection.py
    
    if [ $? -eq 0 ]; then
        echo ""
        echo "╔══════════════════════════════════════════════════════════╗"
        echo "║              ✅ AirSim对接成功！                         ║"
        echo "╚══════════════════════════════════════════════════════════╝"
        echo ""
        echo "下一步:"
        echo "  1. 启动系统:"
        echo "     ros2 launch uav_decision_arbiter multi_uav.launch.py"
        echo ""
        echo "  2. 启动中央算力:"
        echo "     python3 examples/central_planner_airsim.py"
        echo ""
        echo "  3. 观察AirSim中的无人机飞行"
        echo ""
        echo "详细文档:"
        echo "  - airsim_config/README.md (完整对接文档)"
        echo "  - AIRSIM_QUICKSTART.md (快速开始)"
        echo "  - AIRSIM_CONNECTION_DIAGRAM.md (数据流图解)"
    else
        echo ""
        echo "╔══════════════════════════════════════════════════════════╗"
        echo "║              ❌ 连接失败                                 ║"
        echo "╚══════════════════════════════════════════════════════════╝"
        echo ""
        echo "故障排除:"
        echo "  1. 确保AirSim正在运行"
        echo "  2. 检查IP地址（默认127.0.0.1）"
        echo "  3. 查看日志错误信息"
        echo ""
        echo "详细文档: airsim_config/README.md"
    fi
else
    echo ""
    echo "测试已取消"
    echo "请先启动AirSim，然后运行:"
    echo "  python3 airsim_config/test_airsim_connection.py"
fi

echo ""

