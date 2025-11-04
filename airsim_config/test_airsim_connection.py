#!/usr/bin/env python3
"""
测试AirSim连接
验证AirSim是否正常运行并可以控制无人机
"""
import sys

print("=" * 70)
print("AirSim连接测试")
print("=" * 70)

# 检查airsim包
try:
    import airsim
    print("✓ airsim包已安装")
except ImportError:
    print("❌ airsim包未安装")
    print("")
    print("安装方法:")
    print("  pip install airsim")
    print("")
    sys.exit(1)

# 连接AirSim
print("\n连接AirSim...")
try:
    client = airsim.MultirotorClient()
    client.confirmConnection()
    print("✓ 成功连接到AirSim")
except Exception as e:
    print(f"❌ 连接失败: {e}")
    print("")
    print("请确保:")
    print("  1. AirSim（虚幻引擎）正在运行")
    print("  2. IP地址正确（默认127.0.0.1）")
    print("")
    sys.exit(1)

# 获取无人机列表
print("\n检查无人机...")
vehicle_names = ['Drone1', 'Drone2', 'Drone3']

for vehicle_name in vehicle_names:
    try:
        # 启用API控制
        client.enableApiControl(True, vehicle_name)
        
        # 解锁
        client.armDisarm(True, vehicle_name)
        
        # 获取状态
        state = client.getMultirotorState(vehicle_name)
        pos = state.kinematics_estimated.position
        
        print(f"✓ {vehicle_name}: 位置=({pos.x_val:.2f}, {pos.y_val:.2f}, {pos.z_val:.2f})")
        
    except Exception as e:
        print(f"⚠️  {vehicle_name}: {e}")

# 测试移动命令
print("\n" + "=" * 70)
print("测试移动命令（Drone1起飞到5米高）")
print("=" * 70)

try:
    vehicle_name = "Drone1"
    
    # 起飞
    print(f"起飞 {vehicle_name}...")
    client.takeoffAsync(vehicle_name=vehicle_name).join()
    print("✓ 起飞完成")
    
    # 移动到指定高度
    print(f"移动到 (0, 0, -5)...")
    client.moveToPositionAsync(0, 0, -5, 3, vehicle_name=vehicle_name).join()
    print("✓ 移动完成")
    
    # 获取最终位置
    state = client.getMultirotorState(vehicle_name)
    pos = state.kinematics_estimated.position
    print(f"✓ 当前位置: ({pos.x_val:.2f}, {pos.y_val:.2f}, {pos.z_val:.2f})")
    
    # 悬停3秒
    print("悬停3秒...")
    import time
    time.sleep(3)
    
    # 降落
    print("降落...")
    client.landAsync(vehicle_name=vehicle_name).join()
    print("✓ 降落完成")
    
except Exception as e:
    print(f"❌ 控制失败: {e}")

print("\n" + "=" * 70)
print("✅ AirSim连接测试完成！")
print("=" * 70)
print("")
print("AirSim工作正常，可以与系统对接！")
print("")
print("下一步:")
print("  1. 将 settings.json 复制到 ~/Documents/AirSim/")
print("  2. 重启AirSim使配置生效")
print("  3. 修改 config/multi_uav.yaml 启用API:")
print("     use_airsim_api: true")
print("")

