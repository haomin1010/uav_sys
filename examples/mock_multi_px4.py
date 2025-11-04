#!/usr/bin/env python3
"""
测试脚本：模拟多架PX4的位置发布
用于测试编队位置同步功能，无需真实PX4/MAVROS
"""
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped
import time


class MockPX4Publisher(Node):
    """模拟单架PX4的位置发布"""
    
    def __init__(self, uav_id: str, initial_pos: dict):
        super().__init__(f'mock_px4_{uav_id}')
        
        self.uav_id = uav_id
        self.position = initial_pos
        
        # 发布到MAVROS话题
        self.pub = self.create_publisher(
            PoseStamped,
            f'/{uav_id}/mavros/local_position/pose',
            10
        )
        
        # 10Hz发布
        self.timer = self.create_timer(0.1, self.publish_position)
        
        self.get_logger().info(
            f'{uav_id} Mock PX4启动 - 位置(ENU): '
            f'({initial_pos["x"]:.2f}, {initial_pos["y"]:.2f}, {initial_pos["z"]:.2f})'
        )
    
    def publish_position(self):
        """发布位置"""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        
        # 位置（ENU坐标系）
        msg.pose.position.x = self.position['x']
        msg.pose.position.y = self.position['y']
        msg.pose.position.z = self.position['z']
        
        # 姿态（无旋转）
        msg.pose.orientation.w = 1.0
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    
    print("=" * 70)
    print("模拟多机PX4位置发布器")
    print("=" * 70)
    print("模拟场景：3架无人机在不同起飞点，各自的PX4本地坐标都是(0,0,0)")
    print("")
    print("无人机配置(ENU坐标)：")
    print("  UAV1: (0.0, 0.0, 0.0) - 起飞点A")
    print("  UAV2: (0.0, 0.0, 0.0) - 起飞点B（实际距离A约10米）")
    print("  UAV3: (0.0, 0.0, 0.0) - 起飞点C（实际距离A约10米）")
    print("")
    print("发布话题：")
    print("  - /uav1/mavros/local_position/pose")
    print("  - /uav2/mavros/local_position/pose")
    print("  - /uav3/mavros/local_position/pose")
    print("=" * 70)
    print("")
    
    # 创建3架模拟PX4
    # 注意：这里模拟的是各自的局部坐标，都在各自起飞点(0,0,0)
    nodes = [
        MockPX4Publisher('uav1', {'x': 0.0, 'y': 0.0, 'z': 0.0}),
        MockPX4Publisher('uav2', {'x': 0.0, 'y': 0.0, 'z': 0.0}),
        MockPX4Publisher('uav3', {'x': 0.0, 'y': 0.0, 'z': 0.0})
    ]
    
    try:
        executor = MultiThreadedExecutor()
        for node in nodes:
            executor.add_node(node)
        
        print("开始发布位置数据...")
        print("提示：等待formation_sync节点的同步消息")
        print("=" * 70)
        
        executor.spin()
    except KeyboardInterrupt:
        print("\n停止模拟PX4发布器")
    finally:
        for node in nodes:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

