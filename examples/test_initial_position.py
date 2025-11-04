#!/usr/bin/env python3
"""
测试脚本：模拟PX4发布初始位置
用于测试初始位置同步功能，不需要真实的PX4/MAVROS
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time


class TestInitialPositionPublisher(Node):
    def __init__(self):
        super().__init__('test_initial_position_publisher')
        
        self.publisher = self.create_publisher(
            String,
            '/uav/initial_position',
            10
        )
        
        # 等待2秒让其他节点启动
        self.timer = self.create_timer(2.0, self.publish_once)
        self.published = False
        
        self.get_logger().info('测试初始位置发布器已启动，2秒后发布...')
    
    def publish_once(self):
        """发布一次初始位置"""
        if self.published:
            return
        
        # 模拟一个初始位置（例如：机场起飞点）
        initial_position = {
            'timestamp': time.time(),
            'source': 'test_px4',
            'position': {
                'x': 0.0,    # 原点
                'y': 0.0,
                'z': 0.0     # 地面高度
            },
            'orientation': {
                'w': 1.0,    # 无旋转
                'x': 0.0,
                'y': 0.0,
                'z': 0.0
            }
        }
        
        msg = String()
        msg.data = json.dumps(initial_position)
        self.publisher.publish(msg)
        
        self.published = True
        self.get_logger().info('=' * 60)
        self.get_logger().info('✓ 已发布初始位置到 /uav/initial_position')
        self.get_logger().info(f'  位置: ({initial_position["position"]["x"]:.2f}, '
                             f'{initial_position["position"]["y"]:.2f}, '
                             f'{initial_position["position"]["z"]:.2f})')
        self.get_logger().info('=' * 60)
        self.get_logger().info('观察：')
        self.get_logger().info('  - AirSim Adapter 应该设置仿真无人机位置（如果启用）')
        self.get_logger().info('  - RL Adapter 应该接收到初始位置信息')
        self.get_logger().info('=' * 60)
        
        # 取消定时器
        self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = TestInitialPositionPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

