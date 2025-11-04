#!/usr/bin/env python3
"""
多机RL控制示例
演示如何为多架无人机发布决策命令
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
import math


class MultiUAVRLController(Node):
    """多机RL控制器示例"""
    
    def __init__(self):
        super().__init__('multi_uav_rl_controller')
        
        # 创建每架无人机的发布器
        self.uav_ids = ['uav1', 'uav2', 'uav3']
        self.pubs = {}
        
        for uav_id in self.uav_ids:
            self.pubs[uav_id] = self.create_publisher(
                String,
                f'/{uav_id}/rl/decision_output',
                10
            )
        
        # 订阅编队同步（获取初始位置）
        self.sub_formation = self.create_subscription(
            String,
            '/uav/formation_sync',
            self.on_formation_sync,
            10
        )
        
        # 定时发布控制命令（10Hz）
        self.timer = self.create_timer(0.1, self.publish_controls)
        self.counter = 0
        self.formation_received = False
        
        self.get_logger().info('=' * 70)
        self.get_logger().info('多机RL控制器已启动')
        self.get_logger().info(f'控制无人机: {self.uav_ids}')
        self.get_logger().info('等待接收编队初始位置...')
        self.get_logger().info('=' * 70)
    
    def on_formation_sync(self, msg):
        """接收编队同步信息"""
        if self.formation_received:
            return
        
        try:
            data = json.loads(msg.data)
            
            self.get_logger().info('=' * 70)
            self.get_logger().info('✓ 收到编队同步信息！')
            self.get_logger().info('=' * 70)
            
            for uav_id, info in data['uavs'].items():
                pos_enu = info['px4_position_enu']
                offset = info['formation_offset']
                
                self.get_logger().info(
                    f'{uav_id}: '
                    f'PX4_ENU=({pos_enu["x"]:.2f}, {pos_enu["y"]:.2f}, {pos_enu["z"]:.2f}), '
                    f'编队偏移(NED)=({offset["x"]:.2f}, {offset["y"]:.2f}, {offset["z"]:.2f})'
                )
            
            self.get_logger().info('=' * 70)
            self.get_logger().info('开始发布RL控制命令...')
            self.get_logger().info('=' * 70)
            
            self.formation_received = True
            
        except Exception as e:
            self.get_logger().error(f'处理编队同步失败: {e}')
    
    def publish_controls(self):
        """发布控制命令"""
        if not self.formation_received:
            return
        
        self.counter += 1
        t = self.counter * 0.1
        
        # 示例1：所有无人机执行相同的圆形运动（保持队形）
        vx = 1.0 * math.cos(t * 0.5)
        vy = 1.0 * math.sin(t * 0.5)
        vz = 0.1 * math.sin(t * 0.2)
        yaw_rate = 0.2
        
        for uav_id in self.pubs.keys():
            decision = {
                "action": [vx, vy, vz, yaw_rate],
                "timestamp": time.time()
            }
            
            msg = String()
            msg.data = json.dumps(decision)
            self.pubs[uav_id].publish(msg)
        
        if self.counter % 50 == 0:
            self.get_logger().info(
                f'发布编队控制 #{self.counter}: '
                f'v=({vx:.2f}, {vy:.2f}, {vz:.2f}), yaw_rate={yaw_rate:.2f}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = MultiUAVRLController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n多机RL控制器已停止")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

