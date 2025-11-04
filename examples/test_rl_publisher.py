#!/usr/bin/env python3
"""
测试脚本：模拟RL平台发布决策
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
import math


class TestRLPublisher(Node):
    def __init__(self):
        super().__init__('test_rl_publisher')
        
        self.publisher = self.create_publisher(
            String,
            '/rl/decision_output',
            10
        )
        
        # 定时发布，10Hz
        self.timer = self.create_timer(0.1, self.publish_decision)
        self.counter = 0
        
        self.get_logger().info('测试RL发布器已启动，发布到 /rl/decision_output')
    
    def publish_decision(self):
        """发布模拟的RL决策"""
        self.counter += 1
        
        # 模拟一个圆形轨迹的速度命令
        t = self.counter * 0.1
        vx = 1.0 * math.cos(t * 0.5)
        vy = 1.0 * math.sin(t * 0.5)
        vz = 0.1 * math.sin(t * 0.2)
        yaw_rate = 0.2
        
        # RL输出格式
        decision = {
            "action": [vx, vy, vz, yaw_rate],
            "timestamp": time.time()
        }
        
        msg = String()
        msg.data = json.dumps(decision)
        self.publisher.publish(msg)
        
        if self.counter % 10 == 0:
            self.get_logger().info(
                f'发布RL决策 #{self.counter}: v=({vx:.2f}, {vy:.2f}, {vz:.2f})'
            )


def main(args=None):
    rclpy.init(args=args)
    node = TestRLPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

