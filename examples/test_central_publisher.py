#!/usr/bin/env python3
"""
测试脚本：模拟中央算力发布决策
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time


class TestCentralPublisher(Node):
    def __init__(self):
        super().__init__('test_central_publisher')
        
        self.publisher = self.create_publisher(
            String,
            '/central/decision_output',
            10
        )
        
        # 定时发布目标位置，每2秒更新一次
        self.timer = self.create_timer(2.0, self.publish_decision)
        self.waypoints = [
            {"x": 0.0, "y": 0.0, "z": 2.0, "yaw": 0.0},
            {"x": 5.0, "y": 0.0, "z": 2.0, "yaw": 0.0},
            {"x": 5.0, "y": 5.0, "z": 2.0, "yaw": 1.57},
            {"x": 0.0, "y": 5.0, "z": 2.0, "yaw": 3.14},
        ]
        self.current_waypoint = 0
        
        self.get_logger().info('测试中央算力发布器已启动，发布到 /central/decision_output')
    
    def publish_decision(self):
        """发布模拟的中央算力决策"""
        waypoint = self.waypoints[self.current_waypoint]
        
        # 中央算力输出格式（位置命令）
        decision = {
            "type": "position",
            "position": waypoint,
            "timestamp": time.time()
        }
        
        msg = String()
        msg.data = json.dumps(decision)
        self.publisher.publish(msg)
        
        self.get_logger().info(
            f'发布中央决策: 目标位置=({waypoint["x"]:.1f}, {waypoint["y"]:.1f}, {waypoint["z"]:.1f})'
        )
        
        # 循环路点
        self.current_waypoint = (self.current_waypoint + 1) % len(self.waypoints)


def main(args=None):
    rclpy.init(args=args)
    node = TestCentralPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

