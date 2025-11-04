#!/usr/bin/env python3
"""
中央算力示例（AirSim版）
演示如何通过中央算力控制AirSim中的无人机
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time


class CentralPlannerAirSim(Node):
    """中央算力规划器（用于AirSim）"""
    
    def __init__(self):
        super().__init__('central_planner_airsim')
        
        # 发布器：为每架无人机发布决策
        self.pubs = {}
        for uav_id in ['uav1', 'uav2', 'uav3']:
            self.pubs[uav_id] = self.create_publisher(
                String,
                f'/{uav_id}/central/decision_output',
                10
            )
        
        # 订阅编队同步（获取当前位置）
        self.sub_formation = self.create_subscription(
            String,
            '/uav/formation_sync',
            self.on_formation_sync,
            10
        )
        
        # 任务列表（waypoints，NED坐标）
        self.tasks = {
            'uav1': [
                {"x": 0, "y": 0, "z": -5},
                {"x": 10, "y": 0, "z": -5},
                {"x": 10, "y": 10, "z": -5},
                {"x": 0, "y": 10, "z": -5},
            ],
            'uav2': [
                {"x": 0, "y": -5, "z": -5},
                {"x": 15, "y": -5, "z": -5},
                {"x": 15, "y": 5, "z": -5},
            ],
            'uav3': [
                {"x": 0, "y": 5, "z": -5},
                {"x": 5, "y": 15, "z": -5},
                {"x": -5, "y": 15, "z": -5},
            ]
        }
        
        # 当前任务索引
        self.current_task_idx = {uid: 0 for uid in self.tasks.keys()}
        
        # 定时发布（每3秒切换waypoint）
        self.timer = self.create_timer(3.0, self.publish_next_task)
        
        self.get_logger().info('=' * 70)
        self.get_logger().info('中央算力规划器已启动（AirSim版）')
        self.get_logger().info('=' * 70)
        self.get_logger().info('任务模式：顺序访问waypoints')
        self.get_logger().info(f'UAV1: {len(self.tasks["uav1"])}个waypoints')
        self.get_logger().info(f'UAV2: {len(self.tasks["uav2"])}个waypoints')
        self.get_logger().info(f'UAV3: {len(self.tasks["uav3"])}个waypoints')
        self.get_logger().info('=' * 70)
    
    def on_formation_sync(self, msg):
        """接收编队同步信息"""
        try:
            data = json.loads(msg.data)
            self.get_logger().info('收到编队同步，中央算力开始工作')
        except:
            pass
    
    def publish_next_task(self):
        """发布下一个任务"""
        for uav_id, waypoints in self.tasks.items():
            if uav_id not in self.pubs:
                continue
            
            # 获取当前waypoint
            idx = self.current_task_idx[uav_id]
            target = waypoints[idx]
            
            # 构建决策消息
            decision = {
                "type": "position",
                "position": {
                    "x": target["x"],
                    "y": target["y"],
                    "z": target["z"],
                    "yaw": 0.0
                },
                "timestamp": time.time()
            }
            
            # 发布
            msg = String()
            msg.data = json.dumps(decision)
            self.pubs[uav_id].publish(msg)
            
            self.get_logger().info(
                f'中央算力 → {uav_id}: 目标({target["x"]}, {target["y"]}, {target["z"]}) '
                f'[waypoint {idx+1}/{len(waypoints)}]'
            )
            
            # 更新索引（循环）
            self.current_task_idx[uav_id] = (idx + 1) % len(waypoints)


def main(args=None):
    rclpy.init(args=args)
    node = CentralPlannerAirSim()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n中央算力规划器已停止")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

