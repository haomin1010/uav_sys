#!/usr/bin/env python3
"""
轨迹同步器节点
功能：接收仲裁后的权威命令，转发给所有适配器，确保各平台轨迹同步
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
from .command_msg import CommandMsg


class SynchronizerNode(Node):
    """轨迹同步器节点"""
    
    def __init__(self):
        super().__init__('synchronizer_node')
        
        # 参数配置
        self.declare_parameter('publish_rate', 20.0)  # Hz
        
        self.publish_rate = self.get_parameter('publish_rate').value
        
        # 当前权威命令
        self.current_cmd = None
        self.last_cmd_time = 0.0
        
        # 订阅权威命令
        self.sub_authoritative = self.create_subscription(
            String,
            '/uav/authoritative_cmd',
            self.on_authoritative_cmd,
            10
        )
        
        # 发布给各适配器的命令（也可以订阅authoritative_cmd）
        # 这里提供一个统一的同步命令话题
        self.pub_sync_cmd = self.create_publisher(
            String,
            '/uav/sync/cmd',
            10
        )
        
        # 订阅各平台的状态反馈
        self.sub_rl_state = self.create_subscription(
            String,
            '/uav/rl/state',
            lambda msg: self.on_state_feedback(msg, 'rl'),
            10
        )
        
        self.sub_airsim_state = self.create_subscription(
            String,
            '/uav/airsim/state',
            lambda msg: self.on_state_feedback(msg, 'airsim'),
            10
        )
        
        self.sub_px4_state = self.create_subscription(
            String,
            '/uav/px4/state',
            lambda msg: self.on_state_feedback(msg, 'px4'),
            10
        )
        
        # 发布同步状态
        self.pub_sync_status = self.create_publisher(
            String,
            '/uav/sync/status',
            10
        )
        
        # 各平台的状态
        self.platform_states = {
            'rl': {'last_update': 0.0, 'position': None, 'synced': False},
            'airsim': {'last_update': 0.0, 'position': None, 'synced': False},
            'px4': {'last_update': 0.0, 'position': None, 'synced': False}
        }
        
        # 定时器：定期检查同步状态
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)
        
        self.get_logger().info('轨迹同步器节点已启动')
    
    def now(self) -> float:
        """获取当前时间戳"""
        return time.time()
    
    def on_authoritative_cmd(self, msg: String):
        """接收权威命令"""
        try:
            cmd = CommandMsg.from_json(msg.data)
            self.current_cmd = cmd
            self.last_cmd_time = self.now()
            
            self.get_logger().info(
                f'收到权威命令: source={cmd.header.source_id}, '
                f'mode={cmd.meta.mode}, seq={cmd.header.seq}'
            )
            
            # 立即转发给所有适配器
            self.forward_command(cmd)
            
        except Exception as e:
            self.get_logger().error(f'处理权威命令失败: {e}')
    
    def forward_command(self, cmd: CommandMsg):
        """转发命令到所有适配器"""
        msg = String()
        msg.data = cmd.to_json()
        self.pub_sync_cmd.publish(msg)
        
        self.get_logger().debug(
            f'转发同步命令: mode={cmd.meta.mode}, source={cmd.header.source_id}'
        )
    
    def on_state_feedback(self, msg: String, platform: str):
        """接收平台状态反馈"""
        try:
            state = json.loads(msg.data)
            current_time = self.now()
            
            self.platform_states[platform]['last_update'] = current_time
            self.platform_states[platform]['position'] = state.get('position')
            
            self.get_logger().debug(
                f'收到 {platform} 状态: pos={state.get("position")}'
            )
            
        except Exception as e:
            self.get_logger().error(f'解析 {platform} 状态失败: {e}')
    
    def check_synchronization(self) -> dict:
        """检查各平台同步状态"""
        current_time = self.now()
        sync_status = {
            'timestamp': current_time,
            'overall_synced': True,
            'platforms': {}
        }
        
        # 检查各平台状态是否及时
        state_timeout = 2.0  # 2秒超时
        
        for platform, state in self.platform_states.items():
            age = current_time - state['last_update']
            is_alive = age < state_timeout
            
            sync_status['platforms'][platform] = {
                'alive': is_alive,
                'state_age': age,
                'has_position': state['position'] is not None
            }
            
            if not is_alive:
                sync_status['overall_synced'] = False
        
        # TODO: 可以添加位置偏差检查，判断各平台位置是否一致
        
        return sync_status
    
    def timer_callback(self):
        """定时回调：检查并发布同步状态"""
        sync_status = self.check_synchronization()
        
        # 发布同步状态
        msg = String()
        msg.data = json.dumps(sync_status, indent=2)
        self.pub_sync_status.publish(msg)
        
        # 如果有些平台失去同步，发出警告
        if not sync_status['overall_synced']:
            for platform, status in sync_status['platforms'].items():
                if not status['alive']:
                    self.get_logger().warning(
                        f'平台 {platform} 失去同步 (状态已过时 {status["state_age"]:.2f}s)'
                    )


def main(args=None):
    rclpy.init(args=args)
    node = SynchronizerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

