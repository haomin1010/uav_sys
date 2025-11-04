#!/usr/bin/env python3
"""
RL决策平台适配器
功能：
1. 接收RL平台的决策输出，转换为统一CommandMsg格式发送给仲裁器
2. 接收同步命令，转发给RL平台用于可视化展示
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
from .command_msg import (
    CommandMsg, Header, Meta, Body, Velocity, Setpoint, 
    SourceID, PRIORITY_MAP, CommandMode
)


class RLAdapter(Node):
    """RL平台适配器节点"""
    
    def __init__(self):
        super().__init__('rl_adapter')
        
        # 参数配置
        self.declare_parameter('command_timeout', 0.5)  # 命令有效期（秒）
        self.declare_parameter('rl_input_format', 'velocity')  # RL输出格式：velocity/position
        self.declare_parameter('sync_initial_position', True)  # 是否同步初始位置
        
        self.command_timeout = self.get_parameter('command_timeout').value
        self.rl_input_format = self.get_parameter('rl_input_format').value
        self.sync_initial_position = self.get_parameter('sync_initial_position').value
        
        self.seq = 0  # 命令序列号
        
        # 订阅RL平台的决策输出
        # RL平台应该发布到这个话题，格式可以是自定义的
        self.sub_rl_output = self.create_subscription(
            String,
            '/rl/decision_output',
            self.on_rl_decision,
            10
        )
        
        # 发布转换后的统一命令给仲裁器
        self.pub_command = self.create_publisher(
            String,
            '/uav/source/rl/cmd',
            10
        )
        
        # 订阅同步命令（用于RL平台的可视化）
        self.sub_sync_cmd = self.create_subscription(
            String,
            '/uav/sync/cmd',
            self.on_sync_cmd,
            10
        )
        
        # 发布给RL平台的可视化命令
        self.pub_rl_viz = self.create_publisher(
            String,
            '/rl/visualization_cmd',
            10
        )
        
        # 发布RL平台状态
        self.pub_state = self.create_publisher(
            String,
            '/uav/rl/state',
            10
        )
        
        # 订阅RL平台的状态反馈（如果RL平台提供）
        self.sub_rl_state = self.create_subscription(
            String,
            '/rl/state_feedback',
            self.on_rl_state_feedback,
            10
        )
        
        # 订阅初始位置
        if self.sync_initial_position:
            self.sub_initial_position = self.create_subscription(
                String,
                '/uav/initial_position',
                self.on_initial_position,
                10
            )
        
        # 定时发布心跳状态
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz
        
        self.last_rl_state = None
        self.initial_position = None
        
        self.get_logger().info('RL适配器节点已启动')
        self.get_logger().info(f'RL输入格式: {self.rl_input_format}')
    
    def now(self) -> float:
        """获取当前时间戳"""
        return time.time()
    
    def on_rl_decision(self, msg: String):
        """接收RL决策输出"""
        try:
            # 解析RL平台的输出
            # 假设RL输出格式为: {"action": [vx, vy, vz, yaw_rate]} 或 {"position": [x, y, z, yaw]}
            rl_data = json.loads(msg.data)
            
            current_time = self.now()
            self.seq += 1
            
            # 构建统一命令消息
            header = Header(
                timestamp=current_time,
                source_id=SourceID.RL.value,
                seq=self.seq
            )
            
            meta = Meta(
                priority=PRIORITY_MAP[SourceID.RL],
                mode=self.rl_input_format,
                expires_at=current_time + self.command_timeout
            )
            
            body = Body()
            
            # 根据格式转换
            if self.rl_input_format == 'velocity' and 'action' in rl_data:
                action = rl_data['action']
                body.velocity = Velocity(
                    vx=float(action[0]),
                    vy=float(action[1]),
                    vz=float(action[2]),
                    yaw_rate=float(action[3]) if len(action) > 3 else 0.0
                )
            elif self.rl_input_format == 'position' and 'position' in rl_data:
                pos = rl_data['position']
                body.setpoint = Setpoint(
                    x=float(pos[0]),
                    y=float(pos[1]),
                    z=float(pos[2]),
                    yaw=float(pos[3]) if len(pos) > 3 else 0.0
                )
            else:
                self.get_logger().warning(f'无法解析RL数据格式: {rl_data}')
                return
            
            cmd = CommandMsg(header=header, meta=meta, body=body)
            
            # 发布到仲裁器
            cmd_msg = String()
            cmd_msg.data = cmd.to_json()
            self.pub_command.publish(cmd_msg)
            
            self.get_logger().debug(
                f'发送RL命令到仲裁器: seq={self.seq}, mode={self.rl_input_format}'
            )
            
        except Exception as e:
            self.get_logger().error(f'处理RL决策失败: {e}')
    
    def on_sync_cmd(self, msg: String):
        """接收同步命令，转发给RL可视化"""
        try:
            cmd = CommandMsg.from_json(msg.data)
            
            # 转换为RL平台可理解的格式
            viz_data = {
                'timestamp': cmd.header.timestamp,
                'source': cmd.header.source_id,
                'mode': cmd.meta.mode,
                'data': {}
            }
            
            if cmd.body.velocity:
                viz_data['data'] = {
                    'vx': cmd.body.velocity.vx,
                    'vy': cmd.body.velocity.vy,
                    'vz': cmd.body.velocity.vz,
                    'yaw_rate': cmd.body.velocity.yaw_rate
                }
            elif cmd.body.setpoint:
                viz_data['data'] = {
                    'x': cmd.body.setpoint.x,
                    'y': cmd.body.setpoint.y,
                    'z': cmd.body.setpoint.z,
                    'yaw': cmd.body.setpoint.yaw
                }
            elif cmd.body.trajectory:
                viz_data['data'] = {
                    'trajectory': [
                        {
                            't': pt.t_offset,
                            'x': pt.x,
                            'y': pt.y,
                            'z': pt.z,
                            'yaw': pt.yaw
                        } for pt in cmd.body.trajectory
                    ]
                }
            
            # 发布给RL可视化
            viz_msg = String()
            viz_msg.data = json.dumps(viz_data)
            self.pub_rl_viz.publish(viz_msg)
            
            self.get_logger().debug(f'转发命令到RL可视化: source={cmd.header.source_id}')
            
        except Exception as e:
            self.get_logger().error(f'转发同步命令到RL失败: {e}')
    
    def on_rl_state_feedback(self, msg: String):
        """接收RL平台状态反馈"""
        try:
            self.last_rl_state = json.loads(msg.data)
            self.get_logger().debug('收到RL平台状态反馈')
        except Exception as e:
            self.get_logger().error(f'解析RL状态失败: {e}')
    
    def on_initial_position(self, msg: String):
        """接收初始位置并通知RL平台"""
        try:
            initial_pos = json.loads(msg.data)
            self.initial_position = initial_pos
            
            self.get_logger().info(
                f'收到初始位置: source={initial_pos.get("source")}, '
                f'pos=({initial_pos["position"]["x"]:.2f}, '
                f'{initial_pos["position"]["y"]:.2f}, '
                f'{initial_pos["position"]["z"]:.2f})'
            )
            
            # 发送给RL平台（通过可视化话题）
            # RL平台可以使用这个位置来初始化环境
            rl_init_data = {
                'type': 'initial_position',
                'timestamp': initial_pos['timestamp'],
                'position': initial_pos['position'],
                'orientation': initial_pos.get('orientation')
            }
            
            viz_msg = String()
            viz_msg.data = json.dumps(rl_init_data)
            self.pub_rl_viz.publish(viz_msg)
            
            self.get_logger().info('✓ 初始位置已发送给RL平台')
            
        except Exception as e:
            self.get_logger().error(f'处理初始位置失败: {e}')
    
    def timer_callback(self):
        """定时发布状态"""
        state = {
            'timestamp': self.now(),
            'platform': 'rl',
            'position': self.last_rl_state.get('position') if self.last_rl_state else None,
            'velocity': self.last_rl_state.get('velocity') if self.last_rl_state else None,
            'initial_position': self.initial_position
        }
        
        msg = String()
        msg.data = json.dumps(state)
        self.pub_state.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RLAdapter()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

