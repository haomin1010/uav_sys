#!/usr/bin/env python3
"""
PX4飞控适配器
功能：
1. 监听人类通过遥控器/地面站的控制信号，转换为统一CommandMsg格式
2. 接收同步命令，通过MAVROS控制真实或SITL的PX4飞控
注意：需要MAVROS运行并连接到PX4
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, TwistStamped
import json
import time
from typing import Optional
from .command_msg import (
    CommandMsg, Header, Meta, Body, Velocity, Setpoint,
    SourceID, PRIORITY_MAP, CommandMode
)


class PX4Adapter(Node):
    """PX4飞控适配器节点"""
    
    def __init__(self):
        super().__init__('px4_adapter')
        
        # 参数配置
        self.declare_parameter('command_timeout', 0.5)
        self.declare_parameter('human_control_threshold', 0.1)  # 检测人类控制的阈值
        self.declare_parameter('use_mavros', True)  # 是否使用MAVROS
        self.declare_parameter('publish_initial_position', True)  # 是否发布初始位置
        self.declare_parameter('initial_position_delay', 2.0)  # 等待MAVROS数据的延迟（秒）
        
        self.command_timeout = self.get_parameter('command_timeout').value
        self.human_control_threshold = self.get_parameter('human_control_threshold').value
        self.use_mavros = self.get_parameter('use_mavros').value
        self.publish_initial_position = self.get_parameter('publish_initial_position').value
        self.initial_position_delay = self.get_parameter('initial_position_delay').value
        
        self.seq = 0
        self.last_manual_input = None
        self.last_px4_position = None
        self.last_px4_velocity = None
        
        # 如果使用MAVROS，订阅相关话题
        if self.use_mavros:
            self.setup_mavros()
        
        # 发布转换后的命令给仲裁器（人类控制信号）
        self.pub_command = self.create_publisher(
            String,
            '/uav/source/human/cmd',
            10
        )
        
        # 订阅同步命令
        self.sub_sync_cmd = self.create_subscription(
            String,
            '/uav/sync/cmd',
            self.on_sync_cmd,
            10
        )
        
        # 发布PX4状态
        self.pub_state = self.create_publisher(
            String,
            '/uav/px4/state',
            10
        )
        
        # 发布初始位置
        self.pub_initial_position = self.create_publisher(
            String,
            '/uav/initial_position',
            10
        )
        
        # 定时器
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz
        
        # 初始位置发布标志
        self.initial_position_published = False
        
        # 启动后延迟发布初始位置
        if self.publish_initial_position and self.use_mavros:
            self.initial_position_timer = self.create_timer(
                self.initial_position_delay,
                self.publish_initial_position_once
            )
        
        self.get_logger().info('PX4适配器节点已启动')
        if self.use_mavros:
            self.get_logger().info('使用MAVROS连接PX4')
    
    def setup_mavros(self):
        """设置MAVROS相关的订阅和发布"""
        # 订阅当前位置
        self.sub_mavros_position = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.on_mavros_position,
            10
        )
        
        # 订阅当前速度
        self.sub_mavros_velocity = self.create_subscription(
            TwistStamped,
            '/mavros/local_position/velocity_local',
            self.on_mavros_velocity,
            10
        )
        
        # 订阅RC输入（检测人类控制）
        # 注意：这里简化处理，实际可能需要订阅 /mavros/rc/in 或 /mavros/manual_control/control
        self.sub_mavros_manual = self.create_subscription(
            TwistStamped,
            '/mavros/setpoint_velocity/cmd_vel_unstamped',
            self.on_manual_control,
            10
        )
        
        # 发布位置设定点
        self.pub_mavros_position = self.create_publisher(
            PoseStamped,
            '/mavros/setpoint_position/local',
            10
        )
        
        # 发布速度设定点
        self.pub_mavros_velocity = self.create_publisher(
            TwistStamped,
            '/mavros/setpoint_velocity/cmd_vel',
            10
        )
    
    def now(self) -> float:
        return time.time()
    
    def on_mavros_position(self, msg: PoseStamped):
        """接收PX4位置"""
        self.last_px4_position = {
            'x': msg.pose.position.x,
            'y': msg.pose.position.y,
            'z': msg.pose.position.z,
            'orientation': {
                'w': msg.pose.orientation.w,
                'x': msg.pose.orientation.x,
                'y': msg.pose.orientation.y,
                'z': msg.pose.orientation.z
            }
        }
    
    def on_mavros_velocity(self, msg: TwistStamped):
        """接收PX4速度"""
        self.last_px4_velocity = {
            'vx': msg.twist.linear.x,
            'vy': msg.twist.linear.y,
            'vz': msg.twist.linear.z,
            'yaw_rate': msg.twist.angular.z
        }
    
    def on_manual_control(self, msg: TwistStamped):
        """检测人类手动控制信号"""
        # 检查速度指令是否超过阈值（表示人类在操作）
        vx = msg.twist.linear.x
        vy = msg.twist.linear.y
        vz = msg.twist.linear.z
        yaw_rate = msg.twist.angular.z
        
        magnitude = (vx**2 + vy**2 + vz**2)**0.5
        
        if magnitude > self.human_control_threshold or abs(yaw_rate) > 0.1:
            # 检测到人类控制信号
            self.last_manual_input = {
                'vx': vx,
                'vy': vy,
                'vz': vz,
                'yaw_rate': yaw_rate,
                'timestamp': self.now()
            }
            
            # 将人类控制转换为统一命令格式发送给仲裁器
            self.publish_human_command(vx, vy, vz, yaw_rate)
    
    def publish_human_command(self, vx: float, vy: float, vz: float, yaw_rate: float):
        """发布人类控制命令到仲裁器"""
        current_time = self.now()
        self.seq += 1
        
        header = Header(
            timestamp=current_time,
            source_id=SourceID.HUMAN.value,
            seq=self.seq
        )
        
        meta = Meta(
            priority=PRIORITY_MAP[SourceID.HUMAN],
            mode=CommandMode.VELOCITY.value,
            expires_at=current_time + self.command_timeout
        )
        
        body = Body(
            velocity=Velocity(vx=vx, vy=vy, vz=vz, yaw_rate=yaw_rate)
        )
        
        cmd = CommandMsg(header=header, meta=meta, body=body)
        
        # 发布到仲裁器
        cmd_msg = String()
        cmd_msg.data = cmd.to_json()
        self.pub_command.publish(cmd_msg)
        
        self.get_logger().debug(
            f'发送人类控制命令: v=({vx:.2f}, {vy:.2f}, {vz:.2f}), '
            f'yaw_rate={yaw_rate:.2f}'
        )
    
    def on_sync_cmd(self, msg: String):
        """接收同步命令，通过MAVROS控制PX4"""
        try:
            cmd = CommandMsg.from_json(msg.data)
            
            self.get_logger().debug(
                f'收到同步命令: mode={cmd.meta.mode}, source={cmd.header.source_id}'
            )
            
            # 如果命令来自人类自己，不需要再发送回PX4（避免回环）
            if cmd.header.source_id == SourceID.HUMAN.value:
                return
            
            # 通过MAVROS控制PX4
            if self.use_mavros:
                self.execute_px4_command(cmd)
            
        except Exception as e:
            self.get_logger().error(f'执行PX4命令失败: {e}')
    
    def execute_px4_command(self, cmd: CommandMsg):
        """执行PX4控制命令"""
        try:
            if cmd.body.velocity:
                # 发布速度命令
                vel_msg = TwistStamped()
                vel_msg.header.stamp = self.get_clock().now().to_msg()
                vel_msg.header.frame_id = "map"
                vel_msg.twist.linear.x = cmd.body.velocity.vx
                vel_msg.twist.linear.y = cmd.body.velocity.vy
                vel_msg.twist.linear.z = cmd.body.velocity.vz
                vel_msg.twist.angular.z = cmd.body.velocity.yaw_rate
                
                self.pub_mavros_velocity.publish(vel_msg)
                self.get_logger().debug(
                    f'PX4执行速度命令: v=({cmd.body.velocity.vx}, '
                    f'{cmd.body.velocity.vy}, {cmd.body.velocity.vz})'
                )
                
            elif cmd.body.setpoint:
                # 发布位置命令
                pos_msg = PoseStamped()
                pos_msg.header.stamp = self.get_clock().now().to_msg()
                pos_msg.header.frame_id = "map"
                pos_msg.pose.position.x = cmd.body.setpoint.x
                pos_msg.pose.position.y = cmd.body.setpoint.y
                pos_msg.pose.position.z = cmd.body.setpoint.z
                
                # 简化：yaw转四元数（只考虑绕z轴旋转）
                import math
                yaw = cmd.body.setpoint.yaw
                pos_msg.pose.orientation.w = math.cos(yaw / 2)
                pos_msg.pose.orientation.z = math.sin(yaw / 2)
                
                self.pub_mavros_position.publish(pos_msg)
                self.get_logger().debug(
                    f'PX4执行位置命令: pos=({cmd.body.setpoint.x}, '
                    f'{cmd.body.setpoint.y}, {cmd.body.setpoint.z})'
                )
                
            elif cmd.body.trajectory:
                # PX4轨迹控制需要逐点发送，这里简化为发送第一个点
                if len(cmd.body.trajectory) > 0:
                    pt = cmd.body.trajectory[0]
                    pos_msg = PoseStamped()
                    pos_msg.header.stamp = self.get_clock().now().to_msg()
                    pos_msg.header.frame_id = "map"
                    pos_msg.pose.position.x = pt.x
                    pos_msg.pose.position.y = pt.y
                    pos_msg.pose.position.z = pt.z
                    
                    import math
                    yaw = pt.yaw
                    pos_msg.pose.orientation.w = math.cos(yaw / 2)
                    pos_msg.pose.orientation.z = math.sin(yaw / 2)
                    
                    self.pub_mavros_position.publish(pos_msg)
                    self.get_logger().debug('PX4执行轨迹命令（第一个点）')
                
        except Exception as e:
            self.get_logger().error(f'PX4 MAVROS命令执行失败: {e}')
    
    def publish_initial_position_once(self):
        """发布一次初始位置（启动时）"""
        if self.initial_position_published:
            return
        
        if self.last_px4_position is None:
            self.get_logger().warning(
                '尚未接收到PX4位置数据，延迟1秒后重试...'
            )
            # 延迟后再试
            self.initial_position_timer = self.create_timer(
                1.0,
                self.publish_initial_position_once
            )
            return
        
        # 发布初始位置
        initial_pos = {
            'timestamp': self.now(),
            'source': 'px4',
            'position': {
                'x': self.last_px4_position['x'],
                'y': self.last_px4_position['y'],
                'z': self.last_px4_position['z']
            },
            'orientation': self.last_px4_position['orientation']
        }
        
        msg = String()
        msg.data = json.dumps(initial_pos)
        self.pub_initial_position.publish(msg)
        
        self.initial_position_published = True
        self.get_logger().info(
            f'发布初始位置: x={initial_pos["position"]["x"]:.2f}, '
            f'y={initial_pos["position"]["y"]:.2f}, '
            f'z={initial_pos["position"]["z"]:.2f}'
        )
        
        # 取消定时器
        if hasattr(self, 'initial_position_timer'):
            self.initial_position_timer.cancel()
    
    def timer_callback(self):
        """定时发布状态"""
        state = {
            'timestamp': self.now(),
            'platform': 'px4',
            'position': self.last_px4_position,
            'velocity': self.last_px4_velocity,
            'manual_control': self.last_manual_input,
            'connected': self.use_mavros
        }
        
        msg = String()
        msg.data = json.dumps(state)
        self.pub_state.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PX4Adapter()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

