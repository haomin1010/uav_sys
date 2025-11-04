#!/usr/bin/env python3
"""
AirSim平台适配器
功能：
1. 接收中央算力（通过AirSim）的决策，转换为统一CommandMsg格式
2. 接收同步命令，通过AirSim API控制仿真无人机
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
from typing import Optional
from .command_msg import (
    CommandMsg, Header, Meta, Body, Velocity, Setpoint, TrajectoryPoint,
    SourceID, PRIORITY_MAP, CommandMode
)


class AirSimAdapter(Node):
    """AirSim平台适配器节点"""
    
    def __init__(self):
        super().__init__('airsim_adapter')
        
        # 参数配置
        self.declare_parameter('command_timeout', 1.0)
        self.declare_parameter('use_airsim_api', False)  # 是否实际连接AirSim
        self.declare_parameter('airsim_ip', '127.0.0.1')
        self.declare_parameter('vehicle_name', 'Drone1')
        self.declare_parameter('sync_initial_position', True)  # 是否同步初始位置
        self.declare_parameter('uav_id', 'uav1')  # 该adapter对应的无人机ID
        
        self.command_timeout = self.get_parameter('command_timeout').value
        self.use_airsim_api = self.get_parameter('use_airsim_api').value
        self.airsim_ip = self.get_parameter('airsim_ip').value
        self.vehicle_name = self.get_parameter('vehicle_name').value
        self.sync_initial_position = self.get_parameter('sync_initial_position').value
        self.uav_id = self.get_parameter('uav_id').value
        
        self.seq = 0
        self.airsim_client = None
        
        # 如果启用AirSim API，尝试连接
        if self.use_airsim_api:
            self.connect_airsim()
        
        # 订阅中央算力的决策输出
        self.sub_central_decision = self.create_subscription(
            String,
            '/central/decision_output',
            self.on_central_decision,
            10
        )
        
        # 发布转换后的命令给仲裁器
        self.pub_command = self.create_publisher(
            String,
            '/uav/source/central/cmd',
            10
        )
        
        # 订阅同步命令
        self.sub_sync_cmd = self.create_subscription(
            String,
            '/uav/sync/cmd',
            self.on_sync_cmd,
            10
        )
        
        # 发布AirSim状态
        self.pub_state = self.create_publisher(
            String,
            '/uav/airsim/state',
            10
        )
        
        # 订阅初始位置（单机模式）
        if self.sync_initial_position:
            self.sub_initial_position = self.create_subscription(
                String,
                '/uav/initial_position',
                self.on_initial_position,
                10
            )
        
        # 订阅编队同步命令（多机模式）
        self.sub_formation_sync = self.create_subscription(
            String,
            '/uav/formation_sync',
            self.on_formation_sync,
            10
        )
        
        # 定时更新状态
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz
        
        self.get_logger().info('AirSim适配器节点已启动')
        self.get_logger().info(f'绑定无人机ID: {self.uav_id}')
        if self.use_airsim_api:
            self.get_logger().info(f'AirSim连接: {self.airsim_ip}, 飞机: {self.vehicle_name}')
    
    def connect_airsim(self):
        """连接AirSim"""
        try:
            import airsim
            self.airsim_client = airsim.MultirotorClient(ip=self.airsim_ip)
            self.airsim_client.confirmConnection()
            self.airsim_client.enableApiControl(True, self.vehicle_name)
            self.airsim_client.armDisarm(True, self.vehicle_name)
            self.get_logger().info('成功连接到AirSim')
        except ImportError:
            self.get_logger().warning('未安装airsim包，AirSim API功能将不可用')
            self.use_airsim_api = False
        except Exception as e:
            self.get_logger().error(f'连接AirSim失败: {e}')
            self.use_airsim_api = False
    
    def now(self) -> float:
        return time.time()
    
    def on_central_decision(self, msg: String):
        """接收中央算力决策"""
        try:
            # 解析中央算力输出
            # 假设格式: {"type": "position/velocity/trajectory", "data": ...}
            decision = json.loads(msg.data)
            
            current_time = self.now()
            self.seq += 1
            
            header = Header(
                timestamp=current_time,
                source_id=SourceID.CENTRAL.value,
                seq=self.seq
            )
            
            decision_type = decision.get('type', 'position')
            
            meta = Meta(
                priority=PRIORITY_MAP[SourceID.CENTRAL],
                mode=decision_type,
                expires_at=current_time + self.command_timeout
            )
            
            body = Body()
            
            # 根据类型转换
            if decision_type == 'velocity' and 'velocity' in decision:
                vel = decision['velocity']
                body.velocity = Velocity(
                    vx=vel.get('vx', 0.0),
                    vy=vel.get('vy', 0.0),
                    vz=vel.get('vz', 0.0),
                    yaw_rate=vel.get('yaw_rate', 0.0)
                )
            elif decision_type == 'position' and 'position' in decision:
                pos = decision['position']
                body.setpoint = Setpoint(
                    x=pos.get('x', 0.0),
                    y=pos.get('y', 0.0),
                    z=pos.get('z', 0.0),
                    yaw=pos.get('yaw', 0.0)
                )
            elif decision_type == 'trajectory' and 'trajectory' in decision:
                traj = decision['trajectory']
                body.trajectory = [
                    TrajectoryPoint(
                        t_offset=pt.get('t', 0.0),
                        x=pt.get('x', 0.0),
                        y=pt.get('y', 0.0),
                        z=pt.get('z', 0.0),
                        yaw=pt.get('yaw', 0.0)
                    ) for pt in traj
                ]
            else:
                self.get_logger().warning(f'未知的中央决策格式: {decision}')
                return
            
            cmd = CommandMsg(header=header, meta=meta, body=body)
            
            # 发布到仲裁器
            cmd_msg = String()
            cmd_msg.data = cmd.to_json()
            self.pub_command.publish(cmd_msg)
            
            self.get_logger().debug(
                f'发送中央算力命令到仲裁器: seq={self.seq}, type={decision_type}'
            )
            
        except Exception as e:
            self.get_logger().error(f'处理中央决策失败: {e}')
    
    def on_sync_cmd(self, msg: String):
        """接收同步命令，执行AirSim控制"""
        try:
            cmd = CommandMsg.from_json(msg.data)
            
            self.get_logger().debug(
                f'收到同步命令: mode={cmd.meta.mode}, source={cmd.header.source_id}'
            )
            
            # 如果启用了AirSim API，执行实际控制
            if self.use_airsim_api and self.airsim_client:
                self.execute_airsim_command(cmd)
            
        except Exception as e:
            self.get_logger().error(f'执行AirSim命令失败: {e}')
    
    def execute_airsim_command(self, cmd: CommandMsg):
        """执行AirSim控制命令"""
        try:
            if cmd.body.velocity:
                vel = cmd.body.velocity
                self.airsim_client.moveByVelocityAsync(
                    vel.vx, vel.vy, vel.vz,
                    duration=0.1,  # 短时间内执行
                    vehicle_name=self.vehicle_name
                )
                self.get_logger().debug(f'AirSim执行速度命令: v=({vel.vx}, {vel.vy}, {vel.vz})')
                
            elif cmd.body.setpoint:
                pos = cmd.body.setpoint
                self.airsim_client.moveToPositionAsync(
                    pos.x, pos.y, pos.z, 5.0,  # 5m/s速度
                    vehicle_name=self.vehicle_name
                )
                self.get_logger().debug(f'AirSim执行位置命令: pos=({pos.x}, {pos.y}, {pos.z})')
                
            elif cmd.body.trajectory:
                # 转换为AirSim路径格式
                import airsim
                path = [
                    airsim.Vector3r(pt.x, pt.y, pt.z)
                    for pt in cmd.body.trajectory
                ]
                self.airsim_client.moveOnPathAsync(
                    path,
                    velocity=5.0,
                    vehicle_name=self.vehicle_name
                )
                self.get_logger().debug(f'AirSim执行轨迹命令: {len(path)}个点')
                
        except Exception as e:
            self.get_logger().error(f'AirSim API调用失败: {e}')
    
    def get_airsim_state(self) -> Optional[dict]:
        """获取AirSim状态"""
        if not self.use_airsim_api or not self.airsim_client:
            return None
        
        try:
            state = self.airsim_client.getMultirotorState(self.vehicle_name)
            kinematics = state.kinematics_estimated
            
            return {
                'position': {
                    'x': kinematics.position.x_val,
                    'y': kinematics.position.y_val,
                    'z': kinematics.position.z_val
                },
                'velocity': {
                    'vx': kinematics.linear_velocity.x_val,
                    'vy': kinematics.linear_velocity.y_val,
                    'vz': kinematics.linear_velocity.z_val
                },
                'orientation': {
                    'w': kinematics.orientation.w_val,
                    'x': kinematics.orientation.x_val,
                    'y': kinematics.orientation.y_val,
                    'z': kinematics.orientation.z_val
                }
            }
        except Exception as e:
            self.get_logger().error(f'获取AirSim状态失败: {e}')
            return None
    
    def on_initial_position(self, msg: String):
        """接收初始位置并设置AirSim"""
        try:
            initial_pos = json.loads(msg.data)
            
            self.get_logger().info(
                f'收到初始位置: source={initial_pos.get("source")}, '
                f'pos=({initial_pos["position"]["x"]:.2f}, '
                f'{initial_pos["position"]["y"]:.2f}, '
                f'{initial_pos["position"]["z"]:.2f})'
            )
            
            # 如果启用了AirSim API，设置仿真无人机位置
            if self.use_airsim_api and self.airsim_client:
                self.set_airsim_initial_position(initial_pos)
            else:
                self.get_logger().info(
                    'AirSim API未启用，仅记录初始位置（不实际设置）'
                )
                
        except Exception as e:
            self.get_logger().error(f'处理初始位置失败: {e}')
    
    def set_airsim_initial_position(self, initial_pos: dict):
        """设置AirSim仿真无人机的初始位置（单机模式）"""
        try:
            import airsim
            
            pos = initial_pos['position']
            orientation = initial_pos.get('orientation', {})
            
            # 创建Pose对象（注意：AirSim使用NED坐标系）
            pose = airsim.Pose()
            pose.position = airsim.Vector3r(pos['x'], pos['y'], pos['z'])
            
            # 如果有四元数信息，设置姿态
            if orientation:
                pose.orientation = airsim.Quaternionr(
                    orientation.get('x', 0.0),
                    orientation.get('y', 0.0),
                    orientation.get('z', 0.0),
                    orientation.get('w', 1.0)
                )
            
            # 设置位置
            self.airsim_client.simSetVehiclePose(pose, True, self.vehicle_name)
            
            self.get_logger().info(
                f'✓ AirSim位置已设置: ({pos["x"]:.2f}, {pos["y"]:.2f}, {pos["z"]:.2f})'
            )
            
        except Exception as e:
            self.get_logger().error(f'设置AirSim位置失败: {e}')
    
    def on_formation_sync(self, msg: String):
        """接收编队同步命令（多机模式）"""
        try:
            sync_data = json.loads(msg.data)
            
            if sync_data.get('type') != 'formation_sync':
                return
            
            # 查找该adapter对应的无人机同步数据
            uavs = sync_data.get('uavs', {})
            if self.uav_id not in uavs:
                self.get_logger().debug(f'编队同步命令中没有{self.uav_id}的数据')
                return
            
            uav_sync = uavs[self.uav_id]
            
            self.get_logger().info(
                f'收到编队同步命令: {self.uav_id}'
            )
            
            # 如果启用AirSim API，设置位置
            if self.use_airsim_api and self.airsim_client:
                self.set_airsim_formation_position(uav_sync)
            else:
                self.get_logger().info(
                    f'{self.uav_id}: AirSim API未启用，仅记录编队位置'
                )
                airsim_pos = uav_sync['airsim_target_position']
                self.get_logger().info(
                    f'  AirSim目标位置(NED): '
                    f'({airsim_pos["x"]:.2f}, {airsim_pos["y"]:.2f}, {airsim_pos["z"]:.2f})'
                )
            
        except Exception as e:
            self.get_logger().error(f'处理编队同步失败: {e}')
    
    def set_airsim_formation_position(self, uav_sync: dict):
        """设置AirSim编队位置"""
        try:
            import airsim
            
            # AirSim目标位置（NED坐标系）
            target_pos = uav_sync['airsim_target_position']
            orientation = uav_sync.get('orientation', {})
            
            # 创建Pose对象
            pose = airsim.Pose()
            pose.position = airsim.Vector3r(
                target_pos['x'],
                target_pos['y'],
                target_pos['z']
            )
            
            # 设置姿态
            if orientation:
                pose.orientation = airsim.Quaternionr(
                    orientation.get('x', 0.0),
                    orientation.get('y', 0.0),
                    orientation.get('z', 0.0),
                    orientation.get('w', 1.0)
                )
            
            # 设置位置
            self.airsim_client.simSetVehiclePose(pose, True, self.vehicle_name)
            
            self.get_logger().info(
                f'✓ {self.uav_id} AirSim位置已设置(NED): '
                f'({target_pos["x"]:.2f}, {target_pos["y"]:.2f}, {target_pos["z"]:.2f})'
            )
            
        except Exception as e:
            self.get_logger().error(f'设置AirSim编队位置失败: {e}')
    
    def timer_callback(self):
        """定时发布状态"""
        airsim_state = self.get_airsim_state()
        
        state = {
            'timestamp': self.now(),
            'platform': 'airsim',
            'position': airsim_state.get('position') if airsim_state else None,
            'velocity': airsim_state.get('velocity') if airsim_state else None,
            'connected': self.use_airsim_api and self.airsim_client is not None
        }
        
        msg = String()
        msg.data = json.dumps(state)
        self.pub_state.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = AirSimAdapter()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

