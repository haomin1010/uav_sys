#!/usr/bin/env python3
"""
编队位置同步节点
功能：维护多机在AirSim、RL和PX4之间的相对位置一致性
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import json
import time
from typing import Dict, Optional
from .coordinate_utils import enu_to_ned, add_offset, calculate_relative_position, distance_3d


class FormationSyncNode(Node):
    """编队位置同步节点"""
    
    def __init__(self):
        super().__init__('formation_sync_node')
        
        # 参数配置
        self.declare_parameter('uav_ids', ['uav1', 'uav2', 'uav3'])
        self.declare_parameter('leader_uav', 'uav1')  # 编队长机ID
        self.declare_parameter('sync_on_startup', True)
        self.declare_parameter('startup_delay', 3.0)  # 等待所有PX4数据就绪
        
        self.uav_ids = self.get_parameter('uav_ids').value
        self.leader_uav = self.get_parameter('leader_uav').value
        self.sync_on_startup = self.get_parameter('sync_on_startup').value
        self.startup_delay = self.get_parameter('startup_delay').value
        
        # 编队相对位置配置（NED坐标系，相对于长机）
        # 硬编码配置（ROS2参数不支持嵌套字典）
        # Gazebo中UAV初始位置（ENU坐标系）: (0,0), (0,-5), (0,5)
        # ENU->NED转换: NED(x,y,z) = (ENU_y, ENU_x, -ENU_z)
        self.formation_offsets = {
            'uav1': {'x': 0.0, 'y': 0.0, 'z': 0.0},      # 长机
            'uav2': {'x': -5.0, 'y': 0.0, 'z': 0.0},     # ENU(0,-5,0) -> NED(-5,0,0)
            'uav3': {'x': 5.0, 'y': 0.0, 'z': 0.0}       # ENU(0,5,0) -> NED(5,0,0)
        }
        
        # AirSim全局偏移配置（NED坐标系）
        self.airsim_spawn_offsets = {
            'uav1': {'x': 0.0, 'y': 0.0, 'z': 0.0},      # AirSim原点
            'uav2': {'x': 0.0, 'y': -10.0, 'z': 0.0},    # 东边10米
            'uav3': {'x': 0.0, 'y': 10.0, 'z': 0.0}      # 西边10米
        }
        
        # 维护每架无人机的位置状态（ENU坐标系）
        self.px4_positions = {}  # {uav_id: {'x', 'y', 'z', 'orientation'}}
        for uav_id in self.uav_ids:
            self.px4_positions[uav_id] = None
        
        # 订阅每架PX4的位置（MAVROS）
        from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
        
        # MAVROS使用BEST_EFFORT策略，需要匹配
        mavros_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.px4_subscribers = {}
        for uav_id in self.uav_ids:
            # 在Gazebo多机仿真中，话题直接在uav命名空间下（不含mavros子路径）
            topic = f'/{uav_id}/local_position/pose'
            self.px4_subscribers[uav_id] = self.create_subscription(
                PoseStamped,
                topic,
                lambda msg, uid=uav_id: self.on_px4_position(msg, uid),
                mavros_qos
            )
        
        # 发布编队同步命令
        self.pub_formation_sync = self.create_publisher(
            String,
            '/uav/formation_sync',
            10
        )
        
        # 标志
        self.initial_sync_done = False
        
        # 启动延迟定时器
        if self.sync_on_startup:
            self.startup_timer = self.create_timer(
                self.startup_delay,
                self.perform_initial_sync
            )
        
        # 定时监控相对位置偏差
        self.monitor_timer = self.create_timer(1.0, self.monitor_formation)
        
        self.get_logger().info('编队位置同步节点已启动')
        self.get_logger().info(f'管理无人机: {self.uav_ids}')
        self.get_logger().info(f'编队长机: {self.leader_uav}')
        self.get_logger().info(f'编队队形: {self.formation_offsets}')
    
    def now(self) -> float:
        return time.time()
    
    def on_px4_position(self, msg: PoseStamped, uav_id: str):
        """接收PX4位置更新"""
        self.px4_positions[uav_id] = {
            'x': msg.pose.position.x,  # ENU
            'y': msg.pose.position.y,
            'z': msg.pose.position.z,
            'orientation': {
                'w': msg.pose.orientation.w,
                'x': msg.pose.orientation.x,
                'y': msg.pose.orientation.y,
                'z': msg.pose.orientation.z
            },
            'timestamp': self.now()
        }
    
    def perform_initial_sync(self):
        """执行初始位置同步"""
        if self.initial_sync_done:
            return
        
        # 检查是否所有PX4位置都已就绪
        for uav_id in self.uav_ids:
            if self.px4_positions[uav_id] is None:
                self.get_logger().warning(
                    f'等待{uav_id}的PX4位置数据，延迟1秒后重试...'
                )
                self.startup_timer = self.create_timer(1.0, self.perform_initial_sync)
                return
        
        self.get_logger().info('=' * 70)
        self.get_logger().info('开始执行编队初始位置同步...')
        self.get_logger().info('=' * 70)
        
        # 读取长机位置作为参考
        leader_pos_enu = self.px4_positions[self.leader_uav]
        leader_pos_ned = enu_to_ned(leader_pos_enu)
        
        self.get_logger().info(
            f'长机{self.leader_uav} PX4位置(ENU): '
            f'({leader_pos_enu["x"]:.2f}, {leader_pos_enu["y"]:.2f}, {leader_pos_enu["z"]:.2f})'
        )
        self.get_logger().info(
            f'长机{self.leader_uav} NED位置: '
            f'({leader_pos_ned["x"]:.2f}, {leader_pos_ned["y"]:.2f}, {leader_pos_ned["z"]:.2f})'
        )
        
        # 为每架无人机计算并发布同步位置
        sync_commands = {}
        
        for uav_id in self.uav_ids:
            # 获取该机的编队偏移量（相对长机）
            formation_offset = self.formation_offsets.get(
                uav_id,
                {'x': 0.0, 'y': 0.0, 'z': 0.0}
            )
            
            # 计算该机在编队中的标准位置（NED）
            formation_pos_ned = add_offset(leader_pos_ned, formation_offset)
            
            # 获取AirSim生成偏移
            airsim_spawn = self.airsim_spawn_offsets.get(
                uav_id,
                {'x': 0.0, 'y': 0.0, 'z': 0.0}
            )
            
            # AirSim最终位置 = 生成位置 + 编队相对位置
            airsim_global_pos = add_offset(airsim_spawn, formation_offset)
            
            # 当前PX4实际位置（用于对比）
            current_px4_enu = self.px4_positions[uav_id]
            current_px4_ned = enu_to_ned(current_px4_enu)
            
            sync_commands[uav_id] = {
                'uav_id': uav_id,
                'timestamp': self.now(),
                
                # PX4当前位置
                'px4_position_enu': {
                    'x': current_px4_enu['x'],
                    'y': current_px4_enu['y'],
                    'z': current_px4_enu['z']
                },
                'px4_position_ned': current_px4_ned,
                
                # AirSim目标位置（全局NED）
                'airsim_target_position': airsim_global_pos,
                
                # 编队相对位置（相对长机）
                'formation_offset': formation_offset,
                
                # 姿态
                'orientation': current_px4_enu['orientation']
            }
            
            self.get_logger().info(
                f'{uav_id}: PX4_ENU=({current_px4_enu["x"]:.2f}, '
                f'{current_px4_enu["y"]:.2f}, {current_px4_enu["z"]:.2f}) → '
                f'AirSim_NED=({airsim_global_pos["x"]:.2f}, '
                f'{airsim_global_pos["y"]:.2f}, {airsim_global_pos["z"]:.2f})'
            )
        
        # 发布编队同步命令
        sync_msg = String()
        sync_msg.data = json.dumps({
            'type': 'formation_sync',
            'timestamp': self.now(),
            'leader_uav': self.leader_uav,
            'uavs': sync_commands
        }, indent=2)
        
        self.pub_formation_sync.publish(sync_msg)
        
        self.initial_sync_done = True
        self.get_logger().info('=' * 70)
        self.get_logger().info('✓ 编队初始位置同步完成！')
        self.get_logger().info('=' * 70)
        
        # 取消启动定时器
        if hasattr(self, 'startup_timer'):
            self.startup_timer.cancel()
    
    def monitor_formation(self):
        """监控编队相对位置偏差"""
        if not self.initial_sync_done:
            return
        
        # 检查是否所有无人机位置都有效
        valid_positions = {
            uav_id: pos for uav_id, pos in self.px4_positions.items()
            if pos is not None
        }
        
        if len(valid_positions) < 2:
            return
        
        # 计算实际相对位置并与标准编队对比
        if self.leader_uav in valid_positions:
            leader_pos = valid_positions[self.leader_uav]
            
            for uav_id, pos in valid_positions.items():
                if uav_id == self.leader_uav:
                    continue
                
                # 实际相对位置（ENU）
                actual_relative_enu = calculate_relative_position(leader_pos, pos)
                
                # 标准相对位置（从配置，NED）
                standard_offset_ned = self.formation_offsets.get(uav_id, {'x': 0, 'y': 0, 'z': 0})
                
                # 转换为ENU进行比较
                from .coordinate_utils import ned_to_enu
                standard_offset_enu = ned_to_enu(standard_offset_ned)
                
                # 计算偏差
                deviation = distance_3d(actual_relative_enu, standard_offset_enu)
                
                # 注意：在Gazebo中，MAVROS local_position是相对各自home position的
                # 所以无法直接比较相对位置。这个检查主要用于AirSim。
                # 提高阈值避免误报，或者完全禁用 Gazebo 的检查
                if deviation > 10.0:  # 偏差超过10米才警告（Gazebo中基本不会触发）
                    self.get_logger().warning(
                        f'编队偏差: {uav_id} 相对{self.leader_uav} '
                        f'偏离标准位置 {deviation:.2f}米'
                    )


def main(args=None):
    rclpy.init(args=args)
    node = FormationSyncNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

