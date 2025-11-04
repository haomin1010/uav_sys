#!/usr/bin/env python3
"""
RL决策平台主节点
集成：环境模拟 + RL策略 + 可视化界面 + ROS2通信
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import numpy as np
import time
from typing import Dict, List, Optional
import sys

# 添加路径
sys.path.append('/home/lihaomin/project/uav_sys/rl_platform')

from rl_env import MultiUAVEnvironment
from rl_policy import SimpleRLPolicy, CircleFormationPolicy
from rl_visualizer import UAVVisualizer


class RLPlatformNode(Node):
    """RL决策平台节点"""
    
    def __init__(self):
        super().__init__('rl_platform')
        
        # 参数配置
        self.declare_parameter('uav_ids', ['uav1', 'uav2', 'uav3'])
        self.declare_parameter('control_frequency', 10.0)  # Hz
        self.declare_parameter('policy_type', 'circle')  # 'simple' or 'circle'
        self.declare_parameter('enable_visualization', True)
        
        self.uav_ids = self.get_parameter('uav_ids').value
        self.control_frequency = self.get_parameter('control_frequency').value
        self.policy_type = self.get_parameter('policy_type').value
        self.enable_visualization = self.get_parameter('enable_visualization').value
        
        # 创建环境
        self.env = MultiUAVEnvironment(self.uav_ids)
        
        # 创建策略
        if self.policy_type == 'circle':
            self.policy = CircleFormationPolicy(self.uav_ids)
            self.get_logger().info('使用圆形编队策略')
        else:
            self.policy = SimpleRLPolicy(self.uav_ids)
            self.get_logger().info('使用简单导航策略')
        
        # 创建可视化
        if self.enable_visualization:
            try:
                self.visualizer = UAVVisualizer(self.uav_ids)
                self.get_logger().info('可视化界面已启动')
            except Exception as e:
                self.get_logger().warning(f'可视化初始化失败: {e}，继续无界面运行')
                self.visualizer = None
                self.enable_visualization = False
        else:
            self.visualizer = None
        
        # ROS2发布器：为每架无人机创建决策输出话题
        self.decision_publishers = {}
        for uav_id in self.uav_ids:
            self.decision_publishers[uav_id] = self.create_publisher(
                String,
                f'/{uav_id}/rl/decision_output',
                10
            )
        
        # ROS2订阅器：接收可视化命令和初始位置
        for uav_id in self.uav_ids:
            self.create_subscription(
                String,
                f'/{uav_id}/rl/visualization_cmd',
                lambda msg, uid=uav_id: self.on_visualization_cmd(msg, uid),
                10
            )
        
        # 订阅编队同步
        self.sub_formation_sync = self.create_subscription(
            String,
            '/uav/formation_sync',
            self.on_formation_sync,
            10
        )
        
        # 订阅权威命令（看谁在控制）
        self.sub_authoritative = self.create_subscription(
            String,
            '/uav/authoritative_cmd',
            self.on_authoritative_cmd,
            10
        )
        
        # 控制定时器
        control_period = 1.0 / self.control_frequency
        self.control_timer = self.create_timer(control_period, self.control_loop)
        
        # 可视化定时器（如果启用）
        if self.enable_visualization:
            self.viz_timer = self.create_timer(0.033, self.visualization_loop)  # 30 FPS
        
        # 状态变量
        self.rl_step = 0
        self.current_source = None
        self.formation_info = None
        self.paused = False
        
        self.get_logger().info('=' * 70)
        self.get_logger().info('RL决策平台已启动')
        self.get_logger().info(f'管理无人机: {self.uav_ids}')
        self.get_logger().info(f'控制频率: {self.control_frequency} Hz')
        self.get_logger().info(f'策略类型: {self.policy_type}')
        self.get_logger().info('=' * 70)
    
    def on_formation_sync(self, msg: String):
        """接收编队同步信息"""
        try:
            data = json.loads(msg.data)
            
            if data.get('type') != 'formation_sync':
                return
            
            self.formation_info = data
            
            self.get_logger().info('收到编队同步信息，更新RL环境初始位置')
            
            # 更新环境中无人机的位置
            initial_positions = {}
            for uav_id, uav_data in data.get('uavs', {}).items():
                if uav_id in self.uav_ids:
                    pos_enu = uav_data['px4_position_enu']
                    initial_positions[uav_id] = np.array([
                        pos_enu['x'],
                        pos_enu['y'],
                        pos_enu['z']
                    ])
            
            self.env.reset(initial_positions)
            
            # 清空轨迹
            if self.visualizer:
                self.visualizer.clear_trajectories()
            
            self.get_logger().info(f'✓ RL环境已重置，初始位置: {len(initial_positions)}架无人机')
            
        except Exception as e:
            self.get_logger().error(f'处理编队同步失败: {e}')
    
    def on_visualization_cmd(self, msg: String, uav_id: str):
        """接收可视化命令（来自系统）"""
        try:
            data = json.loads(msg.data)
            
            # 处理初始位置命令
            if data.get('type') == 'initial_position':
                pos = data['position']
                initial_pos = np.array([pos['x'], pos['y'], pos['z']])
                
                if uav_id in self.env.uavs:
                    self.env.uavs[uav_id].position = initial_pos
                
                self.get_logger().debug(
                    f'{uav_id} 收到初始位置: ({pos["x"]:.2f}, {pos["y"]:.2f}, {pos["z"]:.2f})'
                )
            
            # 可以处理其他可视化命令
            elif data.get('type') == 'trajectory':
                pass  # TODO: 显示其他源的轨迹
            
        except Exception as e:
            self.get_logger().error(f'处理可视化命令失败: {e}')
    
    def on_authoritative_cmd(self, msg: String):
        """监听权威命令，判断当前谁在控制"""
        try:
            data = json.loads(msg.data)
            header = data.get('header', {})
            source = header.get('source_id')
            
            if source != self.current_source:
                self.current_source = source
                if self.visualizer:
                    self.visualizer.current_source = source
                
                self.get_logger().info(f'当前控制源: {source}')
        
        except Exception as e:
            self.get_logger().error(f'处理权威命令失败: {e}')
    
    def control_loop(self):
        """RL控制循环"""
        if self.paused:
            return
        
        try:
            # 获取观测
            observations = self.env.get_observations()
            
            # RL策略决策
            if self.policy_type == 'circle':
                actions = self.policy.get_batch_actions(observations, self.env.time)
            else:
                # 简单策略需要所有位置（用于避碰）
                all_positions = {
                    uid: self.env.uavs[uid].position 
                    for uid in self.uav_ids
                }
                actions = {}
                for uav_id, obs in observations.items():
                    actions[uav_id] = self.policy.get_action(obs, uav_id, all_positions)
            
            # 环境步进
            obs, rewards, dones, info = self.env.step(actions)
            
            # 发布决策到ROS2
            self.publish_decisions(actions)
            
            self.rl_step += 1
            
            if self.rl_step % 50 == 0:
                self.get_logger().info(
                    f'RL Step {self.rl_step}: '
                    f'控制{len(actions)}架无人机'
                )
        
        except Exception as e:
            self.get_logger().error(f'控制循环错误: {e}')
    
    def publish_decisions(self, actions: Dict[str, np.ndarray]):
        """发布RL决策到ROS2"""
        for uav_id, action in actions.items():
            if uav_id not in self.decision_publishers:
                continue
            
            # 转换为ROS消息格式
            decision = {
                "action": action.tolist(),
                "timestamp": time.time()
            }
            
            msg = String()
            msg.data = json.dumps(decision)
            self.decision_publishers[uav_id].publish(msg)
    
    def visualization_loop(self):
        """可视化循环"""
        if not self.visualizer:
            return
        
        try:
            # 处理pygame事件
            events = self.visualizer.handle_events()
            
            if events['quit']:
                self.get_logger().info('用户退出')
                rclpy.shutdown()
                return
            
            if events['pause']:
                self.paused = not self.paused
                status = "暂停" if self.paused else "继续"
                self.get_logger().info(f'RL决策{status}')
            
            if events['reset']:
                self.env.reset()
                self.visualizer.clear_trajectories()
                self.rl_step = 0
                self.get_logger().info('环境已重置')
            
            # 获取无人机状态
            uav_states = {}
            targets = {}
            
            for uav_id in self.uav_ids:
                uav = self.env.get_state(uav_id)
                if uav:
                    target = self.env.targets.get(uav_id, np.zeros(3))
                    target_distance = np.linalg.norm(uav.position - target)
                    
                    uav_states[uav_id] = {
                        'position': uav.position,
                        'velocity': uav.velocity,
                        'yaw': uav.yaw,
                        'target_distance': target_distance
                    }
                    targets[uav_id] = target
            
            # 渲染
            self.visualizer.render(
                uav_states=uav_states,
                targets=targets,
                current_source=self.current_source,
                formation_info=self.formation_info,
                rl_step=self.rl_step
            )
        
        except Exception as e:
            self.get_logger().error(f'可视化错误: {e}')
    
    def destroy_node(self):
        """清理资源"""
        if self.visualizer:
            self.visualizer.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = RLPlatformNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\n用户中断')
    except Exception as e:
        print(f'错误: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

