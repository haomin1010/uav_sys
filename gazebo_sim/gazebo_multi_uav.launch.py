#!/usr/bin/env python3
"""
Gazebo + PX4 SITL 多机仿真启动文件
启动多个PX4 SITL实例和对应的MAVROS连接
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 参数
    num_uavs_arg = DeclareLaunchArgument(
        'num_uavs',
        default_value='3',
        description='Number of UAVs to spawn'
    )
    
    px4_dir_arg = DeclareLaunchArgument(
        'px4_dir',
        default_value=os.path.expanduser('~/PX4-Autopilot'),
        description='Path to PX4-Autopilot directory'
    )
    
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='empty',
        description='Gazebo world name'
    )
    
    use_gui_arg = DeclareLaunchArgument(
        'use_gui',
        default_value='true',
        description='Launch Gazebo GUI'
    )
    
    # 获取配置
    num_uavs = LaunchConfiguration('num_uavs')
    px4_dir = LaunchConfiguration('px4_dir')
    world = LaunchConfiguration('world')
    use_gui = LaunchConfiguration('use_gui')
    
    actions = [
        num_uavs_arg,
        px4_dir_arg,
        world_arg,
        use_gui_arg,
    ]
    
    # 创建多个MAVROS节点
    # 注意：PX4 SITL需要在外部脚本启动，这里只启动MAVROS
    for i in range(3):  # 默认3架
        uav_id = i + 1
        uav_name = f'uav{uav_id}'
        
        # FCU连接端口
        fcu_udp_port = 14540 + i
        gcs_udp_port = 14557 + i
        fcu_url = f'udp://:{fcu_udp_port}@127.0.0.1:{gcs_udp_port}'
        
        # MAVROS节点
        mavros_node = Node(
            package='mavros',
            executable='mavros_node',
            name='mavros',
            namespace=uav_name,
            output='screen',
            parameters=[{
                'fcu_url': fcu_url,
                'gcs_url': '',
                'system_id': uav_id,
                'target_system_id': uav_id,
                'target_component_id': 1,
                'fcu_protocol': 'v2.0',
                'plugin_allowlist': [
                    'sys_status',
                    'sys_time',
                    'actuator_control',
                    'command',
                    'local_position',
                    'global_position',
                    'setpoint_position',
                    'setpoint_velocity',
                    'setpoint_attitude',
                    'rc_io',
                    'manual_control'
                ]
            }],
            remappings=[
                (f'{uav_name}/mavros/setpoint_position/local', 
                 f'/{uav_name}/mavros/setpoint_position/local'),
            ]
        )
        
        # 延迟启动（给PX4 SITL时间启动）
        delayed_mavros = TimerAction(
            period=float(5.0 + i * 2.0),  # 错开启动时间
            actions=[mavros_node]
        )
        
        actions.append(delayed_mavros)
    
    return LaunchDescription(actions)

