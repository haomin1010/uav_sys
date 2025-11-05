#!/usr/bin/env python3
"""
多机编队系统启动文件
支持集中仲裁架构和编队位置同步
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 获取包的配置目录
    pkg_dir = get_package_share_directory('uav_decision_arbiter')
    config_file = os.path.join(pkg_dir, 'config', 'multi_uav.yaml')
    
    # 声明启动参数
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=config_file,
        description='Full path to multi-UAV config file'
    )
    
    uav_count_arg = DeclareLaunchArgument(
        'uav_count',
        default_value='3',
        description='Number of UAVs in formation'
    )
    
    nodes = []
    
    # 核心节点
    # =====================================================
    
    # 集中仲裁器（一个实例管理所有无人机）
    arbiter_node = Node(
        package='uav_decision_arbiter',
        executable='arbiter',
        name='arbiter',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    nodes.append(arbiter_node)
    
    # 同步器
    synchronizer_node = Node(
        package='uav_decision_arbiter',
        executable='synchronizer',
        name='synchronizer',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    nodes.append(synchronizer_node)
    
    # 编队位置同步管理器（新增）
    formation_sync_node = Node(
        package='uav_decision_arbiter',
        executable='formation_sync',
        name='formation_sync',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    nodes.append(formation_sync_node)
    
    # 为每架无人机启动适配器
    # =====================================================
    uav_ids = ['uav1', 'uav2', 'uav3']
    
    for uav_id in uav_ids:
        # RL适配器
        rl_adapter = Node(
            package='uav_decision_arbiter',
            executable='rl_adapter',
            name=f'{uav_id}_rl_adapter',
            output='screen',
            parameters=[
                LaunchConfiguration('config_file'),
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ],
            remappings=[
                # 决策输入话题（带无人机ID）
                ('/rl/decision_output', f'/{uav_id}/rl/decision_output'),
                # 可视化输出话题
                ('/rl/visualization_cmd', f'/{uav_id}/rl/visualization_cmd'),
                # 状态反馈
                ('/rl/state_feedback', f'/{uav_id}/rl/state_feedback'),
            ]
        )
        nodes.append(rl_adapter)
        
        # AirSim适配器
        airsim_adapter = Node(
            package='uav_decision_arbiter',
            executable='airsim_adapter',
            name=f'{uav_id}_airsim_adapter',
            output='screen',
            parameters=[
                LaunchConfiguration('config_file'),
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ],
            remappings=[
                # 决策输入话题
                ('/central/decision_output', f'/{uav_id}/central/decision_output'),
            ]
        )
        nodes.append(airsim_adapter)
        
        # PX4适配器
        px4_adapter = Node(
            package='uav_decision_arbiter',
            executable='px4_adapter',
            name=f'{uav_id}_px4_adapter',
            output='screen',
            parameters=[
                LaunchConfiguration('config_file'),
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ],
            remappings=[
                # MAVROS话题（在Gazebo多机仿真中，话题直接在uav命名空间下）
                ('/mavros/local_position/pose', f'/{uav_id}/local_position/pose'),
                ('/mavros/local_position/velocity_local', f'/{uav_id}/local_position/velocity_local'),
                ('/mavros/setpoint_position/local', f'/{uav_id}/setpoint_position/local'),
                ('/mavros/setpoint_velocity/cmd_vel', f'/{uav_id}/setpoint_velocity/cmd_vel'),
            ]
        )
        nodes.append(px4_adapter)
    
    return LaunchDescription([
        use_sim_time_arg,
        config_file_arg,
        uav_count_arg,
    ] + nodes)

