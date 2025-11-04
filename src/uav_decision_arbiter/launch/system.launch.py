#!/usr/bin/env python3
"""
无人机多源决策系统启动文件
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
    config_file = os.path.join(pkg_dir, 'config', 'default.yaml')
    
    # 声明启动参数
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=config_file,
        description='Full path to the config file'
    )
    
    # 仲裁器节点
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
    
    # 同步器节点
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
    
    # RL适配器节点
    rl_adapter_node = Node(
        package='uav_decision_arbiter',
        executable='rl_adapter',
        name='rl_adapter',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    
    # AirSim适配器节点
    airsim_adapter_node = Node(
        package='uav_decision_arbiter',
        executable='airsim_adapter',
        name='airsim_adapter',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    
    # PX4适配器节点
    px4_adapter_node = Node(
        package='uav_decision_arbiter',
        executable='px4_adapter',
        name='px4_adapter',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        config_file_arg,
        arbiter_node,
        synchronizer_node,
        rl_adapter_node,
        airsim_adapter_node,
        px4_adapter_node,
    ])

