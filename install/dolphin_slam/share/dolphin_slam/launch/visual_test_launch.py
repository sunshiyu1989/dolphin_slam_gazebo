#!/usr/bin/env python3
"""
视觉链路测试启动文件 - 简化版，专门测试从相机到位置细胞的视觉处理链路
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 获取包的共享目录
    pkg_share = get_package_share_directory('dolphin_slam')
    
    # 声明启动参数
    declare_dataset_path = DeclareLaunchArgument(
        'dataset_path',
        default_value='',
        description='数据集路径')
        
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='使用仿真时间')

    # 获取参数
    dataset_path = LaunchConfiguration('dataset_path')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # 参数文件路径
    params_file = PathJoinSubstitution([
        FindPackageShare('dolphin_slam'),
        'config',
        'dolphin_slam_params.yaml'
    ])

    # 设置全局仿真时间参数
    set_sim_time = SetParameter(name='use_sim_time', value=use_sim_time)

    # === 统一数据控制器 ===
    unified_data_controller = Node(
        package='dolphin_slam',
        executable='unified_data_controller',
        name='unified_data_controller',
        output='screen',
        parameters=[{
            'dataset_path': dataset_path,
            'playback_speed': 1.0,
            'sync_tolerance': 0.1,
            'use_sim_time': use_sim_time
        }]
    )

    # === 视觉处理节点 ===
    image_processing_node = Node(
        package='dolphin_slam',
        executable='image_processing_node',
        name='image_processing_node',
        output='screen',
        parameters=[params_file, {
            'use_sim_time': use_sim_time,
            'debug_mode': True,
            'process_every_n_frames': 1  # 处理每一帧
        }]
    )
    
    local_view_node = Node(
        package='dolphin_slam',
        executable='local_view_node',
        name='local_view_node',
        output='screen',
        parameters=[params_file, {
            'use_sim_time': use_sim_time,
            'enable_debug': True,
            'debug_level': 1,
            'underwater_mode': False,  # 关闭水下模式，减少过滤
            'max_matches_per_second': 50  # 增加匹配频率
        }]
    )
    
    place_cell_node = Node(
        package='dolphin_slam',
        executable='place_cell_node',
        name='place_cell_node',
        output='screen',
        parameters=[params_file, {
            'use_sim_time': False,
            'debug_mode': True,
            'odometry_topic': '/dolphin_slam/odometry'
        }]
    )

    # === 视觉链路诊断节点 ===
    visual_diagnostic = Node(
        package='dolphin_slam',
        executable='visual_chain_diagnostic',
        name='visual_chain_diagnostic',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )

    # === 启动序列安排 ===
    return LaunchDescription([
        # 参数声明
        declare_dataset_path,
        declare_use_sim_time,
        
        # 设置全局参数
        set_sim_time,
        
        # 第1步：启动数据控制器
        unified_data_controller,
        
        # 第2步：延迟1秒启动视觉处理节点
        TimerAction(
            period=1.0,
            actions=[image_processing_node]
        ),
        
        # 第3步：延迟2秒启动局部视觉节点
        TimerAction(
            period=2.0,
            actions=[local_view_node]
        ),
        
        # 第4步：延迟3秒启动位置细胞节点
        TimerAction(
            period=3.0,
            actions=[place_cell_node]
        ),
        
        # 第5步：延迟4秒启动诊断节点
        TimerAction(
            period=4.0,
            actions=[visual_diagnostic]
        ),
    ]) 