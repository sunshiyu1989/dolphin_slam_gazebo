#!/usr/bin/env python3
"""
Dolphin SLAM ROS2 启动文件 - 修复版
使用统一数据控制器解决同步问题，修复TF坐标变换
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, GroupAction
from launch.conditions import IfCondition
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
        
    declare_enable_rviz = DeclareLaunchArgument(
        'enable_rviz',
        default_value='true',
        description='启动 RViz')
        
    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(pkg_share, 'rviz', 'dolphin_slam.rviz'),
        description='RViz 配置文件路径')
        
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='使用仿真时间')
        
    declare_playback_speed = DeclareLaunchArgument(
        'playback_speed',
        default_value='1.0',
        description='播放速度')
        
    declare_auto_stop = DeclareLaunchArgument(
        'auto_stop',
        default_value='true',
        description='数据播放完成后自动停止系统')

    # 获取参数
    dataset_path = LaunchConfiguration('dataset_path')
    enable_rviz = LaunchConfiguration('enable_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    use_sim_time = LaunchConfiguration('use_sim_time')
    playback_speed = LaunchConfiguration('playback_speed')
    auto_stop = LaunchConfiguration('auto_stop')
    
    # 参数文件路径
    params_file = PathJoinSubstitution([
        FindPackageShare('dolphin_slam'),
        'config',
        'dolphin_slam_params.yaml'
    ])

    # 设置全局仿真时间参数
    set_sim_time = SetParameter(name='use_sim_time', value=use_sim_time)

    # === TF变换发布器组 ===
    tf_publishers_group = GroupAction([
        # 1. map -> odom 变换
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            output='screen'
        ),
        
        # 2. odom -> base_link 变换（这个会由里程计动态发布，这里先建立静态关系）
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_base_link_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
            output='screen'
        ),
        
        # 3. 传感器变换
        # DVL传感器 (Doppler Velocity Log) - 在机器人底部后方
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='dvl_tf_publisher',
            arguments=['0.75', '0', '-0.4', '0', '0', '0', 'base_link', 'dvl_link'],
            output='screen'
        ),
        
        # 相机传感器 - 在机器人前方
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_tf_publisher',
            arguments=['0.5', '0', '0.2', '0', '0', '0', 'base_link', 'camera_link'],
            output='screen'
        ),
        
        # 声呐传感器 - 在机器人底部前方
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='sonar_tf_publisher',
            arguments=['0.8', '0', '-0.3', '0', '0', '0', 'base_link', 'sonar_link'],
            output='screen'
        ),
    ])

    # === 统一数据控制器（替代原来的dataset_player和robot_state节点）===
    unified_data_controller = Node(
        package='dolphin_slam',
        executable='unified_data_controller',
        name='unified_data_controller',
        output='screen',
        parameters=[{
            'dataset_path': dataset_path,
            'playback_speed': playback_speed,
            'sync_tolerance': 0.1,  # 时间同步容差
            'use_sim_time': use_sim_time
        }]
    )

    # === SLAM处理节点组 ===
    slam_processors_group = GroupAction([
        # 图像处理节点
        Node(
            package='dolphin_slam',
            executable='image_processing_node',
            name='image_processing_node',
            output='screen',
            parameters=[params_file, {
                'use_sim_time': use_sim_time
            }]
        ),
        
        # 局部视觉细胞节点
        Node(
            package='dolphin_slam',
            executable='local_view_node',
            name='local_view_node',
            output='screen',
            parameters=[params_file, {
                'use_sim_time': use_sim_time
            }]
        ),
        
        # 位置细胞网络节点
        Node(
            package='dolphin_slam',
            executable='place_cell_node',
            name='place_cell_node',
            output='screen',
            parameters=[params_file, {
                'use_sim_time': False,
                'debug_mode': True,
                'odometry_topic': '/dolphin_slam/odometry'
            }]
        ),
        
        # 经验地图节点
        Node(
            package='dolphin_slam',
            executable='experience_map_node',
            name='experience_map_node',
            output='screen',
            parameters=[params_file, {
                'use_sim_time': use_sim_time
            }]
        ),
    ])

    # === 系统监控节点 ===
    system_monitor = Node(
        package='dolphin_slam',
        executable='system_stop_monitor',
        name='system_stop_monitor',
        output='screen',
        parameters=[{
            'grace_period': 5.0,
            'auto_shutdown': auto_stop,
            'use_sim_time': use_sim_time
        }]
    )

    # === RViz可视化节点 ===
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        condition=IfCondition(enable_rviz),
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )

    # === 启动序列安排 ===
    return LaunchDescription([
        # 参数声明
        declare_dataset_path,
        declare_enable_rviz,
        declare_rviz_config,
        declare_use_sim_time,
        declare_playback_speed,
        declare_auto_stop,
        
        # 设置全局参数
        set_sim_time,
        
        # 第1步：立即启动TF发布器（为其他节点提供坐标系）
        tf_publishers_group,
        
        # 第2步：启动系统监控（需要早启动以监听整个过程）
        system_monitor,
        
        # 第3步：延迟1秒启动SLAM处理节点（确保TF已建立）
        TimerAction(
            period=1.0,
            actions=[slam_processors_group]
        ),
        
        # 第4步：延迟2秒启动数据控制器（确保所有处理节点准备就绪）
        TimerAction(
            period=2.0,
            actions=[unified_data_controller]
        ),
        
        # 第5步：延迟3秒启动RViz（确保有数据流）
        TimerAction(
            period=3.0,
            actions=[rviz_node]
        ),
    ])
