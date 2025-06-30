#!/usr/bin/env python3
"""
Dolphin SLAM ROS2 启动文件
启动所有必要的节点和可视化工具
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 获取包的共享目录
    pkg_share = get_package_share_directory('dolphin_slam')
    
    # 声明启动参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    params_file = LaunchConfiguration('params_file',
        default=os.path.join(pkg_share, 'config', 'dolphin_slam_params.yaml'))
    rviz_config = LaunchConfiguration('rviz_config',
        default=os.path.join(pkg_share, 'rviz', 'dolphin_slam.rviz'))
    dataset_path = LaunchConfiguration('dataset_path', default='')
    enable_rviz = LaunchConfiguration('enable_rviz', default='true')
    
    # 启动参数声明
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='使用仿真时间')
        
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=params_file,
        description='参数文件的完整路径')
        
    declare_rviz_config_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=rviz_config,
        description='RViz 配置文件的完整路径')
        
    declare_dataset_path_cmd = DeclareLaunchArgument(
        'dataset_path',
        default_value='',
        description='数据集路径')
        
    declare_enable_rviz_cmd = DeclareLaunchArgument(
        'enable_rviz',
        default_value='true',
        description='启动 RViz')
    
    # 图像处理节点
    image_processing_node = Node(
        package='dolphin_slam',
        executable='image_processing_node',
        name='image_processing_node',
        output='screen',
        parameters=[params_file, {
            'use_sim_time': use_sim_time,
            'dataset.base_path': dataset_path
        }],
        remappings=[
            ('/camera/image_raw', '/dolphin_slam/camera/image'),
            ('/sonar/image_raw', '/dolphin_slam/sonar/image')
        ]
    )
    
    # 局部视觉细胞节点
    local_view_node = Node(
        package='dolphin_slam',
        executable='local_view_node',
        name='local_view_node',
        output='screen',
        parameters=[params_file, {
            'use_sim_time': use_sim_time
        }],
        remappings=[
            ('/features/descriptors', '/dolphin_slam/features/descriptors')
        ]
    )
    
    # 位置细胞网络节点
    place_cell_node = Node(
        package='dolphin_slam',
        executable='place_cell_node',
        name='place_cell_node',
        output='screen',
        parameters=[params_file, {
            'use_sim_time': use_sim_time
        }],
        remappings=[
            ('/robot/odometry', '/robot/odometry'),
            ('/local_view/matches', '/dolphin_slam/local_view/matches')
        ]
    )
    
    # 经验地图节点
    experience_map_node = Node(
        package='dolphin_slam',
        executable='experience_map_node',
        name='experience_map_node',
        output='screen',
        parameters=[params_file, {
            'use_sim_time': use_sim_time
        }],
        remappings=[
            ('/place_cells/activity', '/dolphin_slam/place_cells/activity'),
            ('/local_view/matches', '/dolphin_slam/local_view/matches')
        ]
    )
    
    # 机器人状态节点
    robot_state_node = Node(
        package='dolphin_slam',
        executable='robot_state_node',
        name='robot_state_node',
        output='screen',
        parameters=[params_file, {
            'use_sim_time': use_sim_time,
            'navigation_csv': '/media/psf/Samsung T7/SLAM Data/Sunboat_03-09-2023/2023-09-03-07-58-37/navigation/navigation.csv'
        }]
    )
    
    # 静态 TF 发布器（发布传感器变换）
    static_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        arguments=['0.75', '0', '-0.4', '0', '0', '0', 'base_link', 'dvl_link']
    )
    
    # RViz
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
    
    # 数据集播放器节点（可选）
    dataset_player_node = Node(
        package='dolphin_slam',
        executable='dataset_player_node',
        name='dataset_player_node',
        output='screen',
        parameters=[{
            'dataset_path': dataset_path,
            'playback_speed': 1.0,
            'loop': False,
            'use_sim_time': True
        }],
        condition=IfCondition(use_sim_time)
    )
    
    # 创建启动描述
    ld = LaunchDescription()
    
    # 添加启动参数
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_rviz_config_cmd)
    ld.add_action(declare_dataset_path_cmd)
    ld.add_action(declare_enable_rviz_cmd)
    
    # 添加节点
    ld.add_action(robot_state_node)
    ld.add_action(image_processing_node)
    ld.add_action(local_view_node)
    ld.add_action(place_cell_node)
    ld.add_action(experience_map_node)
    ld.add_action(static_tf_publisher)
    ld.add_action(rviz_node)
    
    # 如果使用仿真时间，添加数据集播放器
    ld.add_action(dataset_player_node)
    
    return ld
