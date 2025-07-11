#!/usr/bin/env python3
"""
Gazebo视觉链路测试启动文件 - 专门测试Gazebo环境中的视觉处理链路
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, 
    IncludeLaunchDescription,
    TimerAction
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration, 
    PathJoinSubstitution,
    Command
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 获取包的共享目录
    pkg_share = get_package_share_directory('dolphin_slam')
    
    # 声明启动参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_file = LaunchConfiguration('world_file', 
        default=os.path.join(pkg_share, 'worlds', 'underwater_world_enhanced.world'))
    enable_rviz = LaunchConfiguration('enable_rviz', default='false')  # 关闭RViz以专注测试
    
    # 启动参数声明
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='使用仿真时间')
        
    declare_world_file_cmd = DeclareLaunchArgument(
        'world_file',
        default_value=world_file,
        description='仿真世界文件')
        
    declare_enable_rviz_cmd = DeclareLaunchArgument(
        'enable_rviz',
        default_value='false',
        description='是否启动 RViz')

    # Gazebo 仿真器
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world': world_file,
            'verbose': 'true',
            'physics': 'ode',
            'debug': 'false',
            'pause': 'false'
        }.items()
    )

    # 生成机器人描述
    robot_description = Command([
        'xacro ',
        PathJoinSubstitution([pkg_share, 'urdf', 'auv_robot.xacro'])
    ])

    # 机器人状态发布器
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description
        }]
    )

    # 在 Gazebo 中生成机器人
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'auv_robot',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0', 
            '-z', '-10.0',  # 水下10米
            '-Y', '0.0'     # 朝向
        ],
        output='screen'
    )

    # 静态变换发布器
    map_to_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # === 简化的SLAM节点 ===
    
    # 里程计发布器
    simple_odom_publisher = Node(
        package='dolphin_slam',
        executable='simple_odom_publisher_node',
        name='simple_odom_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_name': 'auv_robot',
            'base_frame': 'base_link',
            'odom_frame': 'odom',
            'publish_tf': False
        }]
    )
    
    # 图像处理节点 - 重点测试
    image_processing_node = Node(
        package='dolphin_slam',
        executable='image_processing_node',
        name='image_processing_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'feature_type': 'SIFT',
            'max_features': 500,  # 减少特征数以提高速度
            'camera_topic': '/forward_camera/image_raw',  # Gazebo相机话题
            'sonar_topic': '/sonar/image_raw',
            'descriptors_topic': '/features/descriptors',
            'keypoints_topic': '/features/keypoints',
            'debug_mode': True,
            'process_every_n_frames': 1,  # 处理每一帧
            'enable_visualization': False  # 关闭可视化以提高性能
        }]
    )
    
    # 局部视图节点 - 重点测试
    local_view_node = Node(
        package='dolphin_slam',
        executable='local_view_node',
        name='local_view_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'underwater_mode': False,  # 关闭水下模式
            'frame_skip_threshold': 0.5,  # 降低跳过阈值
            'max_matches_per_second': 100,  # 增加匹配频率
            'descriptors_topic': '/features/descriptors',
            'matches_topic': '/local_view/matches',
            'enable_debug': True,
            'debug_level': 2,  # 最高调试级别
            'similarity_threshold': 0.3,  # 降低相似度阈值
            'min_match_count': 5,  # 降低最小匹配数
            'min_template_age': 1.0  # 降低模板最小年龄
        }]
    )
    
    # 位置细胞节点 - 重点测试
    place_cell_node = Node(
        package='dolphin_slam',
        executable='place_cell_node',
        name='place_cell_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'neurons_per_dimension': 16,
            'spatial_scale': 2.0,
            'workspace_center': [0.0, 0.0, -10.0],
            'odometry_topic': '/dolphin_slam/odometry',
            'visual_match_topic': '/local_view/matches',
            'activity_topic': '/place_cells/activity',
            'debug_mode': True,
            'movement_threshold': 0.02,  # 降低移动检测阈值
            'path_integration_strength': 3.0,
            'activity_injection_radius': 1.5
        }]
    )

    # 视觉链路诊断节点 - 核心测试工具
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
    
    # 延迟启动基础节点
    delayed_basic_nodes = TimerAction(
        period=5.0,  # 等待5秒
        actions=[
            robot_state_publisher,
            spawn_robot,
            map_to_odom_tf,
            simple_odom_publisher
        ]
    )

    # 延迟启动视觉处理节点
    delayed_visual_nodes = TimerAction(
        period=8.0,  # 等待8秒
        actions=[
            image_processing_node,
            local_view_node,
            place_cell_node,
            visual_diagnostic
        ]
    )

    # === 创建启动描述 ===
    ld = LaunchDescription()

    # 添加启动参数
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_world_file_cmd)
    ld.add_action(declare_enable_rviz_cmd)

    # 启动 Gazebo 仿真器
    ld.add_action(gazebo_launch)

    # 延迟启动节点
    ld.add_action(delayed_basic_nodes)
    ld.add_action(delayed_visual_nodes)

    return ld

if __name__ == '__main__':
    generate_launch_description() 