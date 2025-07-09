#!/usr/bin/env python3
"""
Dolphin SLAM - 仿真环境启动文件
修复版 - 确保机器人正确spawn
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, 
    IncludeLaunchDescription,
    ExecuteProcess,
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
    robot_model = LaunchConfiguration('robot_model', default='auv_robot')
    rviz_config = LaunchConfiguration('rviz_config',
        default=os.path.join(pkg_share, 'rviz', 'dolphin_slam_sim.rviz'))
    enable_rviz = LaunchConfiguration('enable_rviz', default='true')
    record_bag = LaunchConfiguration('record_bag', default='false')
    
    # 启动参数声明
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='使用仿真时间')
        
    declare_world_file_cmd = DeclareLaunchArgument(
        'world_file',
        default_value=world_file,
        description='仿真世界文件')
        
    declare_robot_model_cmd = DeclareLaunchArgument(
        'robot_model',
        default_value='auv_robot',
        description='机器人模型')
        
    declare_rviz_config_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=rviz_config,
        description='RViz 配置文件路径')
        
    declare_enable_rviz_cmd = DeclareLaunchArgument(
        'enable_rviz',
        default_value='true',
        description='是否启动 RViz')
        
    declare_record_bag_cmd = DeclareLaunchArgument(
        'record_bag',
        default_value='false',
        description='是否录制 ROS bag')

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

    # 在 Gazebo 中生成机器人 - 修复版
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

    # 静态变换发布器（map -> odom）
    map_to_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # === SLAM 节点定义 ===
    
    # 里程计发布器（从 Gazebo 内置里程计和模型状态）
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
            'publish_tf': False  # 让Gazebo处理TF
        }]
    )
    
    # 轨迹评估器
    trajectory_evaluator = Node(
        package='dolphin_slam',
        executable='trajectory_evaluator_node',
        name='trajectory_evaluator',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'gps_origin_lat': 29.5014,
            'gps_origin_lon': 34.9167
        }]
    )
    
    # 图像处理节点
    image_processing_node = Node(
        package='dolphin_slam',
        executable='image_processing_node',
        name='image_processing_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'feature_type': 'SIFT',
            'max_features': 1000
        }]
    )
    
    # 局部视图节点
    local_view_node = Node(
        package='dolphin_slam',
        executable='local_view_node',
        name='local_view_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'underwater_mode': True,
            'frame_skip_threshold': 0.9,
            'max_match_rate': 20.0
        }]
    )
    
    # 位置细胞节点
    place_cell_node = Node(
        package='dolphin_slam',
        executable='place_cell_node',
        name='place_cell_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'neurons_per_dimension': 16,
            'spatial_scale': 1.2
        }]
    )
    
    # 经验地图节点
    experience_map_node = Node(
        package='dolphin_slam',
        executable='experience_map_node',
        name='experience_map_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'match_threshold': 0.8
        }]
    )
    
    # 航点控制器
    waypoint_controller = Node(
        package='dolphin_slam',
        executable='simple_waypoint_controller_node',
        name='waypoint_controller',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'cruise_speed': 1.0,
            'angular_speed': 0.5,
            'waypoint_tolerance': 2.0
        }]
    )

    # RViz 可视化
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(enable_rviz),
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )

    # 数据录制
    bag_record = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', 
             '/forward_camera/image_raw',
             '/odom',
             '/dolphin_slam/odometry', 
             '/imu/data',
             '/cmd_vel',
             '/tf',
             '/tf_static',
             '-o', 'dolphin_slam_simulation'],
        output='screen',
        condition=IfCondition(record_bag)
    )

    # === 定义延迟启动的动作 ===
    
    # 延迟启动 SLAM 节点（等待 Gazebo 完全启动）
    delayed_slam_nodes = TimerAction(
        period=10.0,  # 等待 10 秒
        actions=[
            simple_odom_publisher,
            trajectory_evaluator,
            image_processing_node,
            local_view_node,
            place_cell_node,
            experience_map_node
        ]
    )

    # 延迟启动控制器
    delayed_controller = TimerAction(
        period=15.0,  # 等待 15 秒让其他系统启动
        actions=[waypoint_controller]
    )

    # === 创建启动描述 ===
    ld = LaunchDescription()

    # 添加启动参数
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_world_file_cmd)
    ld.add_action(declare_robot_model_cmd)
    ld.add_action(declare_rviz_config_cmd)
    ld.add_action(declare_enable_rviz_cmd)
    ld.add_action(declare_record_bag_cmd)

    # 启动 Gazebo 仿真器
    ld.add_action(gazebo_launch)

    # 启动机器人相关
    ld.add_action(robot_state_publisher)
    ld.add_action(spawn_robot)

    # 静态变换
    ld.add_action(map_to_odom_tf)

    # 延迟启动 SLAM 系统
    ld.add_action(delayed_slam_nodes)
    
    # 延迟启动控制器
    ld.add_action(delayed_controller)

    # 可视化
    ld.add_action(rviz_node)

    # 数据录制
    ld.add_action(bag_record)

    return ld


if __name__ == '__main__':
    generate_launch_description()
