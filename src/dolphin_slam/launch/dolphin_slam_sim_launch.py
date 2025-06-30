#!/usr/bin/env python3
"""
Dolphin SLAM - 仿真环境启动文件
用于在仿真环境中运行 Dolphin SLAM
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, 
    IncludeLaunchDescription,
    ExecuteProcess,
    TimerAction,
    RegisterEventHandler
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration, 
    PathJoinSubstitution,
    PythonExpression,
    Command
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit


def generate_launch_description():
    # 获取包的共享目录
    pkg_share = get_package_share_directory('dolphin_slam')
    
    # 声明启动参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # 仿真器选择
    simulator = LaunchConfiguration('simulator', default='gazebo')  # gazebo 或 uwsim
    
    # 世界文件
    world_file = LaunchConfiguration('world_file', 
        default=os.path.join(pkg_share, 'worlds', 'underwater_world.world'))
        
    # 机器人模型
    robot_model = LaunchConfiguration('robot_model', default='auv_robot')
    
    # 参数文件
    params_file = LaunchConfiguration('params_file',
        default=os.path.join(pkg_share, 'config', 'dolphin_slam_sim_params.yaml'))
        
    # RViz 配置
    rviz_config = LaunchConfiguration('rviz_config',
        default=os.path.join(pkg_share, 'rviz', 'dolphin_slam_sim.rviz'))
        
    # 是否启动 RViz
    enable_rviz = LaunchConfiguration('enable_rviz', default='true')
    
    # 是否录制 bag
    record_bag = LaunchConfiguration('record_bag', default='false')
    
    # 启动参数声明
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='使用仿真时间')
        
    declare_simulator_cmd = DeclareLaunchArgument(
        'simulator',
        default_value='gazebo',
        choices=['gazebo', 'uwsim'],
        description='选择仿真器')
        
    declare_world_file_cmd = DeclareLaunchArgument(
        'world_file',
        default_value=world_file,
        description='仿真世界文件')
        
    declare_robot_model_cmd = DeclareLaunchArgument(
        'robot_model',
        default_value='auv_robot',
        description='机器人模型')
        
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=params_file,
        description='参数文件路径')
        
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
            'debug': 'false'
        }.items(),
        condition=IfCondition(PythonExpression(["'", simulator, "' == 'gazebo'"]))
    )
    
    # 生成机器人描述
    robot_description = Command([
        'xacro ',
        PathJoinSubstitution([pkg_share, 'urdf', robot_model, '.xacro']),
        ' use_sim:=true',
        ' namespace:=dolphin_slam'
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
            '-z', '-5.0',  # 水下 5 米
            '-Y', '0.0'
        ],
        output='screen',
        condition=IfCondition(PythonExpression(["'", simulator, "' == 'gazebo'"]))
    )
    
    # 传感器仿真节点
    # 相机仿真
    camera_sim_node = Node(
        package='dolphin_slam',
        executable='camera_sim_node',
        name='camera_sim_node',
        output='screen',
        parameters=[params_file, {
            'use_sim_time': use_sim_time,
            'camera_topic': '/dolphin_slam/camera/image_raw',
            'camera_info_topic': '/dolphin_slam/camera/camera_info',
            'frame_id': 'camera_link',
            'update_rate': 10.0
        }]
    )
    
    # 声呐仿真
    sonar_sim_node = Node(
        package='dolphin_slam',
        executable='sonar_sim_node',
        name='sonar_sim_node',
        output='screen',
        parameters=[params_file, {
            'use_sim_time': use_sim_time,
            'sonar_topic': '/dolphin_slam/sonar/image_raw',
            'frame_id': 'sonar_link',
            'update_rate': 2.0,
            'range_max': 20.0,
            'beam_width': 130.0
        }]
    )
    
    # DVL 仿真
    dvl_sim_node = Node(
        package='dolphin_slam',
        executable='dvl_sim_node',
        name='dvl_sim_node',
        output='screen',
        parameters=[params_file, {
            'use_sim_time': use_sim_time,
            'dvl_topic': '/dolphin_slam/dvl/data',
            'frame_id': 'dvl_link',
            'update_rate': 5.0,
            'velocity_noise_std': 0.05
        }]
    )
    
    # IMU 仿真
    imu_sim_node = Node(
        package='dolphin_slam',
        executable='imu_sim_node', 
        name='imu_sim_node',
        output='screen',
        parameters=[params_file, {
            'use_sim_time': use_sim_time,
            'imu_topic': '/dolphin_slam/imu/data',
            'frame_id': 'imu_link',
            'update_rate': 100.0,
            'gyro_noise_std': 0.001,
            'accel_noise_std': 0.01
        }]
    )
    
    # Dolphin SLAM 核心节点
    # 图像处理节点
    image_processing_node = Node(
        package='dolphin_slam',
        executable='image_processing_node',
        name='image_processing_node',
        output='screen',
        parameters=[params_file, {
            'use_sim_time': use_sim_time
        }],
        remappings=[
            ('/camera/image_raw', '/dolphin_slam/camera/image_raw'),
            ('/sonar/image_raw', '/dolphin_slam/sonar/image_raw')
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
        }]
    )
    
    # 位置细胞网络节点
    place_cell_node = Node(
        package='dolphin_slam',
        executable='place_cell_node',
        name='place_cell_node',
        output='screen',
        parameters=[params_file, {
            'use_sim_time': use_sim_time
        }]
    )
    
    # 经验地图节点
    experience_map_node = Node(
        package='dolphin_slam',
        executable='experience_map_node',
        name='experience_map_node',
        output='screen',
        parameters=[params_file, {
            'use_sim_time': use_sim_time
        }]
    )
    
    # 机器人状态节点
    robot_state_node = Node(
        package='dolphin_slam',
        executable='robot_state_node',
        name='robot_state_node',
        output='screen',
        parameters=[params_file, {
            'use_sim_time': use_sim_time
        }],
        remappings=[
            ('/dvl/data', '/dolphin_slam/dvl/data'),
            ('/imu/data', '/dolphin_slam/imu/data')
        ]
    )
    
    # 控制节点（用于仿真中的机器人控制）
    robot_controller_node = Node(
        package='dolphin_slam',
        executable='robot_controller_node',
        name='robot_controller_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'cmd_vel_topic': '/dolphin_slam/cmd_vel',
            'controller_type': 'waypoint',  # waypoint, joystick, autonomous
            'waypoints_file': PathJoinSubstitution([
                pkg_share, 'config', 'sim_waypoints.yaml'
            ])
        }]
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
    
    # Bag 录制
    bag_record = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-a', '-o', 'dolphin_slam_sim'],
        output='screen',
        condition=IfCondition(record_bag)
    )
    
    # 延迟启动 SLAM 节点（等待仿真器启动）
    delayed_slam_nodes = TimerAction(
        period=5.0,
        actions=[
            image_processing_node,
            local_view_node,
            place_cell_node,
            experience_map_node,
            robot_state_node
        ]
    )
    
    # 性能监控节点
    performance_monitor_node = Node(
        package='dolphin_slam',
        executable='performance_monitor_node',
        name='performance_monitor_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'monitor_rate': 1.0,  # Hz
            'topics_to_monitor': [
                '/dolphin_slam/camera/image_raw',
                '/dolphin_slam/sonar/image_raw',
                '/features/descriptors',
                '/place_cells/activity',
                '/experience_map/experiences'
            ]
        }]
    )
    
    # 创建启动描述
    ld = LaunchDescription()
    
    # 添加启动参数
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_world_file_cmd)
    ld.add_action(declare_robot_model_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_rviz_config_cmd)
    ld.add_action(declare_enable_rviz_cmd)
    ld.add_action(declare_record_bag_cmd)
    
    # 添加仿真器
    ld.add_action(gazebo_launch)
    
    # 添加机器人
    ld.add_action(robot_state_publisher)
    ld.add_action(spawn_robot)
    
    # 添加传感器仿真（注意：这些节点需要单独实现）
    # ld.add_action(camera_sim_node)
    # ld.add_action(sonar_sim_node)
    # ld.add_action(dvl_sim_node)
    # ld.add_action(imu_sim_node)
    
    # 添加 SLAM 节点
    ld.add_action(delayed_slam_nodes)
    
    # 添加控制器
    # ld.add_action(robot_controller_node)
    
    # 添加可视化
    ld.add_action(rviz_node)
    
    # 添加录制
    ld.add_action(bag_record)
    
    # 添加性能监控
    # ld.add_action(performance_monitor_node)
    
    return ld
