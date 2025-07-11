#!/usr/bin/env python3
"""
Dolphin SLAM - 经验地图 ROS2 节点 (修复版)
解决轨迹重复日志问题
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray, Marker
import numpy as np
from typing import Optional

class ExperienceMapNode(Node):
    """经验地图 ROS2 节点 (修复版)"""
    
    def __init__(self):
        super().__init__('experience_map_node')
        
        # 声明参数
        self.declare_parameters(
            namespace='',
            parameters=[
                ('odometry_topic', '/dolphin_slam/odometry'),
                ('trajectory_topic', '/dolphin_slam/trajectory'),
                ('max_trajectory_length', 2000),  # 增加到2000
                ('cleanup_threshold', 1500),      # 1500时开始清理
                ('log_interval', 100),             # 每100个点记录一次日志
            ]
        )
        
        # 状态变量
        self.current_odometry: Optional[Odometry] = None
        self.experience_count = 0
        self.trajectory_poses = []
        self.last_log_count = 0
        self.startup_complete = False  # 🔧 添加启动完成标志
        
        # 获取参数
        self.max_trajectory_length = self.get_parameter('max_trajectory_length').value
        self.cleanup_threshold = self.get_parameter('cleanup_threshold').value
        self.log_interval = self.get_parameter('log_interval').value
        
        # 订阅者
        self.pc_activity_sub = self.create_subscription(
            Float32MultiArray,
            '/place_cells/activity',
            self.place_cell_callback,
            10
        )
        
        self.visual_match_sub = self.create_subscription(
            Float32MultiArray,
            '/local_view/matches',
            self.visual_match_callback,
            10
        )
        
        self.odometry_sub = self.create_subscription(
            Odometry,
            self.get_parameter('odometry_topic').value,
            self.odometry_callback,
            10
        )
        
        # 发布者
        self.experience_pub = self.create_publisher(
            Float32MultiArray,
            '/experience_map/experiences',
            10
        )
        
        self.map_markers_pub = self.create_publisher(
            MarkerArray,
            '/experience_map/markers',
            10
        )
        
        self.trajectory_pub = self.create_publisher(
            Path,
            self.get_parameter('trajectory_topic').value,
            10
        )
        
        # 定时器
        self.update_timer = self.create_timer(0.2, self.update_map)
        self.viz_timer = self.create_timer(1.0, self.publish_visualizations)
        self.startup_timer = self.create_timer(0.1, self.startup_check)  # 🔧 添加启动检查定时器
        
        self.get_logger().info('经验地图节点已启动 - 修复版')
        
    def startup_check(self):
        """🔧 启动检查 - 清空历史轨迹数据"""
        if not self.startup_complete:
            # 清空轨迹数据
            self.trajectory_poses = []
            
            # 🧹 发布空轨迹消息清空RViz缓冲区
            empty_trajectory = Path()
            empty_trajectory.header.frame_id = "map"
            empty_trajectory.header.stamp = self.get_clock().now().to_msg()
            empty_trajectory.poses = []  # 空轨迹
            self.trajectory_pub.publish(empty_trajectory)
            
            self.get_logger().info('🧹 已清空历史轨迹数据并发布空轨迹消息，开始新的仿真')
            self.startup_complete = True
            # 停止启动检查定时器
            self.startup_timer.cancel()
        
    def odometry_callback(self, msg: Odometry):
        """处理里程计数据 - 智能轨迹管理"""
        self.current_odometry = msg
        
        # 更新轨迹
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose = msg.pose.pose
        self.trajectory_poses.append(pose_stamped)
        
        # 智能轨迹管理 - 避免频繁重复日志
        trajectory_count = len(self.trajectory_poses)
        
        # 智能清理：当超过阈值时，保留最近的点
        if trajectory_count > self.max_trajectory_length:
            # 保留最近的cleanup_threshold个点
            self.trajectory_poses = self.trajectory_poses[-self.cleanup_threshold:]
            
            # 只在首次清理时记录日志
            if trajectory_count == self.max_trajectory_length + 1:
                self.get_logger().info(
                    f'轨迹点数达到上限({self.max_trajectory_length})，已清理至{self.cleanup_threshold}个点'
                )
        
        # 定期记录轨迹状态，但不要太频繁
        elif trajectory_count % (self.log_interval * 5) == 0:  # 大幅减少轨迹点数日志频率
            self.get_logger().info(f'轨迹点数: {trajectory_count}')
        
    def place_cell_callback(self, msg: Float32MultiArray):
        """处理位置细胞活动"""
        if len(msg.data) > 0:
            max_activity = max(msg.data)
            active_count = sum(1 for x in msg.data if x > 0.1)
            self.get_logger().debug(f'位置细胞活动: 最大={max_activity:.3f}, 活跃={active_count}')
        
    def visual_match_callback(self, msg: Float32MultiArray):
        """处理视觉匹配结果"""
        if len(msg.data) > 0:
            similarity = msg.data[0]
            self.get_logger().debug(f'视觉匹配: {similarity:.3f}')
        
    def update_map(self):
        """更新经验地图"""
        if self.current_odometry is None:
            return
            
        # 简单的经验创建逻辑
        self.experience_count += 1
        
        # 只在特定间隔记录经验数
        if self.experience_count % (self.log_interval * 10) == 0:  # 大幅减少经验计数日志频率
            self.get_logger().info(f'经验计数: {self.experience_count}')
        
    def publish_visualizations(self):
        """发布可视化信息"""
        try:
            # 发布轨迹
            if len(self.trajectory_poses) > 1:
                trajectory = Path()
                trajectory.header.frame_id = "map"
                trajectory.header.stamp = self.get_clock().now().to_msg()
                trajectory.poses = self.trajectory_poses
                self.trajectory_pub.publish(trajectory)
                
        except Exception as e:
            self.get_logger().debug(f'可视化错误: {e}')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = ExperienceMapNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n🛑 用户中断")
    except Exception as e:
        print(f'❌ 节点错误: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
