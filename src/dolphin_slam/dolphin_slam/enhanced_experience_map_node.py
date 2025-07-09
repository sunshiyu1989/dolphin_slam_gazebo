#!/usr/bin/env python3
"""
Dolphin SLAM - 增强经验地图 ROS2 节点
修复轨迹发布和话题订阅问题
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped
import numpy as np
from typing import Optional

class EnhancedExperienceMapNode(Node):
    """增强经验地图 ROS2 节点"""
    
    def __init__(self):
        super().__init__('experience_map_node')
        
        # 状态变量
        self.current_odometry: Optional[Odometry] = None
        self.trajectory_poses = []
        self.place_cell_data = None
        self.visual_match_data = None
        
        # 订阅者 - 使用统一的话题名称
        self.odometry_sub = self.create_subscription(
            Odometry,
            '/dolphin_slam/odometry',
            self.odometry_callback,
            10
        )
        
        self.place_cell_sub = self.create_subscription(
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
        
        # 发布者
        self.trajectory_pub = self.create_publisher(
            Path,
            '/dolphin_slam/trajectory',
            10
        )
        
        self.experience_pub = self.create_publisher(
            Float32MultiArray,
            '/experience_map/experiences',
            10
        )
        
        # 定时器
        self.update_timer = self.create_timer(0.1, self.update_and_publish)
        
        self.get_logger().info('增强经验地图节点已启动 - 修复版本')
        
    def odometry_callback(self, msg: Odometry):
        """处理里程计数据"""
        self.current_odometry = msg
        
        # 创建轨迹点
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose = msg.pose.pose
        
        self.trajectory_poses.append(pose_stamped)
        
        # 限制轨迹长度避免内存溢出
        if len(self.trajectory_poses) > 2000:
            self.trajectory_poses = self.trajectory_poses[-1000:]
        
    def place_cell_callback(self, msg: Float32MultiArray):
        """处理位置细胞活动"""
        self.place_cell_data = msg.data
        
    def visual_match_callback(self, msg: Float32MultiArray):
        """处理视觉匹配数据"""
        self.visual_match_data = msg.data
        
    def update_and_publish(self):
        """更新并发布轨迹和经验数据"""
        # 发布轨迹
        if self.trajectory_poses:
            path_msg = Path()
            path_msg.header.stamp = self.get_clock().now().to_msg()
            path_msg.header.frame_id = "map"
            path_msg.poses = self.trajectory_poses.copy()
            
            self.trajectory_pub.publish(path_msg)
            
        # 发布经验数据
        if self.current_odometry:
            experience_msg = Float32MultiArray()
            
            # 简单的经验数据（位置 + 活动强度）
            data = [
                self.current_odometry.pose.pose.position.x,
                self.current_odometry.pose.pose.position.y,
                self.current_odometry.pose.pose.position.z
            ]
            
            # 添加位置细胞活动（如果有的话）
            if self.place_cell_data:
                max_activity = max(self.place_cell_data) if self.place_cell_data else 0.0
                data.append(max_activity)
            else:
                data.append(0.0)
                
            # 添加视觉匹配强度
            if self.visual_match_data:
                match_strength = sum(self.visual_match_data) if self.visual_match_data else 0.0
                data.append(match_strength)
            else:
                data.append(0.0)
            
            experience_msg.data = data
            self.experience_pub.publish(experience_msg)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = EnhancedExperienceMapNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
