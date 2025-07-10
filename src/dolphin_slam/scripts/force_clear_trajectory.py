#!/usr/bin/env python3
"""
强制轨迹清理脚本 - 使用多种方法清空RViz历史轨迹
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import time
import subprocess
import signal
import os

class ForceTrajectoryCleaner(Node):
    """强制轨迹清理节点"""
    
    def __init__(self):
        super().__init__('force_trajectory_cleaner')
        
        # 创建轨迹发布者
        self.trajectory_pub = self.create_publisher(
            Path,
            '/dolphin_slam/trajectory',
            10
        )
        
        # 等待发布者准备就绪
        time.sleep(1)
        
        # 执行强制清理
        self.force_clear_trajectory()
        
    def force_clear_trajectory(self):
        """强制清空轨迹"""
        self.get_logger().info('🚀 开始强制轨迹清理...')
        
        # 方法1: 发布大量空消息
        self.get_logger().info('🧹 方法1: 发布大量空消息...')
        for i in range(20):
            empty_trajectory = Path()
            empty_trajectory.header.frame_id = "map"
            empty_trajectory.header.stamp = self.get_clock().now().to_msg()
            empty_trajectory.poses = []
            self.trajectory_pub.publish(empty_trajectory)
            time.sleep(0.1)
        
        # 方法2: 发布远距离轨迹然后清空
        self.get_logger().info('🧹 方法2: 远距离轨迹清理...')
        for i in range(5):
            # 远距离轨迹
            far_trajectory = Path()
            far_trajectory.header.frame_id = "map"
            far_trajectory.header.stamp = self.get_clock().now().to_msg()
            
            far_pose = PoseStamped()
            far_pose.header.frame_id = "map"
            far_pose.header.stamp = self.get_clock().now().to_msg()
            far_pose.pose.position.x = 9999.0
            far_pose.pose.position.y = 9999.0
            far_pose.pose.position.z = 9999.0
            far_trajectory.poses = [far_pose]
            
            self.trajectory_pub.publish(far_trajectory)
            time.sleep(0.1)
            
            # 立即清空
            empty_trajectory = Path()
            empty_trajectory.header.frame_id = "map"
            empty_trajectory.header.stamp = self.get_clock().now().to_msg()
            empty_trajectory.poses = []
            self.trajectory_pub.publish(empty_trajectory)
            time.sleep(0.1)
        
        # 方法3: 发布不同时间戳的空消息
        self.get_logger().info('🧹 方法3: 不同时间戳清理...')
        for i in range(10):
            empty_trajectory = Path()
            empty_trajectory.header.frame_id = "map"
            empty_trajectory.header.stamp = self.get_clock().now().to_msg()
            empty_trajectory.poses = []
            self.trajectory_pub.publish(empty_trajectory)
            time.sleep(0.2)
        
        # 方法4: 发布单点轨迹然后清空
        self.get_logger().info('🧹 方法4: 单点轨迹清理...')
        for i in range(3):
            # 单点轨迹
            single_trajectory = Path()
            single_trajectory.header.frame_id = "map"
            single_trajectory.header.stamp = self.get_clock().now().to_msg()
            
            single_pose = PoseStamped()
            single_pose.header.frame_id = "map"
            single_pose.header.stamp = self.get_clock().now().to_msg()
            single_pose.pose.position.x = 0.0
            single_pose.pose.position.y = 0.0
            single_pose.pose.position.z = 0.0
            single_trajectory.poses = [single_pose]
            
            self.trajectory_pub.publish(single_trajectory)
            time.sleep(0.1)
            
            # 清空
            empty_trajectory = Path()
            empty_trajectory.header.frame_id = "map"
            empty_trajectory.header.stamp = self.get_clock().now().to_msg()
            empty_trajectory.poses = []
            self.trajectory_pub.publish(empty_trajectory)
            time.sleep(0.1)
        
        self.get_logger().info('✅ 强制轨迹清理完成')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        cleaner = ForceTrajectoryCleaner()
        # 运行更长时间确保消息发布
        rclpy.spin_once(cleaner, timeout_sec=10.0)
        cleaner.destroy_node()
    except Exception as e:
        print(f'❌ 强制清理错误: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main() 