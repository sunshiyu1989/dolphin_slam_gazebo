#!/usr/bin/env python3
"""
轨迹清理脚本 - 清空RViz历史轨迹 (增强版)
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import time
import subprocess
import signal
import os

class TrajectoryCleaner(Node):
    """轨迹清理节点 (增强版)"""
    
    def __init__(self):
        super().__init__('trajectory_cleaner')
        
        # 创建轨迹发布者
        self.trajectory_pub = self.create_publisher(
            Path,
            '/dolphin_slam/trajectory',
            10
        )
        
        # 等待发布者准备就绪
        time.sleep(1)
        
        # 检查RViz是否运行
        self.wait_for_rviz()
        
        # 发布空轨迹消息
        self.clear_trajectory()
        
    def wait_for_rviz(self):
        """等待RViz启动"""
        self.get_logger().info('⏳ 等待RViz启动...')
        max_wait = 30  # 最多等待30秒
        wait_count = 0
        
        while wait_count < max_wait:
            # 检查RViz进程
            try:
                result = subprocess.run(['pgrep', 'rviz2'], capture_output=True, text=True)
                if result.returncode == 0:
                    self.get_logger().info('✅ RViz已启动，开始清理轨迹')
                    time.sleep(2)  # 给RViz一些时间完全启动
                    return
            except:
                pass
            
            time.sleep(1)
            wait_count += 1
            
        self.get_logger().warning('⚠️ RViz可能未启动，但仍尝试清理轨迹')
        
    def clear_trajectory(self):
        """清空轨迹 (增强版)"""
        self.get_logger().info('🧹 开始清理历史轨迹...')
        
        # 方法1: 发布空轨迹消息
        empty_trajectory = Path()
        empty_trajectory.header.frame_id = "map"
        empty_trajectory.header.stamp = self.get_clock().now().to_msg()
        empty_trajectory.poses = []  # 空轨迹
        
        # 连续发布多次确保清空
        for i in range(10):  # 增加到10次
            self.trajectory_pub.publish(empty_trajectory)
            self.get_logger().info(f'🧹 发布空轨迹消息 #{i+1}/10')
            time.sleep(0.2)  # 增加间隔
        
        # 方法2: 发布特殊标记的轨迹消息
        self.get_logger().info('🧹 发布特殊清理消息...')
        for i in range(5):
            # 创建一个远距离的轨迹点，然后立即清空
            far_trajectory = Path()
            far_trajectory.header.frame_id = "map"
            far_trajectory.header.stamp = self.get_clock().now().to_msg()
            
            # 添加一个远距离点
            far_pose = PoseStamped()
            far_pose.header.frame_id = "map"
            far_pose.header.stamp = self.get_clock().now().to_msg()
            far_pose.pose.position.x = 1000.0
            far_pose.pose.position.y = 1000.0
            far_pose.pose.position.z = 1000.0
            far_trajectory.poses = [far_pose]
            
            self.trajectory_pub.publish(far_trajectory)
            time.sleep(0.1)
            
            # 立即发布空消息
            self.trajectory_pub.publish(empty_trajectory)
            time.sleep(0.1)
        
        self.get_logger().info('✅ 轨迹清理完成')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        cleaner = TrajectoryCleaner()
        # 运行更长时间确保消息发布
        rclpy.spin_once(cleaner, timeout_sec=5.0)
        cleaner.destroy_node()
    except Exception as e:
        print(f'❌ 清理错误: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main() 