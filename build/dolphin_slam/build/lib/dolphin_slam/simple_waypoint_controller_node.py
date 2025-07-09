#!/usr/bin/env python3
"""
调试版航点控制器 - 彻底找出XY方向控制问题
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import numpy as np
import math

class SimpleWaypointController(Node):
    """调试版航点控制器"""
    
    def __init__(self):
        super().__init__('waypoint_controller')
        
        # 🔧 超简单的测试航点：强制XY移动
        self.waypoints = [
            [5.0, 0.0, -14.0],    # 🎯 直接向东5米，深度几乎相同
            [5.0, 5.0, -14.0],    # 🎯 向北5米
            [0.0, 5.0, -14.0],    # 🎯 向西回中心
            [0.0, 0.0, -14.0],    # 🎯 向南回起点
        ]
        
        self.current_waypoint = 0
        self.current_position = None
        self.tolerance = 1.0  # 1米容忍度
        self.max_speed = 1.0   # 降低速度便于调试
        
        # 订阅者
        self.odom_sub = self.create_subscription(
            Odometry, '/dolphin_slam/odometry', self.odometry_callback, 10)
        
        # 发布者
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 更频繁的控制和调试
        self.control_timer = self.create_timer(0.2, self.control_loop)
        self.debug_timer = self.create_timer(1.0, self.debug_status)
        
        self.get_logger().info(f'🔧 调试版航点控制器启动，目标: {self.waypoints[0]}')
        
    def odometry_callback(self, msg: Odometry):
        """处理里程计数据"""
        self.current_position = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])
        
    def control_loop(self):
        """简化的控制循环"""
        if self.current_position is None:
            return
            
        if self.current_waypoint >= len(self.waypoints):
            # 任务完成
            cmd = Twist()
            self.cmd_vel_pub.publish(cmd)
            return
            
        # 获取目标
        target = np.array(self.waypoints[self.current_waypoint])
        error = target - self.current_position
        distance = np.linalg.norm(error)
        
        # 🔧 强制调试输出
        self.get_logger().info(
            f'🎯 目标: {target} | 当前: {self.current_position} | 误差: {error} | 距离: {distance:.3f}m'
        )
        
        # 检查到达
        if distance < self.tolerance:
            self.get_logger().info(f'✅ 到达航点 {self.current_waypoint + 1}')
            self.current_waypoint += 1
            return
            
        # 🔧 超简单比例控制
        cmd = Twist()
        
        # 直接比例控制，确保XY方向有输出
        cmd.linear.x = float(error[0] * 0.5)  # X方向比例控制
        cmd.linear.y = float(error[1] * 0.5)  # Y方向比例控制  
        cmd.linear.z = float(error[2] * 0.5)  # Z方向比例控制
        
        # 限制速度
        cmd.linear.x = max(-self.max_speed, min(self.max_speed, cmd.linear.x))
        cmd.linear.y = max(-self.max_speed, min(self.max_speed, cmd.linear.y))
        cmd.linear.z = max(-self.max_speed, min(self.max_speed, cmd.linear.z))
        
        # 🔧 强制调试控制命令
        self.get_logger().info(
            f'🎮 发送命令: X={cmd.linear.x:.3f}, Y={cmd.linear.y:.3f}, Z={cmd.linear.z:.3f}'
        )
        
        # 🔧 检查控制命令是否合理
        if abs(cmd.linear.x) < 0.01 and abs(cmd.linear.y) < 0.01:
            self.get_logger().warn('⚠️ XY方向控制命令太小！')
        
        self.cmd_vel_pub.publish(cmd)
        
    def debug_status(self):
        """调试状态"""
        if self.current_position is not None and self.current_waypoint < len(self.waypoints):
            target = self.waypoints[self.current_waypoint]
            self.get_logger().info(
                f'📊 状态：航点{self.current_waypoint + 1}/{len(self.waypoints)}, '
                f'目标{target}, 当前位置{self.current_position}'
            )

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = SimpleWaypointController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
