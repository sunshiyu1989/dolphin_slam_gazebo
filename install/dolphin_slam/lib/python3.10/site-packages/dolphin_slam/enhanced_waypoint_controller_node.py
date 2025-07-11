#!/usr/bin/env python3
"""
简化版航点控制器 - 回到工作的版本
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import numpy as np
import math
from typing import List, Optional

class EnhancedWaypointController(Node):
    """简化版航点控制器 - 回到工作的版本"""
    
    def __init__(self):
        super().__init__('enhanced_waypoint_controller')
        
        # 简化的航点序列 - 浅水探索
        self.waypoints = [
            [5.0, 5.0, -3.0],     # 第一个目标：东北方向5米，浅水
            [10.0, 0.0, -4.0],    # 向东10米，中等深度
            [10.0, 10.0, -5.0],   # 向北10米，稍深
            [0.0, 10.0, -4.0],    # 向西10米，中等深度
            [-10.0, 10.0, -3.0],  # 西北10米，浅水
            [-10.0, 0.0, -4.0],   # 向西10米，中等深度
            [-10.0, -10.0, -5.0], # 西南10米，稍深
            [0.0, -10.0, -4.0],   # 向南10米，中等深度
            [10.0, -10.0, -3.0],  # 东南10米，浅水
            [0.0, 0.0, -3.0]      # 最终回到起点，浅水
        ]
        
        self.current_waypoint = 0
        self.current_position = None
        self.tolerance = 1.0  # 航点容差
        self.max_speed = 1.0   # 最大速度
        self.min_speed = 0.2   # 最小速度阈值
        
        # 简化的控制参数
        self.kp_linear = 0.5   # 线性比例增益
        self.kp_angular = 0.3  # 角速度比例增益
        self.max_angular_speed = 0.3  # 最大角速度
        
        # 状态跟踪
        self.waypoint_reached_count = 0
        self.total_distance_traveled = 0.0
        self.last_position = None
        

        
        # 启动控制
        self.startup_complete = False
        self.startup_timer = 0
        self.startup_delay = 30  # 启动延迟
        
        # 订阅者
        self.odom_sub = self.create_subscription(
            Odometry, '/dolphin_slam/odometry', self.odometry_callback, 10)
        
        # 发布者
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 控制频率
        self.control_timer = self.create_timer(0.1, self.control_loop)  # 10Hz控制频率
        self.debug_timer = self.create_timer(15.0, self.debug_status)  # 大幅减少调试频率
        
        self.get_logger().info(f'🔧 简化版航点控制器启动')
        self.get_logger().info(f'🎯 目标航点: {self.waypoints[0]}')
        self.get_logger().info(f'⚙️ 控制参数: Kp={self.kp_linear}, 最大速度={self.max_speed}m/s')
        
    def odometry_callback(self, msg: Odometry):
        """处理里程计数据"""
        self.current_position = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])
        
        # 计算总移动距离
        if self.last_position is not None:
            distance = np.linalg.norm(self.current_position - self.last_position)
            self.total_distance_traveled += distance
            
        self.last_position = self.current_position.copy()
        
    def control_loop(self):
        """简化的控制循环"""
        if self.current_position is None:
            return
            
        # 启动稳定控制
        if not self.startup_complete:
            self.startup_timer += 1
            if self.startup_timer < self.startup_delay:
                # 启动阶段，只发布零速度命令
                cmd = Twist()
                self.cmd_vel_pub.publish(cmd)
                return
            else:
                self.startup_complete = True
                self.get_logger().info('🚀 启动稳定完成，开始导航')
            
        if self.current_waypoint >= len(self.waypoints):
            # 任务完成，停止机器人
            cmd = Twist()
            self.cmd_vel_pub.publish(cmd)
            return
            
        # 获取目标航点
        target = np.array(self.waypoints[self.current_waypoint])
        error = target - self.current_position
        distance = np.linalg.norm(error)
        
        # 检查是否到达航点
        if distance < self.tolerance:
            self.get_logger().info(f'✅ 到达航点 {self.current_waypoint + 1}: {target}')
            self.current_waypoint += 1
            self.waypoint_reached_count += 1
            
            if self.current_waypoint < len(self.waypoints):
                next_target = self.waypoints[self.current_waypoint]
                self.get_logger().info(f'🎯 下一个目标: {next_target}')
            return
            
        # 简化的控制策略
        cmd = Twist()
        
        # 计算目标角度
        target_angle = math.atan2(error[1], error[0])
        
        # 简化的角度控制 - 假设机器人朝向X轴正方向
        angle_error = target_angle
        
        # 处理角度环绕
        while angle_error > math.pi:
            angle_error -= 2 * math.pi
        while angle_error < -math.pi:
            angle_error += 2 * math.pi
        
        # 线性速度控制
        forward_speed = float(distance * self.kp_linear)
        forward_speed = max(0.0, min(self.max_speed, forward_speed))
        
        # 确保最小速度
        if forward_speed > 0.1 and forward_speed < self.min_speed:
            forward_speed = self.min_speed
        
        cmd.linear.x = forward_speed
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0  # 简化Z方向控制
        
        # 角速度控制
        cmd.angular.z = float(angle_error * self.kp_angular)
        cmd.angular.z = max(-self.max_angular_speed, min(self.max_angular_speed, cmd.angular.z))
        
        # 发布控制命令
        self.cmd_vel_pub.publish(cmd)
        
    def debug_status(self):
        """调试状态报告"""
        if self.current_position is not None and self.current_waypoint < len(self.waypoints):
            target = self.waypoints[self.current_waypoint]
            error = np.array(target) - self.current_position
            distance = np.linalg.norm(error)
            
            self.get_logger().info(
                f'📊 控制器状态: 航点{self.current_waypoint + 1}/{len(self.waypoints)}, '
                f'目标{target}, 当前位置{self.current_position}, '
                f'距离{distance:.3f}m, 总移动距离{self.total_distance_traveled:.2f}m'
            )
            
    def get_mission_progress(self) -> dict:
        """获取任务进度"""
        return {
            'current_waypoint': self.current_waypoint,
            'total_waypoints': len(self.waypoints),
            'waypoints_reached': self.waypoint_reached_count,
            'total_distance': self.total_distance_traveled,
            'mission_complete': self.current_waypoint >= len(self.waypoints)
        }

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = EnhancedWaypointController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main() 