#!/usr/bin/env python3
"""
增强版3D航点控制器 - 适用于水下环境
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, PoseStamped
import numpy as np
import math

class EnhancedWaypointController(Node):
    """增强版3D航点控制器"""
    
    def __init__(self):
        super().__init__('enhanced_waypoint_controller')
        
        # 声明参数
        self.declare_parameters(
            namespace='',
            parameters=[
                ('waypoint_tolerance', 2.0),
                ('max_linear_speed', 1.5),
                ('max_angular_speed', 1.0),
                ('kp_linear', 1.0),
                ('ki_linear', 0.1),
                ('kd_linear', 0.05),
                ('kp_angular', 2.0),
                ('ki_angular', 0.2),
                ('kd_angular', 0.1),
                ('odometry_topic', '/dolphin_slam/odometry'),
                ('cmd_vel_topic', '/cmd_vel'),
                ('enable_debug', True),
            ]
        )
        
        # 获取参数
        self.tolerance = self.get_parameter('waypoint_tolerance').value
        self.max_linear = self.get_parameter('max_linear_speed').value
        self.max_angular = self.get_parameter('max_angular_speed').value
        self.kp_lin = self.get_parameter('kp_linear').value
        self.ki_lin = self.get_parameter('ki_linear').value
        self.kd_lin = self.get_parameter('kd_linear').value
        self.kp_ang = self.get_parameter('kp_angular').value
        self.ki_ang = self.get_parameter('ki_angular').value
        self.kd_ang = self.get_parameter('kd_angular').value
        self.debug = self.get_parameter('enable_debug').value
        
        # 3D航点序列（水下搜索模式）
        self.waypoints = [
            [0.0, 0.0, -8.0],     # 下潜到工作深度
            [10.0, 0.0, -8.0],    # 东向
            [10.0, 10.0, -8.0],   # 东北角
            [-10.0, 10.0, -8.0],  # 西北角
            [-10.0, -10.0, -8.0], # 西南角
            [10.0, -10.0, -8.0],  # 东南角
            [5.0, 5.0, -12.0],    # 更深层
            [-5.0, -5.0, -12.0],  # 深层对角
            [0.0, 0.0, -5.0],     # 返回
        ]
        
        self.current_waypoint_idx = 0
        self.current_position = None
        self.last_error = np.zeros(3)
        self.error_integral = np.zeros(3)
        
        # 订阅者
        self.odom_sub = self.create_subscription(
            Odometry,
            self.get_parameter('odometry_topic').value,
            self.odometry_callback,
            10
        )
        
        # 发布者
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            self.get_parameter('cmd_vel_topic').value,
            10
        )
        
        self.path_pub = self.create_publisher(
            Path,
            '/waypoint_path',
            10
        )
        
        # 控制定时器
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        # 状态报告定时器
        self.report_timer = self.create_timer(5.0, self.report_status)
        
        # 发布路径
        self.publish_path()
        
        self.get_logger().info(f'🎯 增强版航点控制器已启动')
        self.get_logger().info(f'📍 总共{len(self.waypoints)}个航点')
        self.get_logger().info(f'🎮 当前目标: {self.waypoints[0]}')
        
    def odometry_callback(self, msg: Odometry):
        """处理里程计数据"""
        self.current_position = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])
        
    def control_loop(self):
        """主控制循环"""
        if self.current_position is None:
            return
            
        if self.current_waypoint_idx >= len(self.waypoints):
            # 任务完成，停止
            self.publish_zero_velocity()
            return
            
        # 获取当前目标航点
        target = np.array(self.waypoints[self.current_waypoint_idx])
        
        # 计算到目标的距离和误差
        error = target - self.current_position
        distance = np.linalg.norm(error)
        
        # 检查是否到达航点
        if distance < self.tolerance:
            self.get_logger().info(f'✅ 到达航点{self.current_waypoint_idx + 1}/{len(self.waypoints)}: {target}')
            self.current_waypoint_idx += 1
            self.error_integral = np.zeros(3)  # 重置积分
            
            if self.current_waypoint_idx < len(self.waypoints):
                next_target = self.waypoints[self.current_waypoint_idx]
                self.get_logger().info(f'🎯 下一个目标: {next_target}')
            return
            
        # PID控制计算
        error_derivative = error - self.last_error
        self.error_integral += error * 0.1  # dt = 0.1s
        
        # PID输出
        pid_output = (self.kp_lin * error + 
                     self.ki_lin * self.error_integral + 
                     self.kd_lin * error_derivative)
        
        # 构造控制命令
        cmd = Twist()
        
        # 线速度（XYZ）
        cmd.linear.x = np.clip(pid_output[0], -self.max_linear, self.max_linear)
        cmd.linear.y = np.clip(pid_output[1], -self.max_linear, self.max_linear)
        cmd.linear.z = np.clip(pid_output[2], -self.max_linear, self.max_linear)
        
        # 角速度（简单航向控制）
        target_yaw = math.atan2(error[1], error[0])
        yaw_error = target_yaw  # 简化，假设当前yaw=0
        cmd.angular.z = np.clip(self.kp_ang * yaw_error, -self.max_angular, self.max_angular)
        
        # 发布控制命令
        self.cmd_vel_pub.publish(cmd)
        
        # 更新
        self.last_error = error
        
    def publish_zero_velocity(self):
        """发布零速度命令"""
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
        
    def publish_path(self):
        """发布路径可视化"""
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()
        
        for waypoint in self.waypoints:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = float(waypoint[0])
            pose.pose.position.y = float(waypoint[1])
            pose.pose.position.z = float(waypoint[2])
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
            
        self.path_pub.publish(path)
        
    def report_status(self):
        """状态报告"""
        if self.current_position is not None and self.current_waypoint_idx < len(self.waypoints):
            target = self.waypoints[self.current_waypoint_idx]
            distance = np.linalg.norm(np.array(target) - self.current_position)
            
            self.get_logger().info(
                f'🚁 航点{self.current_waypoint_idx + 1}/{len(self.waypoints)}: '
                f'目标{target}, 当前位置({self.current_position[0]:.2f}, '
                f'{self.current_position[1]:.2f}, {self.current_position[2]:.2f}), '
                f'距离{distance:.2f}m'
            )

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
