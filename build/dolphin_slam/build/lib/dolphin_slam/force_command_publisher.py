#!/usr/bin/env python3
"""
力命令发布器 - 将cmd_vel转换为力命令
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Wrench
import numpy as np

class ForceCommandPublisher(Node):
    """力命令发布器 - 将cmd_vel转换为力命令"""
    
    def __init__(self):
        super().__init__('force_command_publisher')
        
        # 订阅cmd_vel
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        # 发布力命令
        self.force_pub = self.create_publisher(
            Wrench, '/gazebo_ros_force', 10)
        
        # 力转换参数 - 使用更温和的参数避免振荡
        self.force_scale = 200.0   # 降低力缩放因子
        self.torque_scale = 100.0  # 降低扭矩缩放因子
        
        self.get_logger().info('🔧 力命令发布器已启动')
        self.get_logger().info(f'⚙️ 力缩放: {self.force_scale}, 扭矩缩放: {self.torque_scale}')
        
    def cmd_vel_callback(self, msg: Twist):
        """将cmd_vel转换为力命令"""
        force_cmd = Wrench()
        
        # 线速度转换为力
        force_cmd.force.x = float(msg.linear.x * self.force_scale)
        force_cmd.force.y = float(msg.linear.y * self.force_scale)
        force_cmd.force.z = float(msg.linear.z * self.force_scale)
        
        # 角速度转换为扭矩
        force_cmd.torque.x = float(msg.angular.x * self.torque_scale)
        force_cmd.torque.y = float(msg.angular.y * self.torque_scale)
        force_cmd.torque.z = float(msg.angular.z * self.torque_scale)
        
        # 发布力命令
        self.force_pub.publish(force_cmd)
        
        # 调试输出 - 大幅减少频率
        if abs(force_cmd.force.x) > 50 or abs(force_cmd.force.y) > 50 or abs(force_cmd.force.z) > 50:
            self.get_logger().info(
                f'🎮 力命令: Fx={force_cmd.force.x:.1f}, Fy={force_cmd.force.y:.1f}, Fz={force_cmd.force.z:.1f}, '
                f'Tz={force_cmd.torque.z:.1f}'
            )

def main(args=None):
    rclpy.init(args=args)
    node = ForceCommandPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 