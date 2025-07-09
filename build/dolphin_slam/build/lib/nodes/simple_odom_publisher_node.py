#!/usr/bin/env python3
"""
Dolphin SLAM - 智能里程计发布器
优先使用内置里程计，model_states作为备用
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from gazebo_msgs.msg import ModelStates
from tf2_ros import TransformBroadcaster
import numpy as np


class SimpleOdomPublisher(Node):
    """智能里程计发布器"""
    
    def __init__(self):
        super().__init__('simple_odom_publisher')
        
        # 参数
        self.declare_parameter('robot_name', 'auv_robot')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('publish_tf', False)
        
        self.robot_name = self.get_parameter('robot_name').value
        self.base_frame = self.get_parameter('base_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.publish_tf = self.get_parameter('publish_tf').value
        
        # 数据源状态
        self.builtin_count = 0
        self.model_state_count = 0
        self.publish_count = 0
        self.last_builtin_time = 0.0
        self.using_builtin = False
        
        # 发布到SLAM系统
        self.odom_pub = self.create_publisher(Odometry, '/dolphin_slam/odometry', 10)
        
        # TF广播器
        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)
        
        # 订阅内置里程计（优先）
        self.builtin_odom_sub = self.create_subscription(
            Odometry, '/odom', self.builtin_odom_callback, 10)
        
        # 订阅模型状态（备用）- 稍后创建，给内置里程计优先权
        self.create_timer(2.0, self.setup_model_states_subscription)
        
        # 调试定时器
        self.debug_timer = self.create_timer(5.0, self.debug_status)
        
        self.get_logger().info(f'🤖 智能里程计发布器已启动，机器人: {self.robot_name}')
        self.get_logger().info(f'📡 发布话题: /dolphin_slam/odometry')
        self.get_logger().info(f'⚡ 优先使用内置里程计 /odom')
        
    def setup_model_states_subscription(self):
        """延迟设置model_states订阅"""
        if not hasattr(self, 'model_states_sub'):
            self.model_states_sub = self.create_subscription(
                ModelStates, '/gazebo/model_states', self.model_states_callback, 10)
            self.get_logger().info(f'🔄 已设置备用数据源: /gazebo/model_states')

    def builtin_odom_callback(self, msg):
        """处理内置里程计数据（优先使用）"""
        self.builtin_count += 1
        self.last_builtin_time = self.get_clock().now().nanoseconds / 1e9
        self.using_builtin = True
        
        # 转发到SLAM系统
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame
        
        # 复制位置和速度数据
        odom_msg.pose = msg.pose
        odom_msg.twist = msg.twist
        
        self.odom_pub.publish(odom_msg)
        self.publish_count += 1
        
        # 每20次发布打印一次位置信息
        if self.publish_count % 20 == 1:
            pos = msg.pose.pose.position
            vel = msg.twist.twist.linear
            self.get_logger().info(
                f'📍 内置里程计 #{self.publish_count}: '
                f'位置=({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f}), '
                f'速度=({vel.x:.2f}, {vel.y:.2f}, {vel.z:.2f})'
            )

    def model_states_callback(self, msg):
        """处理模型状态（仅在内置里程计不可用时使用）"""
        self.model_state_count += 1
        
        # 如果最近5秒内有内置里程计数据，就不使用模型状态
        current_time = self.get_clock().now().nanoseconds / 1e9
        if (current_time - self.last_builtin_time) < 5.0 and self.builtin_count > 0:
            return
            
        # 如果没有内置里程计数据，使用模型状态作为备用
        self.using_builtin = False
        
        try:
            if self.robot_name not in msg.name:
                if self.model_state_count % 50 == 1:
                    self.get_logger().warn(f'机器人 {self.robot_name} 不在模型列表中: {msg.name}')
                return
                
            robot_index = msg.name.index(self.robot_name)
            robot_pose = msg.pose[robot_index]
            robot_twist = msg.twist[robot_index]
            
            # 创建里程计消息
            odom_msg = Odometry()
            odom_msg.header.stamp = self.get_clock().now().to_msg()
            odom_msg.header.frame_id = self.odom_frame
            odom_msg.child_frame_id = self.base_frame
            
            odom_msg.pose.pose = robot_pose
            odom_msg.twist.twist = robot_twist
            
            # 设置协方差
            odom_msg.pose.covariance = [0.1] * 36
            odom_msg.twist.covariance = [0.05] * 36
            
            self.odom_pub.publish(odom_msg)
            self.publish_count += 1
            
            # 每20次发布打印一次
            if self.publish_count % 20 == 1:
                pos = robot_pose.position
                self.get_logger().info(
                    f'📍 模型状态备用 #{self.publish_count}: '
                    f'位置=({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f})'
                )
                
        except Exception as e:
            self.get_logger().error(f'处理模型状态时出错: {e}')

    def debug_status(self):
        """状态报告"""
        current_time = self.get_clock().now().nanoseconds / 1e9
        time_since_builtin = current_time - self.last_builtin_time
        
        data_source = "内置里程计" if self.using_builtin else "模型状态"
        builtin_status = "✅" if time_since_builtin < 5.0 else "❌"
        
        self.get_logger().info(
            f'📊 数据源={data_source} {builtin_status}, '
            f'内置={self.builtin_count}, 模型={self.model_state_count}, '
            f'发布={self.publish_count}条到SLAM'
        )


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = SimpleOdomPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"节点运行出错: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
