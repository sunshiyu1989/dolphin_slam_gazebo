#!/usr/bin/env python3
"""
修复版 simple_odom_publisher - 确保正确的数据流
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import numpy as np


def quaternion_to_euler(x, y, z, w):
    """四元数转欧拉角"""
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = np.copysign(np.pi / 2, sinp)
    else:
        pitch = np.arcsin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class FixedSimpleOdomPublisher(Node):
    """修复版简单里程计发布器"""
    
    def __init__(self):
        super().__init__('fixed_simple_odom_publisher')
        
        # 参数
        self.declare_parameter('robot_name', 'auv_robot')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('debug_mode', True)
        
        self.robot_name = self.get_parameter('robot_name').value
        self.base_frame = self.get_parameter('base_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.debug_mode = self.get_parameter('debug_mode').value
        
        # 数据计数器
        self.message_count = 0
        self.publish_count = 0
        
        # 订阅 Gazebo 模型状态
        self.model_states_sub = self.create_subscription(
            ModelStates,
            '/gazebo/model_states',
            self.model_states_callback,
            10
        )
        
        # 发布里程计
        self.odom_pub = self.create_publisher(Odometry, '/robot/odometry', 10)
        
        # TF 广播器
        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)
        
        # 调试定时器
        if self.debug_mode:
            self.debug_timer = self.create_timer(5.0, self.debug_status)
        
        self.get_logger().info(f'🔧 修复版里程计发布器已启动')
        self.get_logger().info(f'   机器人名称: {self.robot_name}')
        self.get_logger().info(f'   订阅话题: /gazebo/model_states')
        self.get_logger().info(f'   发布话题: /robot/odometry')

    def model_states_callback(self, msg):
        """处理 Gazebo 模型状态消息"""
        self.message_count += 1
        
        try:
            # 查找目标机器人
            if self.robot_name not in msg.name:
                if self.debug_mode and self.message_count % 20 == 1:  # 每20次消息打印一次
                    self.get_logger().warn(f'机器人 {self.robot_name} 不在模型列表中')
                    self.get_logger().info(f'可用模型: {msg.name}')
                return
                
            robot_index = msg.name.index(self.robot_name)
            robot_pose = msg.pose[robot_index]
            robot_twist = msg.twist[robot_index]
            
            # 创建里程计消息
            odom_msg = Odometry()
            odom_msg.header.stamp = self.get_clock().now().to_msg()
            odom_msg.header.frame_id = self.odom_frame
            odom_msg.child_frame_id = self.base_frame
            
            # 复制位置和姿态
            odom_msg.pose.pose = robot_pose
            odom_msg.twist.twist = robot_twist
            
            # 设置协方差
            odom_msg.pose.covariance = [0.1] * 36
            odom_msg.twist.covariance = [0.05] * 36
            
            # 发布里程计
            self.odom_pub.publish(odom_msg)
            self.publish_count += 1
            
            # 发布 TF 变换
            if self.publish_tf:
                self.publish_transform(robot_pose, odom_msg.header.stamp)
                
            # 调试信息
            if self.debug_mode and self.publish_count % 50 == 1:  # 每50次发布打印一次
                pos = robot_pose.position
                self.get_logger().info(
                    f'📍 发布里程计 #{self.publish_count}: '
                    f'位置=({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f})'
                )
                
        except Exception as e:
            self.get_logger().error(f'处理模型状态时出错: {e}')

    def publish_transform(self, pose, timestamp):
        """发布 TF 变换"""
        try:
            t = TransformStamped()
            t.header.stamp = timestamp
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            
            t.transform.translation.x = pose.position.x
            t.transform.translation.y = pose.position.y
            t.transform.translation.z = pose.position.z
            t.transform.rotation = pose.orientation
            
            self.tf_broadcaster.sendTransform(t)
            
        except Exception as e:
            self.get_logger().warn(f'发布 TF 变换时出错: {e}')

    def debug_status(self):
        """调试状态信息"""
        if self.debug_mode:
            self.get_logger().info(
                f'📊 状态报告: 接收={self.message_count}条, 发布={self.publish_count}条里程计消息'
            )


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = FixedSimpleOdomPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"节点运行出错: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
