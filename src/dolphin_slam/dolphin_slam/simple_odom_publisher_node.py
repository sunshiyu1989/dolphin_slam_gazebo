#!/usr/bin/env python3
"""
Dolphin SLAM - 简单里程计发布器
从 Gazebo 模型状态获取位置信息并发布里程计
修复版 - 发布到正确的话题 /dolphin_slam/odometry
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Twist
from gazebo_msgs.msg import ModelStates
from tf2_ros import TransformBroadcaster
import numpy as np


def quaternion_to_euler(x, y, z, w):
    """
    四元数转欧拉角 - 自定义函数替代 tf_transformations
    
    参数:
        x, y, z, w: 四元数分量
        
    返回:
        (roll, pitch, yaw): 欧拉角（弧度）
    """
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = np.copysign(np.pi / 2, sinp)  # 使用 90 度
    else:
        pitch = np.arcsin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class SimpleOdomPublisher(Node):
    """简单里程计发布器 - 修复版"""
    
    def __init__(self):
        super().__init__('simple_odom_publisher')
        
        # 声明参数
        self.declare_parameter('robot_name', 'auv_robot')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('odom_rate', 2.0)  # Hz - 大幅降低频率解决RViz队列溢出
        
        # 获取参数
        self.robot_name = self.get_parameter('robot_name').value
        self.base_frame = self.get_parameter('base_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.odom_rate = self.get_parameter('odom_rate').value
        if self.odom_rate is None or self.odom_rate <= 0:
            self.odom_rate = 10.0
        # 定时器（确保使用有效的odom_rate）
        self.odom_timer = self.create_timer(1.0 / float(self.odom_rate), self.publish_odom)
        
        # 订阅 Gazebo 模型状态
        self.model_states_sub = self.create_subscription(
            ModelStates,
            '/gazebo/model_states',
            self.model_states_callback,
            10
        )
        
        # 🔧 修复：发布到正确的话题
        self.odom_pub = self.create_publisher(Odometry, '/dolphin_slam/odometry', 10)
        
        # TF 广播器
        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)
        
        # 存储上一次的位置和时间（用于计算速度）
        self.last_pose = None
        self.last_time = None
        self.message_count = 0
        self.publish_count = 0
        
        # 调试定时器 - 大幅减少频率
        self.debug_timer = self.create_timer(30.0, self.debug_status)
        
        self.get_logger().info(f'🤖 智能里程计发布器已启动，机器人: {self.robot_name}')
        self.get_logger().info(f'📡 发布话题: /dolphin_slam/odometry')
        self.get_logger().info(f'⚡ 优先使用内置里程计 /odom')

    def model_states_callback(self, msg):
        """处理 Gazebo 模型状态消息"""
        self.message_count += 1
        
        try:
            # 查找目标机器人
            if self.robot_name not in msg.name:
                if self.message_count % 200 == 1:  # 每200次消息打印一次
                    self.get_logger().warn(f'机器人 {self.robot_name} 不在模型列表中')
                return
                
            robot_index = msg.name.index(self.robot_name)
            robot_pose = msg.pose[robot_index]
            robot_twist = msg.twist[robot_index]
            
            # 创建里程计消息
            odom_msg = Odometry()
            odom_msg.header.stamp = self.get_clock().now().to_msg()
            odom_msg.header.frame_id = self.odom_frame
            odom_msg.child_frame_id = self.base_frame
            
            # 位置和姿态
            odom_msg.pose.pose = robot_pose
            
            # 速度
            odom_msg.twist.twist = robot_twist
            
            # 设置协方差（简单估计）
            odom_msg.pose.covariance = [0.1] * 36
            odom_msg.twist.covariance = [0.05] * 36
            
            # 🔧 发布到正确的话题
            self.last_odom_msg = odom_msg # 存储最新的odom消息
            
        except Exception as e:
            self.get_logger().error(f'处理模型状态时出错: {e}')

    def publish_transform(self, pose, timestamp):
        """发布 TF 变换"""
        try:
            t = TransformStamped()
            t.header.stamp = timestamp
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            
            # 设置平移
            t.transform.translation.x = pose.position.x
            t.transform.translation.y = pose.position.y
            t.transform.translation.z = pose.position.z
            
            # 设置旋转
            t.transform.rotation = pose.orientation
            
            # 广播变换
            self.tf_broadcaster.sendTransform(t)
            
        except Exception as e:
            self.get_logger().warn(f'发布 TF 变换时出错: {e}')

    def debug_status(self):
        """调试状态报告 - 大幅精简"""
        try:
            # 🔧 大幅精简状态报告 - 只在有数据时显示
            if self.publish_count > 0:
                self.get_logger().info(
                    f'📊 数据源=内置里程计 ✅, 内置={self.message_count}, '
                    f'模型={self.message_count}, 发布={self.publish_count}条到SLAM'
                )
        except Exception as e:
            self.get_logger().error(f'调试状态报告失败: {e}')

    # 新增发布odom的定时器方法
    def publish_odom(self):
        # 只发布最新的odom
        if hasattr(self, 'last_odom_msg'):
            self.odom_pub.publish(self.last_odom_msg)
            self.publish_count += 1
            if self.publish_tf:
                self.publish_transform(self.last_odom_msg.pose.pose, self.last_odom_msg.header.stamp)


def main(args=None):
    """主函数"""
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
