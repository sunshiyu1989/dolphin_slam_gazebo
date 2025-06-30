#!/usr/bin/env python3
"""
Dolphin SLAM - 位置细胞网络 ROS2 节点（完全重写版本）
只使用 Odometry 消息类型，避免任何冲突
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import MarkerArray, Marker
from sensor_msgs.msg import PointCloud2
import numpy as np
from typing import Optional

class PlaceCellNode(Node):
    """位置细胞网络 ROS2 节点"""
    
    def __init__(self):
        super().__init__('place_cell_node')
        
        # 声明参数
        self.declare_parameters(
            namespace='',
            parameters=[
                ('odometry_topic', '/robot/odometry'),
                ('visual_match_topic', '/local_view/matches'),
                ('activity_topic', '/place_cells/activity'),
                ('neurons_per_dimension', 16),
                ('update_rate', 20.0),
            ]
        )
        
        # 获取参数
        self.update_rate = self.get_parameter('update_rate').value
        self.neurons_per_dimension = self.get_parameter('neurons_per_dimension').value
        
        # 状态变量
        self.last_odometry: Optional[Odometry] = None
        self.activity_data = np.random.random(self.neurons_per_dimension**3) * 0.1
        
        # 订阅者 - 只使用 Odometry，绝不使用 PoseWithCovarianceStamped
        self.odometry_sub = self.create_subscription(
            Odometry,
            self.get_parameter('odometry_topic').value,
            self.odometry_callback,
            10
        )
        
        self.visual_match_sub = self.create_subscription(
            Float32MultiArray,
            self.get_parameter('visual_match_topic').value,
            self.visual_match_callback,
            10
        )
        
        # 发布者
        self.activity_pub = self.create_publisher(
            Float32MultiArray,
            self.get_parameter('activity_topic').value,
            10
        )
        
        # 定时器
        self.update_timer = self.create_timer(
            1.0 / self.update_rate,
            self.update_network
        )
        
        self.get_logger().info(f'位置细胞网络节点已启动 - {self.neurons_per_dimension}³ 神经元')
        
    def odometry_callback(self, msg: Odometry):
        """处理机器人里程计数据 - 只接受 Odometry 类型"""
        self.last_odometry = msg
        
        # 提取位置
        position = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])
        
        self.get_logger().debug(f'里程计更新: pos=[{position[0]:.2f}, {position[1]:.2f}, {position[2]:.2f}]')
        
    def visual_match_callback(self, msg: Float32MultiArray):
        """处理视觉匹配数据"""
        if len(msg.data) > 0:
            similarity = msg.data[0]
            self.get_logger().debug(f'视觉匹配: 相似度={similarity:.3f}')
        
    def update_network(self):
        """更新位置细胞网络"""
        if self.last_odometry is None:
            return
            
        try:
            # 简单的位置细胞活动模拟
            position = np.array([
                self.last_odometry.pose.pose.position.x,
                self.last_odometry.pose.pose.position.y,
                self.last_odometry.pose.pose.position.z
            ])
            
            # 创建简单的高斯活动模式
            center_idx = int(self.neurons_per_dimension / 2)
            self.activity_data = np.random.random(self.neurons_per_dimension**3) * 0.1
            self.activity_data[center_idx] = 1.0  # 峰值活动
            
            # 发布活动数据
            self.publish_activity()
            
        except Exception as e:
            self.get_logger().error(f'网络更新失败: {e}')
    
    def publish_activity(self):
        """发布位置细胞活动"""
        try:
            msg = Float32MultiArray()
            msg.data = self.activity_data.tolist()
            self.activity_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f'发布活动失败: {e}')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = PlaceCellNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'错误: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
