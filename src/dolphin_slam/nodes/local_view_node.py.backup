#!/usr/bin/env python3
"""
Dolphin SLAM - 局部视觉细胞 ROS2 节点（最小安全版本）
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import MarkerArray, Marker
from cv_bridge import CvBridge
import numpy as np

class LocalViewNode(Node):
    """局部视觉细胞 ROS2 节点"""
    
    def __init__(self):
        super().__init__('local_view_node')
        
        # 声明参数
        self.declare_parameters(
            namespace='',
            parameters=[
                ('descriptors_topic', '/features/descriptors'),
                ('matches_topic', '/local_view/matches'),
                ('matching_algorithm', 'fabmap'),
                ('similarity_threshold', 0.65),
            ]
        )
        
        # 获取参数
        self.descriptors_topic = self.get_parameter('descriptors_topic').value
        self.matches_topic = self.get_parameter('matches_topic').value
        self.matching_algorithm = self.get_parameter('matching_algorithm').value
        self.similarity_threshold = self.get_parameter('similarity_threshold').value
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # 计数器
        self.descriptor_count = 0
        self.match_count = 0
        
        # 订阅者
        self.descriptors_sub = self.create_subscription(
            Image,
            self.descriptors_topic,
            self.descriptors_callback,
            10
        )
        
        # 发布者
        self.match_pub = self.create_publisher(
            Float32MultiArray,
            self.matches_topic,
            10
        )
        
        self.template_activity_pub = self.create_publisher(
            MarkerArray,
            '/local_view/template_activity',
            10
        )
        
        self.stats_pub = self.create_publisher(
            MarkerArray,
            '/local_view/statistics',
            10
        )
        
        # 定时器
        self.viz_timer = self.create_timer(1.0, self.publish_visualizations)
        self.stats_timer = self.create_timer(5.0, self.publish_statistics)
        
        self.get_logger().info(f'局部视觉细胞节点已启动（安全版本）')
        
    def descriptors_callback(self, msg: Image):
        """处理描述符输入"""
        try:
            self.descriptor_count += 1
            
            # 创建简单的匹配响应
            match_msg = Float32MultiArray()
            match_msg.data = [0.5, 0.0]  # 固定相似度和模板ID
            
            # 发布匹配结果
            self.match_pub.publish(match_msg)
            
            self.match_count += 1
            
            if self.match_count % 100 == 0:
                self.get_logger().info(f'已处理 {self.match_count} 个视觉匹配')
                
        except Exception as e:
            self.get_logger().error(f'描述符处理失败: {e}')
            
    def publish_visualizations(self):
        """发布可视化信息"""
        try:
            # 创建简单的可视化标记
            markers = MarkerArray()
            
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "local_view"
            marker.id = 0
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            
            marker.pose.position.x = 0.0
            marker.pose.position.y = 0.0
            marker.pose.position.z = 2.0
            marker.pose.orientation.w = 1.0
            
            marker.scale.z = 0.3
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            
            marker.text = f"LocalView: {self.match_count} matches"
            
            markers.markers.append(marker)
            self.template_activity_pub.publish(markers)
            
        except Exception as e:
            self.get_logger().debug(f'可视化错误: {e}')
        
    def publish_statistics(self):
        """发布统计信息"""
        try:
            markers = MarkerArray()
            
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "local_view_stats"
            marker.id = 1
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            
            marker.pose.position.x = 0.0
            marker.pose.position.y = 0.0
            marker.pose.position.z = 1.5
            marker.pose.orientation.w = 1.0
            
            marker.scale.z = 0.2
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            
            marker.text = f"Descriptors: {self.descriptor_count}"
            
            markers.markers.append(marker)
            self.stats_pub.publish(markers)
            
        except Exception as e:
            self.get_logger().debug(f'统计错误: {e}')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = LocalViewNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'节点错误: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
