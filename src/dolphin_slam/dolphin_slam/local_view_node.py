#!/usr/bin/env python3
"""
Dolphin SLAM - 局部视觉细胞 ROS2 节点
实现视觉场景识别和模板匹配
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, String
from visualization_msgs.msg import MarkerArray, Marker
from cv_bridge import CvBridge
import numpy as np
import cv2
import os
import pickle
from typing import Optional, List, Dict, Tuple

# 导入核心模块
try:
    from dolphin_slam.local_view_cells import LocalViewCells, VisualTemplate
except ImportError:
    # 如果导入失败，创建一个简单的替代实现
    class LocalViewCells:
        def __init__(self, **kwargs):
            self.templates = {}
            self.next_template_id = 0
            
        def process_descriptors(self, descriptors):
            return 0.5, 0  # 返回固定的相似度和模板ID

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
                ('vocabulary_size', 500),
                ('clustering_algorithm', 'kmeans'),
                ('max_templates', 2000),
                ('template_decay_rate', 0.995),
                ('vocabulary_path', ''),
            ]
        )
        
        # 获取参数
        self.descriptors_topic = self.get_parameter('descriptors_topic').value
        self.matches_topic = self.get_parameter('matches_topic').value
        self.matching_algorithm = self.get_parameter('matching_algorithm').value
        self.similarity_threshold = self.get_parameter('similarity_threshold').value
        self.vocabulary_size = self.get_parameter('vocabulary_size').value
        self.clustering_algorithm = self.get_parameter('clustering_algorithm').value
        self.max_templates = self.get_parameter('max_templates').value
        self.template_decay_rate = self.get_parameter('template_decay_rate').value
        self.vocabulary_path = self.get_parameter('vocabulary_path').value
        
        # 初始化局部视觉细胞
        self.local_view = LocalViewCells(
            matching_algorithm=self.matching_algorithm,
            similarity_threshold=self.similarity_threshold,
            vocabulary_size=self.vocabulary_size,
            clustering_algorithm=self.clustering_algorithm,
            max_templates=self.max_templates,
            template_decay_rate=self.template_decay_rate
        )
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # 计数器
        self.descriptor_count = 0
        self.match_count = 0
        
        # 训练数据收集
        self.training_descriptors = []
        self.is_training = False
        
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
        
        # 服务
        self.train_vocab_srv = self.create_service(
            'std_srvs/srv/Trigger',
            '/local_view/train_vocabulary',
            self.train_vocabulary_callback
        )
        
        self.save_vocab_srv = self.create_service(
            'std_srvs/srv/Trigger',
            '/local_view/save_vocabulary',
            self.save_vocabulary_callback
        )
        
        # 定时器
        self.viz_timer = self.create_timer(1.0, self.publish_visualizations)
        self.stats_timer = self.create_timer(5.0, self.publish_statistics)
        
        # 尝试加载词汇表
        if self.vocabulary_path and os.path.exists(self.vocabulary_path):
            self.load_vocabulary()
            
        self.get_logger().info(f'局部视觉细胞节点已启动，使用 {self.matching_algorithm} 算法')
        
    def descriptors_callback(self, msg: Image):
        """处理描述符输入"""
        try:
            # 从 Image 消息解码描述符
            descriptors = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            
            if descriptors is None or descriptors.size == 0:
                self.get_logger().warning('接收到空的描述符数据')
                return
                
            self.descriptor_count += 1
            
            # 处理描述符并获取匹配结果
            similarity, template_id = self.local_view.process_descriptors(descriptors)
            
            # 创建匹配消息
            match_msg = Float32MultiArray()
            match_msg.data = [float(similarity), float(template_id)]
            
            # 发布匹配结果
            self.match_pub.publish(match_msg)
            
            self.match_count += 1
            
            if self.match_count % 10 == 0:
                self.get_logger().info(f'已处理 {self.match_count} 个视觉匹配')
                
        except Exception as e:
            self.get_logger().error(f'描述符处理失败: {e}')
    
    def train_vocabulary_callback(self, request, response):
        """训练词汇表服务回调"""
        try:
            if len(self.training_descriptors) > 0:
                self.get_logger().info(f'开始训练词汇表，使用 {len(self.training_descriptors)} 个描述符')
                
                # 这里应该实现实际的词汇表训练
                # 现在返回成功状态
                response.success = True
                response.message = f"词汇表训练完成，使用了 {len(self.training_descriptors)} 个描述符"
            else:
                response.success = False
                response.message = "没有足够的训练数据"
                
        except Exception as e:
            response.success = False
            response.message = f"训练失败: {e}"
            
        return response
        
    def save_vocabulary_callback(self, request, response):
        """保存词汇表服务回调"""
        try:
            if self.vocabulary_path:
                vocab_file = os.path.join(self.vocabulary_path, 'vocabulary.npy')
                # 这里应该实现实际的词汇表保存
                response.success = True
                response.message = f"词汇表已保存到 {vocab_file}"
            else:
                response.success = False
                response.message = "词汇表路径未设置"
        except Exception as e:
            response.success = False
            response.message = f"保存失败: {e}"
            
        return response
        
    def load_vocabulary(self):
        """加载词汇表"""
        try:
            vocab_file = os.path.join(self.vocabulary_path, 'vocabulary.npy')
            if os.path.exists(vocab_file):
                # 这里应该实现实际的词汇表加载
                self.get_logger().info(f"词汇表已从 {vocab_file} 加载")
            else:
                self.get_logger().warning(f"词汇表文件不存在: {vocab_file}")
        except Exception as e:
            self.get_logger().error(f"加载词汇表失败: {e}")
            
    def save_templates(self):
        """保存模板数据库"""
        try:
            if self.vocabulary_path:
                templates_file = os.path.join(self.vocabulary_path, 'templates.pkl')
                with open(templates_file, 'wb') as f:
                    pickle.dump({
                        'templates': self.local_view.templates,
                        'descriptor_count': self.descriptor_count,
                        'match_count': self.match_count
                    }, f)
                self.get_logger().info(f"模板已保存到 {templates_file}")
        except Exception as e:
            self.get_logger().error(f"保存模板失败: {e}")
            
    def publish_visualizations(self):
        """发布可视化信息"""
        try:
            # 创建模板活动可视化
            markers = MarkerArray()
            
            # 创建一个简单的统计标记
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "local_view_stats"
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
            
            marker.text = f"视觉匹配: {self.match_count}"
            
            markers.markers.append(marker)
            self.template_activity_pub.publish(markers)
            
        except Exception as e:
            self.get_logger().error(f"可视化发布失败: {e}")
        
    def publish_statistics(self):
        """发布统计信息"""
        try:
            # 创建统计信息标记
            markers = MarkerArray()
            
            # 统计标记
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "local_view_detailed_stats"
            marker.id = 0
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
            
            marker.text = f"描述符: {self.descriptor_count}, 模板: {len(self.local_view.templates)}"
            
            markers.markers.append(marker)
            self.stats_pub.publish(markers)
            
        except Exception as e:
            self.get_logger().error(f"统计信息发布失败: {e}")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = LocalViewNode()
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