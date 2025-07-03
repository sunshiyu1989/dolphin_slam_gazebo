#!/usr/bin/env python3
"""
Dolphin SLAM - 图像处理 ROS2 节点
处理相机和声呐图像，提取特征并发布描述符
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Header
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
import cv2
import numpy as np
from typing import Optional

# 导入核心模块
from dolphin_slam.image_processing import ImageProcessor

# 自定义消息（需要在 msg/ 目录中定义）
# from dolphin_slam_msgs.msg import Descriptors, Keypoints

class ImageProcessingNode(Node):
    """图像处理 ROS2 节点"""
    
    def __init__(self):
        super().__init__('image_processing_node')
        
        # 声明参数
        self.declare_parameters(
            namespace='',
            parameters=[
                ('feature_type', 'SURF'),
                ('max_features', 1000),
                ('surf_hessian_threshold', 400),
                ('surf_upright', False),
                ('process_every_n_frames', 1),
                ('enable_visualization', True),
                ('camera_topic', '/camera/image_raw'),
                ('sonar_topic', '/sonar/image_raw'),
                ('descriptors_topic', '/features/descriptors'),
                ('keypoints_topic', '/features/keypoints'),
            ]
        )
        
        # 获取参数
        self.feature_type = self.get_parameter('feature_type').value
        self.max_features = self.get_parameter('max_features').value
        self.hessian_threshold = self.get_parameter('surf_hessian_threshold').value
        self.upright = self.get_parameter('surf_upright').value
        self.process_every_n = self.get_parameter('process_every_n_frames').value
        self.enable_viz = self.get_parameter('enable_visualization').value
        
        # 初始化图像处理器
        self.processor = ImageProcessor(
            feature_type=self.feature_type,
            max_features=self.max_features,
            hessian_threshold=self.hessian_threshold,
            upright=self.upright
        )
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # 帧计数器
        self.camera_frame_count = 0
        self.sonar_frame_count = 0
        
        # 订阅者
        self.camera_sub = self.create_subscription(
            Image,
            self.get_parameter('camera_topic').value,
            self.camera_callback,
            10
        )
        
        self.sonar_sub = self.create_subscription(
            Image,
            self.get_parameter('sonar_topic').value,
            self.sonar_callback,
            10
        )
        
        # 发布者
        # 特征点可视化
        self.keypoints_pub = self.create_publisher(
            MarkerArray,
            self.get_parameter('keypoints_topic').value,
            10
        )
        
        # 带特征点的图像
        self.camera_features_pub = self.create_publisher(
            Image,
            '/camera/image_features',
            10
        )
        
        self.sonar_features_pub = self.create_publisher(
            Image,
            '/sonar/image_features',
            10
        )
        
        # 描述符（简化版，实际应使用自定义消息）
        self.descriptors_pub = self.create_publisher(
            Image,  # 临时使用 Image 消息存储描述符
            self.get_parameter('descriptors_topic').value,
            10
        )
        
        self.get_logger().info(f'图像处理节点已启动，使用 {self.feature_type} 特征')
        
    def camera_callback(self, msg: Image):
        """处理相机图像"""
        self.camera_frame_count += 1
        
        # 检查是否应该处理这一帧
        if self.camera_frame_count % self.process_every_n != 0:
            return
            
        try:
            # 转换为 OpenCV 格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # 提取特征
            keypoints, descriptors = self.processor.process_camera_image(cv_image)
            
            if len(keypoints) > 0:
                # 发布描述符
                self.publish_descriptors(descriptors, msg.header, 'camera')
                
                # 可视化
                if self.enable_viz:
                    # 发布特征点标记
                    self.publish_keypoint_markers(keypoints, msg.header, 'camera')
                    
                    # 发布带特征点的图像
                    feature_image = self.processor.draw_keypoints(cv_image, keypoints)
                    feature_msg = self.bridge.cv2_to_imgmsg(feature_image, encoding='bgr8')
                    feature_msg.header = msg.header
                    self.camera_features_pub.publish(feature_msg)
                    
                self.get_logger().debug(f'相机图像: 检测到 {len(keypoints)} 个特征点')
                
        except Exception as e:
            self.get_logger().error(f'处理相机图像时出错: {e}')
            
    def sonar_callback(self, msg: Image):
        """处理声呐图像"""
        self.sonar_frame_count += 1
        
        # 检查是否应该处理这一帧
        if self.sonar_frame_count % self.process_every_n != 0:
            return
            
        try:
            # 转换为 OpenCV 格式
            # 声呐数据可能是单通道浮点数
            if msg.encoding == '32FC1':
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            else:
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
                
            # 提取特征
            keypoints, descriptors = self.processor.process_sonar_image(cv_image)
            
            if len(keypoints) > 0:
                # 发布描述符
                self.publish_descriptors(descriptors, msg.header, 'sonar')
                
                # 可视化
                if self.enable_viz:
                    # 发布特征点标记
                    self.publish_keypoint_markers(keypoints, msg.header, 'sonar')
                    
                    # 发布带特征点的图像
                    # 先转换为可视化的格式
                    viz_image = cv2.normalize(cv_image, None, 0, 255, cv2.NORM_MINMAX)
                    viz_image = viz_image.astype(np.uint8)
                    if len(viz_image.shape) == 2:
                        viz_image = cv2.cvtColor(viz_image, cv2.COLOR_GRAY2BGR)
                        
                    feature_image = self.processor.draw_keypoints(viz_image, keypoints)
                    feature_msg = self.bridge.cv2_to_imgmsg(feature_image, encoding='bgr8')
                    feature_msg.header = msg.header
                    self.sonar_features_pub.publish(feature_msg)
                    
                self.get_logger().debug(f'声呐图像: 检测到 {len(keypoints)} 个特征点')
                
        except Exception as e:
            self.get_logger().error(f'处理声呐图像时出错: {e}')
            
    def publish_descriptors(self, descriptors: np.ndarray, header: Header, source: str):
        """
        发布描述符
        
        注意：这是简化版本，实际应该使用自定义消息类型
        """
        # 将描述符矩阵编码为图像消息（临时方案）
        # 每行是一个描述符
        desc_msg = Image()
        desc_msg.header = header
        desc_msg.height = descriptors.shape[0]
        desc_msg.width = descriptors.shape[1]
        desc_msg.encoding = '32FC1'
        desc_msg.is_bigendian = False
        desc_msg.step = desc_msg.width * 4  # float32 = 4 bytes
        desc_msg.data = descriptors.astype(np.float32).tobytes()
        
        self.descriptors_pub.publish(desc_msg)
        
    def publish_keypoint_markers(self, keypoints, header: Header, source: str):
        """发布特征点的可视化标记 - 修复版本"""
        marker_array = MarkerArray()
        
        # 1. 首先清除所有旧标记
        delete_marker = Marker()
        delete_marker.header = header
        delete_marker.ns = f"{source}_keypoints"
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)
        
        # 2. 为每个特征点创建唯一 ID 的标记
        for i, kp in enumerate(keypoints):
            marker = Marker()
            marker.header = header
            marker.ns = f"{source}_keypoints"
            
            # 确保不同来源的 marker 有不同的 ID 范围
            if source == 'camera':
                marker.id = i  # camera: 0-999
            elif source == 'sonar':
                marker.id = i + 1000  # sonar: 1000-1999
            else:
                marker.id = i + 2000  # 其他: 2000+
                
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            # 位置设置
            marker.pose.position.x = kp.pt[0] / 1000.0
            marker.pose.position.y = kp.pt[1] / 1000.0
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0
            
            # 大小设置
            scale = max(0.01, kp.size / 100.0)
            marker.scale.x = scale
            marker.scale.y = scale
            marker.scale.z = scale
            
            # 颜色区分
            if source == 'camera':
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            else:  # sonar
                marker.color.r = 1.0
                marker.color.g = 0.5
                marker.color.b = 0.0
            marker.color.a = 0.8
            
            # 设置生命周期
            marker.lifetime = rclpy.duration.Duration(seconds=1.0).to_msg()
            
            marker_array.markers.append(marker)
        
        # 3. 发布前检查重复 ID
        ids_seen = set()
        clean_markers = []
        
        for marker in marker_array.markers:
            if marker.action == Marker.DELETEALL:
                clean_markers.append(marker)
            else:
                key = (marker.ns, marker.id)
                if key not in ids_seen:
                    ids_seen.add(key)
                    clean_markers.append(marker)
                else:
                    self.get_logger().warn(f"跳过重复的 marker: ns={marker.ns}, id={marker.id}")
        
        marker_array.markers = clean_markers
        self.keypoints_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = ImageProcessingNode()
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
