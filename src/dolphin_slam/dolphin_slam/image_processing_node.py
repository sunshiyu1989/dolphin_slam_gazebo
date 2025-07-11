#!/usr/bin/env python3
"""
修复版图像处理节点 - 连接Gazebo实时传感器
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from visualization_msgs.msg import MarkerArray, Marker
from cv_bridge import CvBridge
import cv2
import numpy as np
from typing import Optional

class ImageProcessingNode(Node):
    """修复版图像处理节点"""
    
    def __init__(self):
        super().__init__('image_processing_node')
        
        # 参数配置
        self.camera_topic = '/forward_camera/image_raw'
        self.sonar_topic = '/sonar/image_raw'
        self.descriptors_topic = '/features/descriptors'
        self.keypoints_topic = '/features/keypoints'
        self.feature_type = 'SIFT'
        self.max_features = 300
        self.process_every_n = 3
        self.enable_viz = True
        self.debug_mode = True
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # 初始化特征检测器
        self._init_feature_detector()
        
        # 帧计数器
        self.camera_frame_count = 0
        self.processed_frames = 0
        
        # 订阅者 - 重点：确保正确的话题
        self.camera_sub = self.create_subscription(
            Image,
            self.camera_topic,
            self.camera_callback,
            10
        )
        
        # 发布者
        self.descriptors_pub = self.create_publisher(
            Image,
            self.descriptors_topic,
            10
        )
        
        self.keypoints_pub = self.create_publisher(
            MarkerArray,
            self.keypoints_topic,
            10
        )
        
        # 可视化发布者
        self.camera_features_pub = self.create_publisher(
            Image,
            '/camera/image_features',
            10
        )
        
        # 定时器用于状态报告
        self.report_timer = self.create_timer(5.0, self.report_status)
        
        self.get_logger().info(f'🎯 修复版图像处理节点已启动')
        self.get_logger().info(f'📷 订阅相机话题: {self.camera_topic}')
        self.get_logger().info(f'🔍 使用特征: {self.feature_type}, 最大特征数: {self.max_features}')
        
    def _init_feature_detector(self):
        """初始化特征检测器"""
        try:
            if self.feature_type.upper() == 'SIFT':
                self.detector = cv2.SIFT_create(nfeatures=self.max_features)
            elif self.feature_type.upper() == 'ORB':
                self.detector = cv2.ORB_create(nfeatures=self.max_features)
            else:
                # 默认使用SIFT
                self.detector = cv2.SIFT_create(nfeatures=self.max_features)
                
            self.get_logger().info(f'✅ {self.feature_type} 特征检测器初始化成功')
            
        except Exception as e:
            self.get_logger().error(f'❌ 特征检测器初始化失败: {e}')
            # 备用方案
            self.detector = cv2.ORB_create(nfeatures=self.max_features)
            
    def camera_callback(self, msg: Image):
        """处理相机图像"""
        self.camera_frame_count += 1
        
        # 跳帧处理
        if self.camera_frame_count % self.process_every_n != 0:
            return
            
        try:
            # 转换为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # 转为灰度图
            if len(cv_image.shape) == 3:
                gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            else:
                gray = cv_image
                
            # 应用CLAHE对比度增强（水下环境重要）
            clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
            enhanced = clahe.apply(gray)
            
            # 特征检测和描述
            keypoints, descriptors = self.detector.detectAndCompute(enhanced, None)
            
            if descriptors is not None and len(keypoints) > 0:
                # 发布描述符（简化版本 - 使用图像消息）
                self._publish_descriptors(descriptors, msg.header)
                
                # 发布关键点可视化
                self._publish_keypoints(keypoints, msg.header)
                
                # 可视化特征点
                if self.enable_viz:
                    self._publish_features_visualization(cv_image, keypoints)
                
                self.processed_frames += 1
                
                # 🔧 降低日志输出频率 - 每100次处理显示一次
                if not hasattr(self, '_process_count'):
                    self._process_count = 0
                self._process_count += 1
                
                if self._process_count % 100 == 1:
                    processing_rate = (self.processed_frames / self.camera_frame_count * 100) if self.camera_frame_count > 0 else 0
                    self.get_logger().info(
                        f'📊 图像处理状态: 接收{self.camera_frame_count}帧, '
                        f'处理{self.processed_frames}帧, 处理率{processing_rate:.1f}%'
                    )
            else:
                # 🔧 减少警告频率 - 每100帧显示一次
                if self.debug_mode and self.camera_frame_count % 100 == 0:
                    self.get_logger().warn(f'⚠️ 帧#{self.camera_frame_count}: 未检测到特征点')
                    
        except Exception as e:
            self.get_logger().error(f'❌ 图像处理失败: {e}')
            
    def _publish_descriptors(self, descriptors, header):
        """发布描述符数据"""
        try:
            # 将描述符转换为图像格式（临时方案）
            # 正常应该使用自定义消息类型
            desc_normalized = descriptors.astype(np.float32)
            
            # 归一化到0-255范围
            if desc_normalized.max() > 0:
                desc_normalized = (desc_normalized / desc_normalized.max() * 255).astype(np.uint8)
            
            # 确保是2D图像格式
            if len(desc_normalized.shape) == 1:
                desc_normalized = desc_normalized.reshape(1, -1)
                
            # 创建图像消息
            desc_msg = self.bridge.cv2_to_imgmsg(desc_normalized, 'mono8')
            desc_msg.header = header
            
            self.descriptors_pub.publish(desc_msg)
            
        except Exception as e:
            self.get_logger().error(f'❌ 描述符发布失败: {e}')
            
    def _publish_keypoints(self, keypoints, header):
        """发布关键点可视化"""
        try:
            markers = MarkerArray()
            
            for i, kp in enumerate(keypoints[:50]):  # 限制显示数量
                marker = Marker()
                marker.header = header
                marker.header.frame_id = "camera_link"
                marker.ns = "keypoints"
                marker.id = i
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                
                # 关键点位置（简化映射到3D）
                marker.pose.position.x = float(kp.pt[0] / 1000.0)  # 缩放
                marker.pose.position.y = float(kp.pt[1] / 1000.0)
                marker.pose.position.z = 0.1
                marker.pose.orientation.w = 1.0
                
                # 标记大小和颜色
                marker.scale.x = 0.02
                marker.scale.y = 0.02
                marker.scale.z = 0.02
                
                marker.color.a = 1.0
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                
                marker.lifetime.sec = 1
                
                markers.markers = list(markers.markers) + [marker]
                
            self.keypoints_pub.publish(markers)
            
        except Exception as e:
            self.get_logger().error(f'❌ 关键点发布失败: {e}')
            
    def _publish_features_visualization(self, image, keypoints):
        """发布带特征点的图像可视化"""
        try:
            # 绘制特征点
            img_with_keypoints = cv2.drawKeypoints(
                image, keypoints, None, 
                color=(0, 255, 0), 
                flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS
            )
            
            # 发布可视化图像
            viz_msg = self.bridge.cv2_to_imgmsg(img_with_keypoints, 'bgr8')
            self.camera_features_pub.publish(viz_msg)
            
        except Exception as e:
            self.get_logger().error(f'❌ 可视化发布失败: {e}')
            
    def report_status(self):
        """状态报告"""
        self.get_logger().info(
            f'📊 图像处理状态: 接收{self.camera_frame_count}帧, '
            f'处理{self.processed_frames}帧, '
            f'处理率{self.processed_frames/max(self.camera_frame_count,1)*100:.1f}%'
        )

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = ImageProcessingNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
