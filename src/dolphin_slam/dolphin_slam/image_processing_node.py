#!/usr/bin/env python3
"""
Dolphin SLAM - 图像处理 ROS2 节点
兼容新版本 OpenCV，支持多种特征检测器
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ImageProcessingNode(Node):
    """图像处理 ROS2 节点 - 兼容版本"""
    
    def __init__(self):
        super().__init__('image_processing_node')
        
        # 声明参数
        self.declare_parameters(
            namespace='',
            parameters=[
                ('feature_type', 'AUTO'),  # 自动选择最佳可用检测器
                ('max_features', 1000),
                ('enable_visualization', True),
            ]
        )
        
        # 获取参数
        self.feature_type = self.get_parameter('feature_type').value
        self.max_features = self.get_parameter('max_features').value
        self.enable_viz = self.get_parameter('enable_visualization').value
        
        # 初始化特征检测器
        self.detector = self._init_best_detector()
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # 创建测试定时器
        self.timer = self.create_timer(2.0, self.test_callback)
        
        self.get_logger().info(f'🐬 图像处理节点已启动，使用 {self.actual_detector_type} 特征检测器')
        
    def _init_best_detector(self):
        """自动选择最佳可用的特征检测器"""
        
        detectors_to_try = [
            ('SURF', lambda: cv2.xfeatures2d.SURF_create(hessianThreshold=400) if hasattr(cv2, 'xfeatures2d') else None),
            ('SIFT', lambda: cv2.SIFT_create(nfeatures=self.max_features)),
            ('ORB', lambda: cv2.ORB_create(nfeatures=self.max_features)),
            ('AKAZE', lambda: cv2.AKAZE_create()),
            ('BRISK', lambda: cv2.BRISK_create()),
        ]
        
        # 如果用户指定了特定类型，优先尝试
        if self.feature_type.upper() != 'AUTO':
            for name, creator in detectors_to_try:
                if name == self.feature_type.upper():
                    try:
                        detector = creator()
                        if detector is not None:
                            self.actual_detector_type = name
                            self.get_logger().info(f'✅ 使用指定的 {name} 检测器')
                            return detector
                    except Exception as e:
                        self.get_logger().warn(f'⚠️ {name} 不可用: {e}')
                    break
        
        # 自动选择第一个可用的检测器
        for name, creator in detectors_to_try:
            try:
                detector = creator()
                if detector is not None:
                    self.actual_detector_type = name
                    self.get_logger().info(f'✅ 自动选择 {name} 检测器')
                    return detector
            except Exception as e:
                self.get_logger().debug(f'跳过 {name}: {e}')
                continue
        
        # 如果所有都失败，抛出错误
        raise RuntimeError("❌ 没有可用的特征检测器")
        
    def test_callback(self):
        """测试回调函数"""
        self.get_logger().info(f'🌊 {self.actual_detector_type} 图像处理节点运行正常')

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
