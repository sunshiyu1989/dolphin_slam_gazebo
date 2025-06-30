#!/usr/bin/env python3
"""
Dolphin SLAM - 图像处理模块
提取视觉特征并生成描述符 - 兼容新版本 OpenCV
"""

import cv2
import numpy as np
from typing import Optional, Tuple, List
import logging

class ImageProcessor:
    """处理相机和声呐图像，提取特征点和描述符"""
    
    def __init__(self, feature_type='AUTO', max_features=1000, 
                 hessian_threshold=400, upright=False):
        """
        初始化图像处理器
        
        参数:
            feature_type: 特征类型 ('SURF', 'SIFT', 'ORB', 'AUTO')
            max_features: 最大特征点数
            hessian_threshold: SURF Hessian 阈值
            upright: 是否使用 upright SURF
        """
        self.requested_feature_type = feature_type
        self.max_features = max_features
        self.logger = logging.getLogger(__name__)
        
        # 初始化特征检测器（自动选择最佳可用的）
        self.detector, self.feature_type = self._init_best_detector(hessian_threshold, upright)
        
        # 存储上一帧的特征，用于匹配
        self.prev_keypoints = None
        self.prev_descriptors = None
        
        # 初始化 CLAHE 用于图像增强
        self.clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        
    def _init_best_detector(self, hessian_threshold, upright):
        """自动选择最佳可用的特征检测器"""
        
        detectors_to_try = [
            ('SURF', lambda: cv2.xfeatures2d.SURF_create(
                hessianThreshold=hessian_threshold, upright=upright
            ) if hasattr(cv2, 'xfeatures2d') else None),
            ('SIFT', lambda: cv2.SIFT_create(nfeatures=self.max_features)),
            ('ORB', lambda: cv2.ORB_create(nfeatures=self.max_features)),
            ('AKAZE', lambda: cv2.AKAZE_create()),
            ('BRISK', lambda: cv2.BRISK_create()),
        ]
        
        # 如果用户指定了特定类型，优先尝试
        if self.requested_feature_type.upper() != 'AUTO':
            for name, creator in detectors_to_try:
                if name == self.requested_feature_type.upper():
                    try:
                        detector = creator()
                        if detector is not None:
                            if self.logger:
                                self.logger.info(f'使用指定的 {name} 检测器')
                            return detector, name
                    except Exception as e:
                        if self.logger:
                            self.logger.warning(f'{name} 不可用: {e}，尝试自动选择')
                    break
        
        # 自动选择第一个可用的检测器
        for name, creator in detectors_to_try:
            try:
                detector = creator()
                if detector is not None:
                    if self.logger:
                        self.logger.info(f'自动选择 {name} 检测器')
                    return detector, name
            except Exception as e:
                if self.logger:
                    self.logger.debug(f'跳过 {name}: {e}')
                continue
        
        # 如果所有都失败，抛出错误
        raise RuntimeError("没有可用的特征检测器")
        
    def enhance_underwater_image(self, image: np.ndarray) -> np.ndarray:
        """
        水下图像增强
        
        参数:
            image: 输入图像
            
        返回:
            enhanced_image: 增强后的图像
        """
        if len(image.shape) == 3:
            # 彩色图像：在 LAB 颜色空间中增强 L 通道
            lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
            lab[:, :, 0] = self.clahe.apply(lab[:, :, 0])
            enhanced = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)
        else:
            # 灰度图像：直接应用 CLAHE
            enhanced = self.clahe.apply(image)
            
        return enhanced
        
    def process_camera_image(self, image: np.ndarray) -> Tuple[List, np.ndarray]:
        """
        处理相机图像
        
        参数:
            image: 输入图像 (BGR 或灰度)
            
        返回:
            keypoints: 关键点列表
            descriptors: 描述符矩阵
        """
        # 转换为灰度图
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image.copy()
            
        # 图像增强
        enhanced = self.enhance_underwater_image(gray)
        
        # 检测特征点和计算描述符
        keypoints, descriptors = self.detector.detectAndCompute(enhanced, None)
        
        # 确保返回列表格式
        if keypoints is None:
            keypoints = []
        else:
            keypoints = list(keypoints)
        
        # 限制特征点数量
        if len(keypoints) > self.max_features:
            # 按响应强度排序并保留最强的特征点
            keypoints = sorted(keypoints, key=lambda x: x.response, reverse=True)
            keypoints = keypoints[:self.max_features]
            if descriptors is not None:
                descriptors = descriptors[:self.max_features]
        
        if self.logger:
            self.logger.debug(f"相机图像检测到 {len(keypoints)} 个特征点")
        
        return keypoints, descriptors
        
    def process_sonar_image(self, image: np.ndarray) -> Tuple[List, np.ndarray]:
        """
        处理声呐图像
        
        参数:
            image: 声呐图像 (通常是极坐标格式)
            
        返回:
            keypoints: 关键点列表
            descriptors: 描述符矩阵
        """
        # 确保图像是正确的格式
        if image.dtype != np.uint8:
            # 标准化到 0-255 范围
            if image.max() <= 1.0:
                sonar_uint8 = (image * 255).astype(np.uint8)
            else:
                sonar_uint8 = cv2.normalize(image, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        else:
            sonar_uint8 = image
            
        # 声呐图像预处理
        # 应用高斯滤波减少噪声
        denoised = cv2.GaussianBlur(sonar_uint8, (3, 3), 0)
        
        # 应用 CLAHE 增强对比度
        enhanced = self.clahe.apply(denoised)
        
        # 检测特征点和计算描述符
        keypoints, descriptors = self.detector.detectAndCompute(enhanced, None)
        
        # 确保返回列表格式
        if keypoints is None:
            keypoints = []
        else:
            keypoints = list(keypoints)
        
        # 限制特征点数量
        if len(keypoints) > self.max_features:
            keypoints = sorted(keypoints, key=lambda x: x.response, reverse=True)
            keypoints = keypoints[:self.max_features]
            if descriptors is not None:
                descriptors = descriptors[:self.max_features]
        
        if self.logger:
            self.logger.debug(f"声呐图像检测到 {len(keypoints)} 个特征点")
        
        return keypoints, descriptors
        
    def match_features(self, descriptors1: np.ndarray, 
                      descriptors2: np.ndarray) -> List:
        """
        匹配两组描述符
        
        参数:
            descriptors1: 第一组描述符
            descriptors2: 第二组描述符
            
        返回:
            matches: 匹配列表
        """
        if descriptors1 is None or descriptors2 is None or len(descriptors1) == 0 or len(descriptors2) == 0:
            return []
            
        # 创建匹配器
        if self.feature_type in ['SURF', 'SIFT', 'AKAZE']:
            matcher = cv2.BFMatcher(cv2.NORM_L2, crossCheck=False)
        else:  # ORB, BRISK
            matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
            
        try:
            # 使用 KNN 匹配
            matches = matcher.knnMatch(descriptors1, descriptors2, k=2)
            
            # 应用 Lowe's ratio test
            good_matches = []
            for match_pair in matches:
                if len(match_pair) == 2:
                    m, n = match_pair
                    if m.distance < 0.7 * n.distance:
                        good_matches.append(m)
                        
            return good_matches
        except Exception as e:
            if self.logger:
                self.logger.warning(f"特征匹配失败: {e}")
            return []
        
    def compute_bow_descriptor(self, descriptors: np.ndarray, 
                             vocabulary: np.ndarray) -> np.ndarray:
        """
        计算视觉词袋（BoW）描述符
        
        参数:
            descriptors: 特征描述符
            vocabulary: 视觉词汇表
            
        返回:
            bow_descriptor: BoW 描述符向量
        """
        if descriptors is None or len(descriptors) == 0:
            return np.zeros(len(vocabulary))
            
        try:
            # 创建 FLANN 匹配器用于快速最近邻搜索
            FLANN_INDEX_KDTREE = 1
            index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
            search_params = dict(checks=50)
            flann = cv2.FlannBasedMatcher(index_params, search_params)
            
            # 找到每个描述符最近的视觉词
            matches = flann.match(descriptors.astype(np.float32), 
                                vocabulary.astype(np.float32))
            
            # 计算词频
            bow_descriptor = np.zeros(len(vocabulary))
            for match in matches:
                bow_descriptor[match.trainIdx] += 1
                
            # 归一化
            if np.sum(bow_descriptor) > 0:
                bow_descriptor = bow_descriptor / np.sum(bow_descriptor)
                
            return bow_descriptor
        except Exception as e:
            if self.logger:
                self.logger.warning(f"BoW 描述符计算失败: {e}")
            return np.zeros(len(vocabulary))
        
    def extract_shape_descriptors(self, image: np.ndarray) -> np.ndarray:
        """
        提取形状描述符（用于声呐目标）
        
        参数:
            image: 输入图像
            
        返回:
            shape_descriptors: 形状描述符矩阵
        """
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image
            
        # 二值化
        _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        
        # 找到轮廓
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        shape_descriptors = []
        for contour in contours:
            if cv2.contourArea(contour) > 100:  # 过滤小轮廓
                # 计算 Hu 矩
                moments = cv2.moments(contour)
                if moments['m00'] != 0:  # 避免除零错误
                    hu_moments = cv2.HuMoments(moments).flatten()
                    # 取对数使其更稳定
                    hu_moments = -np.sign(hu_moments) * np.log10(np.abs(hu_moments) + 1e-10)
                    shape_descriptors.append(hu_moments)
                    
        if len(shape_descriptors) == 0:
            return np.array([]).reshape(0, 7)  # Hu 矩有 7 个值
            
        return np.array(shape_descriptors)
        
    def draw_keypoints(self, image: np.ndarray, keypoints: List) -> np.ndarray:
        """
        在图像上绘制关键点
        
        参数:
            image: 输入图像
            keypoints: 关键点列表
            
        返回:
            output_image: 绘制了关键点的图像
        """
        output_image = image.copy()
        
        # 绘制关键点
        cv2.drawKeypoints(output_image, keypoints, output_image, 
                         color=(0, 255, 0), flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        
        return output_image
    
    def get_detector_info(self) -> dict:
        """
        获取检测器信息
        
        返回:
            info: 检测器信息字典
        """
        return {
            'requested_type': self.requested_feature_type,
            'actual_type': self.feature_type,
            'max_features': self.max_features,
            'detector_available': self.detector is not None
        }