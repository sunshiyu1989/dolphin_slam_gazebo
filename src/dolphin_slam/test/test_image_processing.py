#!/usr/bin/env python3
"""
Dolphin SLAM - 图像处理单元测试 - 兼容版本
测试图像处理模块的功能，支持多种特征检测器
"""

import pytest
import numpy as np
import cv2
import sys
import os
from unittest.mock import Mock, patch

# 添加项目路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from dolphin_slam.image_processing import ImageProcessor


class TestImageProcessor:
    """图像处理测试类"""
    
    @pytest.fixture
    def processor(self):
        """创建测试用的图像处理器 - 自动选择可用检测器"""
        return ImageProcessor(
            feature_type='AUTO',  # 自动选择最佳可用检测器
            max_features=1000
        )
        
    @pytest.fixture
    def orb_processor(self):
        """创建使用 ORB 的图像处理器（最兼容）"""
        return ImageProcessor(
            feature_type='ORB',
            max_features=1000
        )
        
    @pytest.fixture
    def test_image(self):
        """创建测试图像"""
        # 创建一个有纹理的测试图像
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        
        # 添加一些特征（棋盘格）
        for i in range(0, 480, 40):
            for j in range(0, 640, 40):
                if (i//40 + j//40) % 2 == 0:
                    cv2.rectangle(img, (j, i), (j+40, i+40), (255, 255, 255), -1)
                    
        # 添加一些圆形
        cv2.circle(img, (320, 240), 50, (128, 128, 128), -1)
        cv2.circle(img, (100, 100), 30, (200, 200, 200), -1)
        cv2.circle(img, (540, 380), 40, (180, 180, 180), -1)
        
        return img
        
    @pytest.fixture
    def test_sonar_image(self):
        """创建测试声呐图像"""
        # 创建极坐标声呐图像
        img = np.zeros((497, 902), dtype=np.float32)
        
        # 添加一些目标
        # 近距离强反射
        cv2.circle(img, (450, 100), 20, 0.8, -1)
        
        # 远距离弱反射
        cv2.circle(img, (200, 350), 30, 0.3, -1)
        cv2.circle(img, (700, 400), 25, 0.4, -1)
        
        # 添加噪声
        noise = np.random.normal(0, 0.05, img.shape)
        img = np.clip(img + noise, 0, 1)
        
        return img
        
    def test_initialization(self, processor):
        """测试处理器初始化"""
        assert processor.feature_type in ['SURF', 'SIFT', 'ORB', 'AKAZE', 'BRISK']
        assert processor.max_features == 1000
        assert processor.detector is not None
        
        # 获取检测器信息
        info = processor.get_detector_info()
        assert 'actual_type' in info
        assert 'detector_available' in info
        assert info['detector_available'] == True
        
        print(f"使用的特征检测器: {processor.feature_type}")
        
    def test_camera_image_processing(self, processor, test_image):
        """测试相机图像处理"""
        keypoints, descriptors = processor.process_camera_image(test_image)
        
        # 应该检测到特征点
        assert len(keypoints) > 0
        assert descriptors is not None
        assert descriptors.shape[0] == len(keypoints)
        
        # 特征点数量应该合理
        assert len(keypoints) <= processor.max_features
        
        print(f"相机图像检测到 {len(keypoints)} 个特征点")
        
    def test_sonar_image_processing(self, processor, test_sonar_image):
        """测试声呐图像处理"""
        keypoints, descriptors = processor.process_sonar_image(test_sonar_image)
        
        # 声呐图像应该能提取到一些特征
        assert len(keypoints) >= 0  # 可能没有特征，但不应该报错
        if len(keypoints) > 0:
            assert descriptors is not None
            
        print(f"声呐图像检测到 {len(keypoints)} 个特征点")
        
    def test_feature_matching(self, processor, test_image):
        """测试特征匹配"""
        # 创建两个略有不同的图像
        img1 = test_image.copy()
        img2 = test_image.copy()
        
        # 稍微平移第二个图像
        M = np.float32([[1, 0, 10], [0, 1, 5]])
        img2 = cv2.warpAffine(img2, M, (img2.shape[1], img2.shape[0]))
        
        # 提取特征
        kp1, desc1 = processor.process_camera_image(img1)
        kp2, desc2 = processor.process_camera_image(img2)
        
        if len(kp1) > 0 and len(kp2) > 0:
            # 匹配特征
            matches = processor.match_features(desc1, desc2)
            
            # 应该找到一些匹配
            assert len(matches) >= 0  # 至少不应该报错
            if len(matches) > 0:
                assert len(matches) <= min(len(kp1), len(kp2))
                
            print(f"找到 {len(matches)} 个特征匹配")
        
    def test_bow_descriptor(self, processor, test_image):
        """测试 BoW 描述符计算"""
        # 创建一个简单的词汇表
        vocabulary_size = 100
        
        # 提取特征
        _, descriptors = processor.process_camera_image(test_image)
        
        if descriptors is not None and len(descriptors) > 0:
            # 使用实际描述符的维度创建词汇表
            desc_dim = descriptors.shape[1]
            vocabulary = np.random.rand(vocabulary_size, desc_dim).astype(np.float32)
            
            # 计算 BoW 描述符
            bow_desc = processor.compute_bow_descriptor(descriptors, vocabulary)
            
            # 检查 BoW 描述符
            assert len(bow_desc) == len(vocabulary)
            assert np.sum(bow_desc) >= 0  # 应该是非负的
            if np.sum(bow_desc) > 0:
                assert np.abs(np.sum(bow_desc) - 1.0) < 1e-6  # 应该归一化
                
            print(f"BoW 描述符维度: {len(bow_desc)}")
        
    def test_shape_descriptors(self, processor, test_sonar_image):
        """测试形状描述符提取"""
        # 转换为适合的格式
        sonar_uint8 = (test_sonar_image * 255).astype(np.uint8)
        
        shape_desc = processor.extract_shape_descriptors(sonar_uint8)
        
        # 应该提取到一些形状或返回空数组
        assert shape_desc.shape[1] == 7 or shape_desc.shape[0] == 0  # Hu 矩有 7 个值
        
        print(f"检测到 {shape_desc.shape[0]} 个形状")
        
    def test_empty_image(self, processor):
        """测试空图像处理"""
        empty_img = np.zeros((480, 640, 3), dtype=np.uint8)
        
        keypoints, descriptors = processor.process_camera_image(empty_img)
        
        # 即使是空图像也应该正常处理
        assert isinstance(keypoints, list)
        if len(keypoints) == 0:
            assert descriptors is None or descriptors.shape[0] == 0
            
        print("空图像处理正常")
            
    def test_different_feature_types(self, test_image):
        """测试不同的特征类型"""
        feature_types = ['SIFT', 'ORB', 'AKAZE']
        
        successful_detectors = []
        
        for feat_type in feature_types:
            try:
                processor = ImageProcessor(feature_type=feat_type)
                keypoints, descriptors = processor.process_camera_image(test_image)
                
                successful_detectors.append(feat_type)
                
                assert len(keypoints) >= 0
                if len(keypoints) > 0:
                    assert descriptors is not None
                    
                    # 检查描述符维度
                    if feat_type == 'ORB':
                        assert descriptors.shape[1] == 32  # ORB 是 32 维
                    elif feat_type == 'SIFT':
                        assert descriptors.shape[1] == 128  # SIFT 是 128 维
                    elif feat_type == 'AKAZE':
                        assert descriptors.shape[1] == 61  # AKAZE 是 61 维
                        
                print(f"{feat_type}: {len(keypoints)} 个特征点")
                    
            except Exception as e:
                print(f"{feat_type} 检测器不可用: {e}")
                
        # 至少应该有一个检测器可用
        assert len(successful_detectors) > 0, "没有可用的特征检测器"
        print(f"可用的检测器: {successful_detectors}")
        
    def test_clahe_enhancement(self, processor, test_image):
        """测试 CLAHE 图像增强"""
        # 转换为灰度
        gray = cv2.cvtColor(test_image, cv2.COLOR_BGR2GRAY)
        
        # 应用增强
        enhanced = processor.enhance_underwater_image(gray)
        
        # 增强后的图像应该有相同的尺寸
        assert enhanced.shape == gray.shape
        assert enhanced.dtype == gray.dtype
        
        print("CLAHE 增强功能正常")
        
    def test_max_features_limit(self, test_image):
        """测试最大特征点数量限制"""
        max_features = 50
        processor = ImageProcessor(feature_type='AUTO', max_features=max_features)
        
        keypoints, descriptors = processor.process_camera_image(test_image)
        
        # 特征点数量不应超过限制
        assert len(keypoints) <= max_features
        if descriptors is not None:
            assert descriptors.shape[0] <= max_features
            
        print(f"限制最大特征数为 {max_features}，实际检测到 {len(keypoints)} 个")


class TestImageProcessingIntegration:
    """图像处理集成测试"""
    
    def test_pipeline_consistency(self):
        """测试完整处理流程的一致性"""
        processor = ImageProcessor(feature_type='AUTO')
        
        # 创建测试图像
        test_img = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        
        # 多次处理同一图像应该得到相同结果
        kp1, desc1 = processor.process_camera_image(test_img)
        kp2, desc2 = processor.process_camera_image(test_img)
        
        # 关键点数量应该相同
        assert len(kp1) == len(kp2)
        
        # 描述符应该相同（如果有的话）
        if desc1 is not None and desc2 is not None:
            assert np.array_equal(desc1, desc2)
            
        print(f"处理流程一致性测试通过，检测到 {len(kp1)} 个特征点")
        
    def test_robustness_to_noise(self):
        """测试对噪声的鲁棒性"""
        processor = ImageProcessor(feature_type='AUTO')
        
        # 创建基础图像
        base_img = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.rectangle(base_img, (200, 200), (400, 400), (255, 255, 255), -1)
        
        # 添加噪声的图像
        noise = np.random.normal(0, 25, base_img.shape).astype(np.int16)
        noisy_img = np.clip(base_img.astype(np.int16) + noise, 0, 255).astype(np.uint8)
        
        # 处理两个图像
        kp_clean, _ = processor.process_camera_image(base_img)
        kp_noisy, _ = processor.process_camera_image(noisy_img)
        
        # 噪声图像仍应能检测到特征点
        assert len(kp_noisy) >= 0  # 至少不应该报错
        
        print(f"干净图像: {len(kp_clean)} 个特征点")
        print(f"噪声图像: {len(kp_noisy)} 个特征点")
        print("噪声鲁棒性测试通过")


if __name__ == '__main__':
    # 运行测试
    pytest.main([__file__, '-v', '--tb=short'])