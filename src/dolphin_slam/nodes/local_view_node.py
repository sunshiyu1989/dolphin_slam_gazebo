#!/usr/bin/env python3
"""
水下环境优化的局部视觉细胞 ROS2 节点
- 修复 list index out of range 错误
- 优化水下环境的匹配策略
- 时间序列优先匹配
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import MarkerArray, Marker
from cv_bridge import CvBridge
import numpy as np
import cv2
from typing import List, Tuple, Optional
import time
from collections import deque

class VisualTemplate:
    """水下环境优化的视觉模板类"""
    def __init__(self, template_id: int, descriptors: np.ndarray, timestamp: float):
        self.template_id = template_id
        self.descriptors = descriptors
        self.timestamp = timestamp
        self.activation_count = 0
        self.last_activation = timestamp
        self.creation_time = timestamp
        
        # 预计算统计信息
        self.mean_descriptor = np.mean(descriptors, axis=0)
        self.num_features = descriptors.shape[0]
        
        # 水下环境特定：时间权重（越新的模板权重越高）
        self.temporal_weight = 1.0
        
    def update_activation(self, current_time: float):
        """更新模板激活状态"""
        self.activation_count += 1
        self.last_activation = current_time
        
    def update_temporal_weight(self, current_time: float, max_age_seconds: float = 60.0):
        """更新时间权重 - 水下环境优化"""
        age = current_time - self.creation_time
        # 指数衰减：越新的模板权重越高
        self.temporal_weight = np.exp(-age / max_age_seconds)

class UnderwaterLocalViewNode(Node):
    """水下环境优化的局部视觉细胞 ROS2 节点"""
    
    def __init__(self):
        super().__init__('local_view_node')
        
        # 声明参数 - 水下环境优化
        self.declare_parameters(
            namespace='',
            parameters=[
                ('descriptors_topic', '/features/descriptors'),
                ('matches_topic', '/local_view/matches'),
                ('matching_algorithm', 'temporal_feature_matching'),
                ('similarity_threshold', 0.08),        # 水下环境：降低阈值
                ('max_templates', 50),                 # 减少模板数，重点关注最近的
                ('enable_debug', True),
                ('debug_level', 1),
                ('min_match_count', 3),                # 降低最小匹配数
                ('match_ratio_threshold', 0.8),        # 提高ratio test阈值
                ('temporal_weight_factor', 2.0),       # 时间权重系数
                ('recent_template_priority', 5),        # 优先检查最近N个模板
            ]
        )
        
        # 获取参数
        self.descriptors_topic = self.get_parameter('descriptors_topic').value
        self.matches_topic = self.get_parameter('matches_topic').value
        self.matching_algorithm = self.get_parameter('matching_algorithm').value
        self.similarity_threshold = self.get_parameter('similarity_threshold').value
        self.max_templates = self.get_parameter('max_templates').value
        self.enable_debug = self.get_parameter('enable_debug').value
        self.debug_level = self.get_parameter('debug_level').value
        self.min_match_count = self.get_parameter('min_match_count').value
        self.match_ratio_threshold = self.get_parameter('match_ratio_threshold').value
        self.temporal_weight_factor = self.get_parameter('temporal_weight_factor').value
        self.recent_template_priority = self.get_parameter('recent_template_priority').value
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # 视觉模板库 - 水下优化：按时间排序
        self.templates: List[VisualTemplate] = []
        self.template_counter = 0
        
        # 统计信息
        self.descriptor_count = 0
        self.match_count = 0
        self.successful_matches = 0
        self.temporal_matches = 0  # 时间优先匹配成功数
        self.list_index_errors = 0  # 记录list index错误
        
        # 水下环境特定统计
        self.adjacent_frame_similarities = []  # 相邻帧相似度
        self.temporal_gaps = []                # 时间间隔
        
        # 创建特征匹配器 - 水下环境优化
        self.matcher = cv2.BFMatcher(cv2.NORM_L2, crossCheck=False)
        
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
        
        # 定时报告 - 水下环境特定
        self.report_timer = self.create_timer(20.0, self.underwater_debug_report)
        
        self.get_logger().info(f'🌊 水下环境优化的局部视觉细胞节点已启动')
        self.get_logger().info(f'相似度阈值: {self.similarity_threshold}, 最小匹配数: {self.min_match_count}')
        self.get_logger().info(f'优先检查最近 {self.recent_template_priority} 个模板')
        
    def descriptors_callback(self, msg: Image):
        """水下环境优化的描述符处理"""
        try:
            self.descriptor_count += 1
            current_time = time.time()
            
            # 解码描述符
            descriptors = self.decode_descriptors_safe(msg)
            
            if descriptors is None or len(descriptors) == 0:
                if self.debug_level >= 2:
                    self.get_logger().warn(f'❌ 无效描述符 #{self.descriptor_count}')
                return
            
            # 检查描述符数量是否过少
            if len(descriptors) < 5:
                if self.debug_level >= 2:
                    self.get_logger().warn(f'⚠️ 特征数量过少: {len(descriptors)}')
                return
                
            if self.debug_level >= 2:
                self.get_logger().info(
                    f'🌊 描述符 #{self.descriptor_count}: {descriptors.shape}, '
                    f'范围=[{np.min(descriptors):.1f}, {np.max(descriptors):.1f}]'
                )
            
            # 更新所有模板的时间权重
            self.update_temporal_weights(current_time)
            
            # 执行水下环境优化的视觉匹配
            match_result = self.perform_underwater_matching(descriptors, current_time)
            
            # 发布匹配结果
            self.publish_match_result(match_result)
            
            # 更新统计
            self.match_count += 1
            
            if self.debug_level >= 1 and self.match_count % 100 == 0:
                self.get_logger().info(f'🌊 水下匹配进度: {self.match_count}次')
                
        except Exception as e:
            self.list_index_errors += 1
            if self.debug_level >= 1:
                self.get_logger().error(f'❌ 描述符处理异常 #{self.list_index_errors}: {e}')
            
    def decode_descriptors_safe(self, msg: Image) -> Optional[np.ndarray]:
        """安全的描述符解码 - 增强错误处理"""
        try:
            if msg.encoding != '32FC1' or msg.height == 0 or msg.width == 0 or len(msg.data) == 0:
                return None
                
            expected_size = msg.height * msg.width * 4
            if len(msg.data) != expected_size:
                return None
                
            data = np.frombuffer(msg.data, dtype=np.float32)
            descriptors = data.reshape(msg.height, msg.width)
            
            # 检查数据有效性
            if np.any(np.isnan(descriptors)) or np.any(np.isinf(descriptors)):
                return None
            
            return descriptors
            
        except Exception as e:
            if self.debug_level >= 2:
                self.get_logger().error(f'描述符解码失败: {e}')
            return None
    
    def update_temporal_weights(self, current_time: float):
        """更新所有模板的时间权重"""
        for template in self.templates:
            template.update_temporal_weight(current_time)
    
    def perform_underwater_matching(self, descriptors: np.ndarray, current_time: float) -> dict:
        """水下环境优化的视觉匹配"""
        
        # 如果没有模板，创建第一个模板
        if len(self.templates) == 0:
            return self.create_new_template(descriptors, current_time)
        
        # 水下环境策略1：优先检查最近的模板
        recent_templates = self.templates[-self.recent_template_priority:]
        
        best_match_id = -1
        best_similarity = 0.0
        best_match_count = 0
        is_temporal_match = False
        
        # 首先检查最近的模板（时间优先策略）
        for template in reversed(recent_templates):  # 从最新开始
            try:
                similarity, match_count = self.safe_feature_matching(descriptors, template.descriptors)
                
                # 应用时间权重加成
                weighted_similarity = similarity * (1.0 + self.temporal_weight_factor * template.temporal_weight)
                
                if self.debug_level >= 2:
                    self.get_logger().info(
                        f'  最近模板#{template.template_id}: 原始相似度={similarity:.4f}, '
                        f'时间权重={template.temporal_weight:.3f}, 加权相似度={weighted_similarity:.4f}, 匹配数={match_count}'
                    )
                
                if weighted_similarity > best_similarity:
                    best_similarity = weighted_similarity
                    best_match_id = template.template_id
                    best_match_count = match_count
                    is_temporal_match = True
                    
            except Exception as e:
                if self.debug_level >= 2:
                    self.get_logger().error(f'最近模板匹配失败: {e}')
        
        # 如果最近模板匹配成功，直接返回（水下环境优化）
        if (best_similarity > self.similarity_threshold and 
            best_match_count >= self.min_match_count and 
            is_temporal_match):
            
            self.successful_matches += 1
            self.temporal_matches += 1
            
            matched_template = next(t for t in self.templates if t.template_id == best_match_id)
            matched_template.update_activation(current_time)
            
            # 记录相邻帧相似度（仅当匹配最新模板时）
            if matched_template == self.templates[-1]:
                self.adjacent_frame_similarities.append(best_similarity)
                if len(self.templates) > 1:
                    time_gap = current_time - self.templates[-1].creation_time
                    self.temporal_gaps.append(time_gap)
            
            if self.debug_level >= 1:
                self.get_logger().info(
                    f'✅ 水下时间优先匹配: 模板#{best_match_id}, 相似度={best_similarity:.4f}, 匹配数={best_match_count}'
                )
            
            return {
                'matched': True,
                'template_id': best_match_id,
                'similarity': best_similarity,
                'match_count': best_match_count,
                'is_temporal': True,
                'is_novel': False
            }
        
        # 如果最近模板匹配失败，检查所有模板（但权重较低）
        for template in self.templates[:-self.recent_template_priority]:
            try:
                similarity, match_count = self.safe_feature_matching(descriptors, template.descriptors)
                
                if similarity > best_similarity:
                    best_similarity = similarity
                    best_match_id = template.template_id
                    best_match_count = match_count
                    is_temporal_match = False
                    
            except Exception as e:
                if self.debug_level >= 2:
                    self.get_logger().error(f'全局模板匹配失败: {e}')
        
        # 判断是否匹配成功
        if best_similarity > self.similarity_threshold and best_match_count >= self.min_match_count:
            self.successful_matches += 1
            matched_template = next(t for t in self.templates if t.template_id == best_match_id)
            matched_template.update_activation(current_time)
            
            if self.debug_level >= 1:
                self.get_logger().info(
                    f'✅ 全局匹配成功: 模板#{best_match_id}, 相似度={best_similarity:.4f}, 匹配数={best_match_count}'
                )
            
            return {
                'matched': True,
                'template_id': best_match_id,
                'similarity': best_similarity,
                'match_count': best_match_count,
                'is_temporal': is_temporal_match,
                'is_novel': False
            }
        else:
            # 创建新模板
            if self.debug_level >= 1:
                self.get_logger().info(
                    f'📝 创建新模板: 最高相似度={best_similarity:.4f}, 最多匹配={best_match_count} '
                    f'(阈值={self.similarity_threshold}, 最小匹配={self.min_match_count})'
                )
            return self.create_new_template(descriptors, current_time)
    
    def safe_feature_matching(self, desc1: np.ndarray, desc2: np.ndarray) -> Tuple[float, int]:
        """安全的特征匹配 - 修复 list index out of range"""
        try:
            # 确保描述符是float32类型
            desc1 = desc1.astype(np.float32)
            desc2 = desc2.astype(np.float32)
            
            # 检查描述符数量
            if len(desc1) == 0 or len(desc2) == 0:
                return 0.0, 0
            
            # 检查描述符维度
            if desc1.shape[1] != desc2.shape[1]:
                return 0.0, 0
            
            # 对于很少的特征，使用简单匹配
            if len(desc1) < 3 or len(desc2) < 3:
                return self.simple_distance_matching(desc1, desc2)
            
            # 确定k值 - 防止 list index out of range
            k = min(2, len(desc2), len(desc1))
            if k < 2:
                return self.simple_distance_matching(desc1, desc2)
            
            # KNN匹配
            matches = self.matcher.knnMatch(desc1, desc2, k=k)
            
            # 安全处理匹配结果
            good_matches = []
            for match_group in matches:
                if match_group is None or len(match_group) == 0:
                    continue
                    
                if len(match_group) == 1:
                    # 只有一个匹配，直接接受
                    good_matches.append(match_group[0])
                elif len(match_group) >= 2:
                    # 应用Lowe's ratio test
                    m, n = match_group[0], match_group[1]
                    if m.distance < self.match_ratio_threshold * n.distance:
                        good_matches.append(m)
            
            match_count = len(good_matches)
            
            if match_count == 0:
                return 0.0, 0
            
            # 计算相似度
            match_ratio = match_count / min(len(desc1), len(desc2))
            avg_distance = np.mean([m.distance for m in good_matches])
            
            # 距离转相似度（水下环境优化：更宽松的距离阈值）
            distance_similarity = max(0.0, 1.0 - avg_distance / 300.0)  # 增加到300
            
            # 组合相似度
            similarity = 0.6 * match_ratio + 0.4 * distance_similarity
            
            return float(similarity), match_count
            
        except Exception as e:
            if self.debug_level >= 2:
                self.get_logger().error(f'安全特征匹配失败: {e}')
            return 0.0, 0
    
    def simple_distance_matching(self, desc1: np.ndarray, desc2: np.ndarray) -> Tuple[float, int]:
        """简单距离匹配 - 用于特征数量很少的情况"""
        try:
            # 计算平均描述符
            mean1 = np.mean(desc1, axis=0)
            mean2 = np.mean(desc2, axis=0)
            
            # 欧几里得距离
            distance = np.linalg.norm(mean1 - mean2)
            
            # 转换为相似度
            similarity = max(0.0, 1.0 - distance / 400.0)  # 水下环境：更宽松
            
            # 虚拟匹配数
            virtual_matches = min(len(desc1), len(desc2))
            
            return float(similarity), virtual_matches
            
        except Exception as e:
            if self.debug_level >= 2:
                self.get_logger().error(f'简单距离匹配失败: {e}')
            return 0.0, 0
    
    def create_new_template(self, descriptors: np.ndarray, timestamp: float) -> dict:
        """创建新的视觉模板 - 水下环境优化"""
        template = VisualTemplate(self.template_counter, descriptors.copy(), timestamp)
        self.templates.append(template)
        self.template_counter += 1
        
        # 限制模板数量 - 保留最新的
        if len(self.templates) > self.max_templates:
            removed_template = self.templates.pop(0)  # 移除最旧的
            if self.debug_level >= 1:
                self.get_logger().info(f'🗑️ 删除旧模板#{removed_template.template_id}')
        
        return {
            'matched': False,
            'template_id': template.template_id,
            'similarity': 0.0,
            'match_count': 0,
            'is_temporal': False,
            'is_novel': True
        }
    
    def publish_match_result(self, match_result: dict):
        """发布匹配结果"""
        match_msg = Float32MultiArray()
        
        # 数据格式：[相似度, 模板ID, 匹配标志, 新奇标志]
        match_msg.data = [
            float(match_result['similarity']),
            float(match_result['template_id']),
            float(1.0 if match_result['matched'] else 0.0),
            float(1.0 if match_result['is_novel'] else 0.0)
        ]
        
        self.match_pub.publish(match_msg)
    
    def underwater_debug_report(self):
        """水下环境特定的调试报告"""
        if self.match_count == 0:
            self.get_logger().info('🌊 水下环境调试: 还没有匹配数据')
            return
        
        # 基本统计
        success_rate = (self.successful_matches / self.match_count) * 100
        temporal_rate = (self.temporal_matches / self.match_count) * 100 if self.match_count > 0 else 0
        
        # 相邻帧统计
        if self.adjacent_frame_similarities:
            adj_sim_mean = np.mean(self.adjacent_frame_similarities)
            adj_sim_min = np.min(self.adjacent_frame_similarities)
        else:
            adj_sim_mean = adj_sim_min = 0.0
        
        # 时间间隔统计
        if self.temporal_gaps:
            avg_gap = np.mean(self.temporal_gaps)
        else:
            avg_gap = 0.0
        
        report = f"""
🌊 水下环境调试报告 ({self.match_count} 次匹配)
{'='*50}
基本统计:
  总成功率: {success_rate:.1f}%
  时间优先成功率: {temporal_rate:.1f}%
  模板数量: {len(self.templates)}
  处理错误: {self.list_index_errors}
  
水下环境特定:
  相邻帧平均相似度: {adj_sim_mean:.4f}
  相邻帧最低相似度: {adj_sim_min:.4f}
  平均时间间隔: {avg_gap:.2f}秒
  
诊断建议:"""
        
        if adj_sim_mean < 0.3:
            report += "\n  ⚠️ 相邻帧相似度过低，考虑降低阈值或检查特征提取"
        elif adj_sim_mean > 0.7:
            report += "\n  ✅ 相邻帧相似度良好"
        else:
            report += "\n  ℹ️ 相邻帧相似度中等"
        
        if temporal_rate > 70:
            report += "\n  ✅ 时间优先策略有效"
        else:
            report += "\n  ⚠️ 时间优先策略效果有限，考虑调整参数"
        
        if success_rate < 30:
            report += "\n  ❌ 建议降低 similarity_threshold 到 0.05"
        elif success_rate < 50:
            report += "\n  ⚠️ 建议微调 similarity_threshold 到 0.06"
        else:
            report += "\n  ✅ 匹配成功率良好"
        
        self.get_logger().info(report)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = UnderwaterLocalViewNode()
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