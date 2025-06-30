#!/usr/bin/env python3
"""
Dolphin SLAM - 局部视觉细胞模块
使用 FAB-MAP 算法进行场景识别和管理视觉模板
"""

import numpy as np
from typing import List, Tuple, Optional, Dict
import logging
from collections import deque
import cv2
from sklearn.cluster import KMeans
import pickle

class LocalViewCells:
    """
    局部视觉细胞 - 管理视觉模板和场景识别
    """
    
    def __init__(self,
                 matching_algorithm: str = 'fabmap',
                 similarity_threshold: float = 0.65,
                 vocabulary_size: int = 1000,
                 max_templates: int = 5000,
                 template_decay_rate: float = 0.995):
        """
        初始化局部视觉细胞
        
        参数:
            matching_algorithm: 匹配算法 ('fabmap' 或 'bow')
            similarity_threshold: 相似度阈值
            vocabulary_size: 视觉词汇表大小
            max_templates: 最大模板数量
            template_decay_rate: 模板衰减率
        """
        self.matching_algorithm = matching_algorithm
        self.similarity_threshold = similarity_threshold
        self.vocabulary_size = vocabulary_size
        self.max_templates = max_templates
        self.template_decay_rate = template_decay_rate
        
        self.logger = logging.getLogger(__name__)
        
        # 视觉模板存储
        self.templates: Dict[int, VisualTemplate] = {}
        self.next_template_id = 0
        
        # 当前活跃的模板
        self.current_template_id: Optional[int] = None
        
        # 视觉词汇表
        self.vocabulary: Optional[np.ndarray] = None
        self.bow_extractor: Optional[cv2.BOWImgDescriptorExtractor] = None
        
        # FAB-MAP 相关
        self.fabmap_model: Optional[FabmapModel] = None
        
        # 模板激活历史
        self.activation_history = deque(maxlen=100)
        
        # 统计信息
        self.total_comparisons = 0
        self.successful_matches = 0
        
    def train_vocabulary(self, descriptors_list: List[np.ndarray]):
        """
        训练视觉词汇表
        
        参数:
            descriptors_list: 描述符列表
        """
        self.logger.info(f"开始训练视觉词汇表，目标大小: {self.vocabulary_size}")
        
        # 合并所有描述符
        all_descriptors = np.vstack(descriptors_list)
        
        # 使用 K-means 聚类创建词汇表
        kmeans = KMeans(n_clusters=self.vocabulary_size, 
                       random_state=42,
                       n_init=10)
        kmeans.fit(all_descriptors.astype(np.float32))
        
        self.vocabulary = kmeans.cluster_centers_
        
        # 创建 BoW 提取器
        FLANN_INDEX_KDTREE = 1
        index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
        search_params = dict(checks=50)
        flann = cv2.FlannBasedMatcher(index_params, search_params)
        
        self.bow_extractor = cv2.BOWImgDescriptorExtractor(
            cv2.SIFT_create(), flann)
        self.bow_extractor.setVocabulary(self.vocabulary.astype(np.float32))
        
        self.logger.info("视觉词汇表训练完成")
        
    def add_template(self, descriptors: np.ndarray, 
                    bow_descriptor: Optional[np.ndarray] = None) -> int:
        """
        添加新的视觉模板
        
        参数:
            descriptors: 原始描述符
            bow_descriptor: BoW 描述符（可选）
            
        返回:
            template_id: 新模板的 ID
        """
        # 如果没有提供 BoW 描述符，计算它
        if bow_descriptor is None and self.vocabulary is not None:
            bow_descriptor = self._compute_bow_descriptor(descriptors)
            
        template = VisualTemplate(
            id=self.next_template_id,
            descriptors=descriptors,
            bow_descriptor=bow_descriptor,
            activation_count=1,
            activation_strength=1.0
        )
        
        self.templates[template.id] = template
        self.current_template_id = template.id
        self.next_template_id += 1
        
        # 如果超过最大模板数，删除最旧的
        if len(self.templates) > self.max_templates:
            self._prune_templates()
            
        self.logger.debug(f"添加新模板 #{template.id}")
        
        return template.id
        
    def find_match(self, descriptors: np.ndarray) -> Tuple[Optional[int], float]:
        """
        查找最匹配的视觉模板
        
        参数:
            descriptors: 查询描述符
            
        返回:
            (template_id, similarity): 最佳匹配的模板 ID 和相似度
        """
        if not self.templates:
            return None, 0.0
            
        self.total_comparisons += 1
        
        if self.matching_algorithm == 'fabmap':
            return self._fabmap_match(descriptors)
        else:  # bow
            return self._bow_match(descriptors)
            
    def _bow_match(self, descriptors: np.ndarray) -> Tuple[Optional[int], float]:
        """使用 BoW 进行匹配"""
        if self.vocabulary is None:
            return None, 0.0
            
        # 计算查询的 BoW 描述符
        query_bow = self._compute_bow_descriptor(descriptors)
        
        best_match_id = None
        best_similarity = 0.0
        
        for template_id, template in self.templates.items():
            if template.bow_descriptor is None:
                continue
                
            # 计算余弦相似度
            similarity = self._cosine_similarity(query_bow, template.bow_descriptor)
            
            if similarity > best_similarity:
                best_similarity = similarity
                best_match_id = template_id
                
        if best_similarity > self.similarity_threshold:
            self.successful_matches += 1
            # 更新模板激活
            self._update_template_activation(best_match_id, best_similarity)
            return best_match_id, best_similarity
        else:
            return None, best_similarity
            
    def _fabmap_match(self, descriptors: np.ndarray) -> Tuple[Optional[int], float]:
        """使用 FAB-MAP 进行匹配"""
        if self.fabmap_model is None:
            # 简化的 FAB-MAP 实现
            return self._simplified_fabmap_match(descriptors)
            
        # 完整 FAB-MAP 实现（需要额外的依赖）
        # 这里提供接口，实际实现需要 OpenFABMAP
        pass
        
    def _simplified_fabmap_match(self, descriptors: np.ndarray) -> Tuple[Optional[int], float]:
        """简化的 FAB-MAP 匹配（基于概率）"""
        if self.vocabulary is None:
            return self._bow_match(descriptors)  # 退化为 BoW
            
        # 计算观察似然
        query_bow = self._compute_bow_descriptor(descriptors)
        
        best_match_id = None
        best_probability = 0.0
        
        for template_id, template in self.templates.items():
            if template.bow_descriptor is None:
                continue
                
            # 计算贝叶斯概率（简化版）
            # P(Li|Zk) ∝ P(Zk|Li) * P(Li)
            
            # 似然项 P(Zk|Li)
            likelihood = self._compute_likelihood(query_bow, template.bow_descriptor)
            
            # 先验项 P(Li) - 基于模板的激活历史
            prior = template.activation_strength / len(self.templates)
            
            # 后验概率
            posterior = likelihood * prior
            
            if posterior > best_probability:
                best_probability = posterior
                best_match_id = template_id
                
        # 转换概率到相似度分数
        similarity = best_probability
        
        if similarity > self.similarity_threshold:
            self.successful_matches += 1
            self._update_template_activation(best_match_id, similarity)
            return best_match_id, similarity
        else:
            return None, similarity
            
    def _compute_bow_descriptor(self, descriptors: np.ndarray) -> np.ndarray:
        """计算 BoW 描述符"""
        if self.bow_extractor is not None:
            # 使用 OpenCV 的 BoW 提取器
            keypoints = [cv2.KeyPoint(x=0, y=0, size=1) for _ in range(len(descriptors))]
            bow_descriptor = self.bow_extractor.compute(
                np.zeros((100, 100), dtype=np.uint8), 
                keypoints, 
                descriptors.astype(np.float32)
            )
            return bow_descriptor[0] if bow_descriptor is not None else np.zeros(self.vocabulary_size)
        else:
            # 手动计算
            return self._manual_bow_computation(descriptors)
            
    def _manual_bow_computation(self, descriptors: np.ndarray) -> np.ndarray:
        """手动计算 BoW 描述符"""
        if self.vocabulary is None:
            return np.zeros(0)
            
        bow_descriptor = np.zeros(self.vocabulary_size)
        
        for desc in descriptors:
            # 找到最近的视觉词
            distances = np.linalg.norm(self.vocabulary - desc, axis=1)
            nearest_word = np.argmin(distances)
            bow_descriptor[nearest_word] += 1
            
        # 归一化
        if np.sum(bow_descriptor) > 0:
            bow_descriptor = bow_descriptor / np.sum(bow_descriptor)
            
        return bow_descriptor
        
    def _cosine_similarity(self, vec1: np.ndarray, vec2: np.ndarray) -> float:
        """计算余弦相似度"""
        dot_product = np.dot(vec1, vec2)
        norm1 = np.linalg.norm(vec1)
        norm2 = np.linalg.norm(vec2)
        
        if norm1 == 0 or norm2 == 0:
            return 0.0
            
        return dot_product / (norm1 * norm2)
        
    def _compute_likelihood(self, observation: np.ndarray, 
                          template: np.ndarray) -> float:
        """计算观察似然（简化版）"""
        # 使用负指数距离作为似然
        distance = np.linalg.norm(observation - template)
        likelihood = np.exp(-distance)
        return likelihood
        
    def _update_template_activation(self, template_id: int, activation: float):
        """更新模板激活"""
        if template_id in self.templates:
            template = self.templates[template_id]
            template.activation_count += 1
            template.activation_strength = (
                template.activation_strength * 0.9 + activation * 0.1
            )
            template.last_activation_time = self.total_comparisons
            
        # 记录激活历史
        self.activation_history.append(template_id)
        
        # 衰减所有其他模板
        for tid, tmpl in self.templates.items():
            if tid != template_id:
                tmpl.activation_strength *= self.template_decay_rate
                
    def _prune_templates(self):
        """修剪模板数据库"""
        # 按激活强度排序
        sorted_templates = sorted(
            self.templates.items(),
            key=lambda x: x[1].activation_strength,
            reverse=True
        )
        
        # 保留前 N 个
        keep_ids = [tid for tid, _ in sorted_templates[:self.max_templates]]
        
        # 删除其余的
        self.templates = {
            tid: tmpl for tid, tmpl in self.templates.items()
            if tid in keep_ids
        }
        
    def get_active_templates(self, top_k: int = 10) -> List[Tuple[int, float]]:
        """
        获取最活跃的模板
        
        参数:
            top_k: 返回前 k 个
            
        返回:
            [(template_id, activation_strength), ...]
        """
        sorted_templates = sorted(
            [(tid, tmpl.activation_strength) 
             for tid, tmpl in self.templates.items()],
            key=lambda x: x[1],
            reverse=True
        )
        
        return sorted_templates[:top_k]
        
    def save_vocabulary(self, filename: str):
        """保存视觉词汇表"""
        if self.vocabulary is not None:
            np.save(filename, self.vocabulary)
            self.logger.info(f"词汇表已保存到 {filename}")
            
    def load_vocabulary(self, filename: str):
        """加载视觉词汇表"""
        self.vocabulary = np.load(filename)
        self.logger.info(f"词汇表已从 {filename} 加载")
        
        # 重新创建 BoW 提取器
        FLANN_INDEX_KDTREE = 1
        index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
        search_params = dict(checks=50)
        flann = cv2.FlannBasedMatcher(index_params, search_params)
        
        self.bow_extractor = cv2.BOWImgDescriptorExtractor(
            cv2.SIFT_create(), flann)
        self.bow_extractor.setVocabulary(self.vocabulary.astype(np.float32))
        
    def get_statistics(self) -> Dict:
        """获取统计信息"""
        return {
            'total_templates': len(self.templates),
            'total_comparisons': self.total_comparisons,
            'successful_matches': self.successful_matches,
            'match_rate': self.successful_matches / max(1, self.total_comparisons),
            'current_template': self.current_template_id,
            'vocabulary_size': len(self.vocabulary) if self.vocabulary is not None else 0
        }


class VisualTemplate:
    """视觉模板数据结构"""
    def __init__(self, id: int, descriptors: np.ndarray,
                 bow_descriptor: Optional[np.ndarray] = None,
                 activation_count: int = 0,
                 activation_strength: float = 1.0):
        self.id = id
        self.descriptors = descriptors
        self.bow_descriptor = bow_descriptor
        self.activation_count = activation_count
        self.activation_strength = activation_strength
        self.last_activation_time = 0
        self.creation_time = 0


class FabmapModel:
    """FAB-MAP 模型接口（需要额外实现）"""
    def __init__(self):
        pass
        
    def add_place(self, bow_descriptor: np.ndarray):
        """添加新地点"""
        pass
        
    def compare(self, bow_descriptor: np.ndarray) -> List[float]:
        """比较与所有已知地点"""
        pass
