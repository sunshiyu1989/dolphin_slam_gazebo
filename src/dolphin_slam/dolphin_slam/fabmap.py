#!/usr/bin/env python3
"""
Dolphin SLAM - FAB-MAP 算法实现
Fast Appearance-Based Mapping 的简化 Python 实现
"""

import numpy as np
from typing import List, Tuple, Optional, Dict
import logging
from scipy.special import logsumexp
from dataclasses import dataclass
import pickle


@dataclass
class FabmapParams:
    """FAB-MAP 参数"""
    p_new_place: float = 0.9  # 新地点的先验概率
    p_see_given_exists: float = 0.39  # P(z|e)
    p_see_given_not_exists: float = 0.0  # P(z|¬e)
    p_false_positive: float = 0.01  # 误报率
    p_false_negative: float = 0.01  # 漏报率
    motion_model_std: float = 5.0  # 运动模型标准差
    
    
class ChowLiuTree:
    """Chow-Liu 树用于建模特征之间的依赖关系"""
    
    def __init__(self, num_features: int):
        self.num_features = num_features
        self.edges = []  # (parent, child) 列表
        self.root = 0
        self.parents = [-1] * num_features  # 每个节点的父节点
        self.children = [[] for _ in range(num_features)]  # 每个节点的子节点
        
    def build_from_data(self, observations: np.ndarray):
        """从观察数据构建 Chow-Liu 树"""
        # 计算互信息矩阵
        mi_matrix = self._compute_mutual_information(observations)
        
        # 使用 Kruskal 算法构建最大生成树
        edges = []
        for i in range(self.num_features):
            for j in range(i+1, self.num_features):
                edges.append((mi_matrix[i, j], i, j))
                
        edges.sort(reverse=True)  # 按互信息降序排序
        
        # Union-Find 结构
        parent = list(range(self.num_features))
        
        def find(x):
            if parent[x] != x:
                parent[x] = find(parent[x])
            return parent[x]
            
        def union(x, y):
            px, py = find(x), find(y)
            if px != py:
                parent[px] = py
                return True
            return False
            
        # 构建树
        for mi, i, j in edges:
            if union(i, j):
                self.edges.append((i, j))
                if len(self.edges) == self.num_features - 1:
                    break
                    
        # 转换为有向树（选择节点 0 作为根）
        self._convert_to_directed_tree()
        
    def _compute_mutual_information(self, observations: np.ndarray) -> np.ndarray:
        """计算特征之间的互信息"""
        n_samples, n_features = observations.shape
        mi_matrix = np.zeros((n_features, n_features))
        
        for i in range(n_features):
            for j in range(i+1, n_features):
                # 计算联合概率和边缘概率
                p_00 = np.sum((observations[:, i] == 0) & (observations[:, j] == 0)) / n_samples
                p_01 = np.sum((observations[:, i] == 0) & (observations[:, j] == 1)) / n_samples
                p_10 = np.sum((observations[:, i] == 1) & (observations[:, j] == 0)) / n_samples
                p_11 = np.sum((observations[:, i] == 1) & (observations[:, j] == 1)) / n_samples
                
                p_i0 = p_00 + p_01
                p_i1 = p_10 + p_11
                p_j0 = p_00 + p_10
                p_j1 = p_01 + p_11
                
                # 计算互信息
                mi = 0
                for pi, pj, pij in [(p_i0, p_j0, p_00), (p_i0, p_j1, p_01),
                                   (p_i1, p_j0, p_10), (p_i1, p_j1, p_11)]:
                    if pij > 0 and pi > 0 and pj > 0:
                        mi += pij * np.log(pij / (pi * pj))
                        
                mi_matrix[i, j] = mi_matrix[j, i] = mi
                
        return mi_matrix
        
    def _convert_to_directed_tree(self):
        """将无向树转换为有向树"""
        # 使用 BFS 从根节点开始
        visited = [False] * self.num_features
        queue = [self.root]
        visited[self.root] = True
        
        adj_list = [[] for _ in range(self.num_features)]
        for i, j in self.edges:
            adj_list[i].append(j)
            adj_list[j].append(i)
            
        while queue:
            node = queue.pop(0)
            for neighbor in adj_list[node]:
                if not visited[neighbor]:
                    visited[neighbor] = True
                    self.parents[neighbor] = node
                    self.children[node].append(neighbor)
                    queue.append(neighbor)
                    

class SimpleFabmap:
    """简化版 FAB-MAP 实现"""
    
    def __init__(self, params: Optional[FabmapParams] = None):
        self.params = params or FabmapParams()
        self.logger = logging.getLogger(__name__)
        
        # 地点数据库
        self.places: List[np.ndarray] = []  # 每个地点的 BoW 向量
        self.num_words = 0  # 词汇表大小
        
        # Chow-Liu 树
        self.cl_tree: Optional[ChowLiuTree] = None
        
        # 概率表
        self.word_probs: Optional[np.ndarray] = None  # P(zi|Li)
        self.word_given_word: Optional[Dict] = None  # P(zi|zj,Li)
        
        # 归一化因子
        self.normalization_constant = 0.0
        
    def train(self, training_data: List[np.ndarray]):
        """训练 FAB-MAP 模型"""
        self.logger.info(f"训练 FAB-MAP，样本数: {len(training_data)}")
        
        if not training_data:
            return
            
        # 确定词汇表大小
        self.num_words = len(training_data[0])
        
        # 转换为二进制矩阵（词是否出现）
        binary_data = np.array([bow > 0 for bow in training_data], dtype=int)
        
        # 构建 Chow-Liu 树
        self.cl_tree = ChowLiuTree(self.num_words)
        self.cl_tree.build_from_data(binary_data)
        
        # 计算词汇概率
        self._compute_word_probabilities(binary_data)
        
        # 计算条件概率
        self._compute_conditional_probabilities(binary_data)
        
        self.logger.info("FAB-MAP 训练完成")
        
    def _compute_word_probabilities(self, binary_data: np.ndarray):
        """计算词汇出现概率"""
        # P(zi=1|Li) - 在地点 i 看到词 i 的概率
        self.word_probs = np.mean(binary_data, axis=0)
        
        # 平滑处理
        epsilon = 0.01
        self.word_probs = np.clip(self.word_probs, epsilon, 1 - epsilon)
        
    def _compute_conditional_probabilities(self, binary_data: np.ndarray):
        """计算条件概率 P(zi|zj,Li)"""
        self.word_given_word = {}
        
        for child in range(self.num_words):
            parent = self.cl_tree.parents[child]
            if parent >= 0:
                # 计算 P(child|parent)
                parent_1 = binary_data[:, parent] == 1
                child_1_given_parent_1 = np.sum(
                    (binary_data[:, child] == 1) & parent_1
                ) / np.sum(parent_1)
                
                parent_0 = binary_data[:, parent] == 0
                child_1_given_parent_0 = np.sum(
                    (binary_data[:, child] == 1) & parent_0
                ) / np.sum(parent_0)
                
                self.word_given_word[(child, parent)] = {
                    'p11': child_1_given_parent_1,
                    'p10': child_1_given_parent_0
                }
                
    def add_place(self, bow_descriptor: np.ndarray) -> int:
        """添加新地点"""
        place_id = len(self.places)
        self.places.append(bow_descriptor.copy())
        return place_id
        
    def compare(self, query_bow: np.ndarray) -> List[float]:
        """
        比较查询描述符与所有已知地点
        
        返回:
            probabilities: 每个地点的概率列表
        """
        if not self.places:
            return []
            
        # 转换为二进制
        query_binary = (query_bow > 0).astype(int)
        
        # 计算每个地点的对数概率
        log_probs = []
        
        for place_id, place_bow in enumerate(self.places):
            place_binary = (place_bow > 0).astype(int)
            
            # 计算 P(Z|Li)
            log_prob = self._compute_observation_likelihood(
                query_binary, place_id
            )
            
            # 添加运动模型（简化版：基于地点间距离）
            if place_id > 0:
                distance_penalty = abs(place_id - len(self.places) + 1)
                log_prob -= (distance_penalty / self.params.motion_model_std) ** 2
                
            log_probs.append(log_prob)
            
        # 新地点的概率
        new_place_log_prob = np.log(self.params.p_new_place)
        log_probs.append(new_place_log_prob)
        
        # 归一化（从对数空间）
        log_probs = np.array(log_probs)
        probs = np.exp(log_probs - logsumexp(log_probs))
        
        return probs.tolist()
        
    def _compute_observation_likelihood(self, observation: np.ndarray, 
                                      place_id: int) -> float:
        """计算观察似然 P(Z|Li)"""
        if self.cl_tree is None or self.word_probs is None:
            # 未训练，使用简单模型
            return self._simple_likelihood(observation, place_id)
            
        # 使用 Chow-Liu 树计算
        log_likelihood = 0.0
        
        for word_idx in range(self.num_words):
            parent_idx = self.cl_tree.parents[word_idx]
            
            if parent_idx < 0:
                # 根节点
                if observation[word_idx] == 1:
                    p = self.word_probs[word_idx]
                else:
                    p = 1 - self.word_probs[word_idx]
            else:
                # 条件概率
                key = (word_idx, parent_idx)
                if key in self.word_given_word:
                    probs = self.word_given_word[key]
                    if observation[parent_idx] == 1:
                        p = probs['p11'] if observation[word_idx] == 1 else (1 - probs['p11'])
                    else:
                        p = probs['p10'] if observation[word_idx] == 1 else (1 - probs['p10'])
                else:
                    # 回退到独立概率
                    if observation[word_idx] == 1:
                        p = self.word_probs[word_idx]
                    else:
                        p = 1 - self.word_probs[word_idx]
                        
            # 避免对数下溢
            p = np.clip(p, 1e-10, 1 - 1e-10)
            log_likelihood += np.log(p)
            
        return log_likelihood
        
    def _simple_likelihood(self, observation: np.ndarray, place_id: int) -> float:
        """简单似然模型（未训练时使用）"""
        if place_id >= len(self.places):
            return np.log(1e-10)
            
        place_binary = (self.places[place_id] > 0).astype(int)
        
        # 计算匹配的词汇数
        matches = np.sum(observation == place_binary)
        total = len(observation)
        
        # 简单的二项分布模型
        p_match = 1 - self.params.p_false_positive
        p_mismatch = self.params.p_false_positive
        
        log_likelihood = (matches * np.log(p_match) + 
                         (total - matches) * np.log(p_mismatch))
        
        return log_likelihood
        
    def save(self, filename: str):
        """保存模型"""
        model_data = {
            'params': self.params,
            'places': self.places,
            'num_words': self.num_words,
            'cl_tree': self.cl_tree,
            'word_probs': self.word_probs,
            'word_given_word': self.word_given_word
        }
        
        with open(filename, 'wb') as f:
            pickle.dump(model_data, f)
            
        self.logger.info(f"FAB-MAP 模型已保存到 {filename}")
        
    def load(self, filename: str):
        """加载模型"""
        with open(filename, 'rb') as f:
            model_data = pickle.load(f)
            
        self.params = model_data['params']
        self.places = model_data['places']
        self.num_words = model_data['num_words']
        self.cl_tree = model_data['cl_tree']
        self.word_probs = model_data['word_probs']
        self.word_given_word = model_data['word_given_word']
        
        self.logger.info(f"FAB-MAP 模型已从 {filename} 加载")
        

class Fabmap2(SimpleFabmap):
    """FAB-MAP 2.0 的扩展功能"""
    
    def __init__(self, params: Optional[FabmapParams] = None):
        super().__init__(params)
        
        # FAB-MAP 2.0 特性
        self.sampled_places: List[int] = []  # 采样的地点索引
        self.inverted_index: Dict[int, List[int]] = {}  # 倒排索引
        
    def add_place_with_sampling(self, bow_descriptor: np.ndarray, 
                               sampling_rate: float = 0.1) -> int:
        """添加地点with采样（FAB-MAP 2.0）"""
        place_id = self.add_place(bow_descriptor)
        
        # 决定是否将此地点加入采样集
        if np.random.random() < sampling_rate:
            self.sampled_places.append(place_id)
            
        # 更新倒排索引
        active_words = np.where(bow_descriptor > 0)[0]
        for word_idx in active_words:
            if word_idx not in self.inverted_index:
                self.inverted_index[word_idx] = []
            self.inverted_index[word_idx].append(place_id)
            
        return place_id
        
    def compare_fast(self, query_bow: np.ndarray, top_k: int = 10) -> List[Tuple[int, float]]:
        """
        快速比较（使用倒排索引）
        
        返回:
            [(place_id, probability), ...] 前 k 个最可能的地点
        """
        # 找到候选地点
        active_words = np.where(query_bow > 0)[0]
        candidate_places = set()
        
        for word_idx in active_words:
            if word_idx in self.inverted_index:
                candidate_places.update(self.inverted_index[word_idx])
                
        # 只比较候选地点
        candidates = list(candidate_places)
        if not candidates:
            return [(len(self.places), self.params.p_new_place)]  # 新地点
            
        # 计算概率
        query_binary = (query_bow > 0).astype(int)
        results = []
        
        for place_id in candidates:
            log_prob = self._compute_observation_likelihood(query_binary, place_id)
            results.append((place_id, log_prob))
            
        # 排序并返回前 k 个
        results.sort(key=lambda x: x[1], reverse=True)
        results = results[:top_k]
        
        # 转换为概率
        if results:
            log_probs = np.array([r[1] for r in results])
            probs = np.exp(log_probs - logsumexp(log_probs))
            results = [(results[i][0], probs[i]) for i in range(len(results))]
            
        return results
