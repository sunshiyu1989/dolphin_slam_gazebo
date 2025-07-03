#!/usr/bin/env python3
"""
Dolphin SLAM - 经验地图模块
构建和维护拓扑-度量混合地图
"""

import numpy as np
from typing import List, Tuple, Optional, Dict
import logging
from dataclasses import dataclass
from collections import defaultdict

@dataclass
class Experience:
    """经验节点数据结构"""
    id: int
    x: float  # 度量位置
    y: float
    z: float
    theta: float  # 朝向
    visual_template_id: int  # 关联的视觉模板
    place_cell_center: Tuple[float, float, float]  # 位置细胞活动中心
    creation_time: float
    last_update_time: float
    visit_count: int = 1
    
class ExperienceMap:
    """
    经验地图 - 管理拓扑-度量混合地图
    """
    
    def __init__(self,
                 match_threshold: float = 0.75,
                 lv_factor: float = 0.5,
                 pc_factor: float = 0.5,
                 min_experience_age: int = 5,
                 loop_closure_threshold: float = 0.8):
        """
        初始化经验地图
        
        参数:
            match_threshold: 经验匹配阈值
            lv_factor: 局部视觉权重因子
            pc_factor: 位置细胞权重因子
            min_experience_age: 最小经验年龄（用于闭环检测）
            loop_closure_threshold: 闭环检测阈值
        """
        self.match_threshold = match_threshold
        self.lv_factor = lv_factor
        self.pc_factor = pc_factor
        self.min_experience_age = min_experience_age
        self.loop_closure_threshold = loop_closure_threshold
        
        self.logger = logging.getLogger(__name__)
        
        # 经验存储
        self.experiences: Dict[int, Experience] = {}
        self.next_experience_id = 0
        
        # 拓扑连接（邻接表）
        self.connections: Dict[int, List[int]] = defaultdict(list)
        
        # 当前经验
        self.current_experience_id: Optional[int] = None
        
        # 经验索引（用于快速查找）
        self.visual_template_index: Dict[int, List[int]] = defaultdict(list)
        
        # 地图统计
        self.total_distance_traveled = 0.0
        self.loop_closures_detected = 0
        
    def create_experience(self, x: float, y: float, z: float, theta: float,
                         visual_template_id: int, 
                         place_cell_center: Tuple[float, float, float],
                         current_time: float) -> int:
        """
        创建新的经验节点
        
        参数:
            x, y, z: 度量位置
            theta: 朝向
            visual_template_id: 视觉模板 ID
            place_cell_center: 位置细胞活动中心
            current_time: 当前时间
            
        返回:
            experience_id: 新经验的 ID
        """
        exp = Experience(
            id=self.next_experience_id,
            x=x, y=y, z=z, theta=theta,
            visual_template_id=visual_template_id,
            place_cell_center=place_cell_center,
            creation_time=current_time,
            last_update_time=current_time
        )
        
        self.experiences[exp.id] = exp
        self.visual_template_index[visual_template_id].append(exp.id)
        
        # 如果有当前经验，创建连接
        if self.current_experience_id is not None:
            self._add_connection(self.current_experience_id, exp.id)
            
        self.current_experience_id = exp.id
        self.next_experience_id += 1
        
        self.logger.info(f"创建新经验 #{exp.id} at ({x:.2f}, {y:.2f}, {z:.2f})")
        
        return exp.id
        
    def update_experience(self, experience_id: int, x: float, y: float, z: float,
                         theta: float, current_time: float):
        """
        更新现有经验的位置
        
        参数:
            experience_id: 经验 ID
            x, y, z: 新的度量位置
            theta: 新的朝向
            current_time: 当前时间
        """
        if experience_id not in self.experiences:
            return
            
        exp = self.experiences[experience_id]
        
        # 使用加权平均更新位置
        alpha = 0.1  # 学习率
        exp.x = (1 - alpha) * exp.x + alpha * x
        exp.y = (1 - alpha) * exp.y + alpha * y
        exp.z = (1 - alpha) * exp.z + alpha * z
        exp.theta = self._circular_mean(exp.theta, theta, 1 - alpha, alpha)
        
        exp.last_update_time = current_time
        exp.visit_count += 1
        
    def find_match(self, visual_template_id: int, visual_similarity: float,
                  place_cell_center: Tuple[float, float, float],
                  current_position: Tuple[float, float, float]) -> Optional[int]:
        """
        查找匹配的经验
        
        参数:
            visual_template_id: 视觉模板 ID
            visual_similarity: 视觉相似度
            place_cell_center: 位置细胞活动中心
            current_position: 当前估计位置
            
        返回:
            matched_experience_id: 匹配的经验 ID（如果没有则为 None）
        """
        # 首先检查是否有相同视觉模板的经验
        candidate_experiences = self.visual_template_index.get(visual_template_id, [])
        
        if not candidate_experiences:
            return None
            
        best_match_id = None
        best_match_score = 0.0
        
        for exp_id in candidate_experiences:
            exp = self.experiences[exp_id]
            
            # 计算位置细胞距离
            pc_distance = np.linalg.norm(
                np.array(place_cell_center) - np.array(exp.place_cell_center)
            )
            pc_similarity = np.exp(-pc_distance / 10.0)  # 衰减因子
            
            # 计算度量距离（作为额外验证）
            metric_distance = np.sqrt(
                (current_position[0] - exp.x)**2 +
                (current_position[1] - exp.y)**2 +
                (current_position[2] - exp.z)**2
            )
            
            # 组合相似度
            combined_similarity = (self.lv_factor * visual_similarity + 
                                 self.pc_factor * pc_similarity)
            
            # 考虑度量距离的惩罚
            if metric_distance > 10.0:  # 10米以上距离降低匹配可能性
                combined_similarity *= 0.5
                
            if combined_similarity > best_match_score:
                best_match_score = combined_similarity
                best_match_id = exp_id
                
        if best_match_score > self.match_threshold:
            self.logger.info(f"找到匹配经验 #{best_match_id}，相似度: {best_match_score:.3f}")
            return best_match_id
        else:
            return None
            
    def detect_loop_closure(self, current_experience_id: int) -> List[Tuple[int, float]]:
        """
        检测闭环
        
        参数:
            current_experience_id: 当前经验 ID
            
        返回:
            loop_closures: [(经验ID, 相似度)] 列表
        """
        if current_experience_id not in self.experiences:
            return []
            
        current_exp = self.experiences[current_experience_id]
        loop_closures = []
        
        # 遍历所有足够老的经验
        for exp_id, exp in self.experiences.items():
            # 跳过自己和最近的经验
            if (exp_id == current_experience_id or 
                abs(exp_id - current_experience_id) < self.min_experience_age):
                continue
                
            # 计算度量距离
            distance = np.sqrt(
                (current_exp.x - exp.x)**2 +
                (current_exp.y - exp.y)**2 +
                (current_exp.z - exp.z)**2
            )
            
            # 如果距离足够近，检查视觉相似度
            if distance < 5.0:  # 5米阈值
                # 这里简化处理，实际应该重新计算视觉相似度
                if current_exp.visual_template_id == exp.visual_template_id:
                    similarity = 0.9  # 相同模板给高相似度
                else:
                    similarity = 0.3  # 不同模板给低相似度
                    
                if similarity > self.loop_closure_threshold:
                    loop_closures.append((exp_id, similarity))
                    self.logger.info(f"检测到闭环: 经验 #{current_experience_id} <-> #{exp_id}")
                    
        if loop_closures:
            self.loop_closures_detected += len(loop_closures)
            
        return loop_closures
        
    def _add_connection(self, exp1_id: int, exp2_id: int):
        """添加两个经验之间的连接"""
        if exp2_id not in self.connections[exp1_id]:
            self.connections[exp1_id].append(exp2_id)
        if exp1_id not in self.connections[exp2_id]:
            self.connections[exp2_id].append(exp1_id)
            
    def _circular_mean(self, angle1: float, angle2: float, 
                      weight1: float, weight2: float) -> float:
        """计算圆形量（角度）的加权平均"""
        x = weight1 * np.cos(angle1) + weight2 * np.cos(angle2)
        y = weight1 * np.sin(angle1) + weight2 * np.sin(angle2)
        return np.arctan2(y, x)
        
    def get_path(self, start_id: int, goal_id: int) -> Optional[List[int]]:
        """
        使用 A* 算法在拓扑图中寻找路径
        
        参数:
            start_id: 起始经验 ID
            goal_id: 目标经验 ID
            
        返回:
            path: 经验 ID 列表（如果没有路径则为 None）
        """
        if start_id not in self.experiences or goal_id not in self.experiences:
            return None
            
        # A* 算法实现
        from heapq import heappush, heappop
        
        def heuristic(exp1_id: int, exp2_id: int) -> float:
            """启发式函数：欧几里得距离"""
            exp1 = self.experiences[exp1_id]
            exp2 = self.experiences[exp2_id]
            return np.sqrt((exp1.x - exp2.x)**2 + 
                          (exp1.y - exp2.y)**2 + 
                          (exp1.z - exp2.z)**2)
            
        # 初始化
        open_set = [(0, start_id)]
        came_from = {}
        g_score = {start_id: 0}
        f_score = {start_id: heuristic(start_id, goal_id)}
        
        while open_set:
            current_f, current = heappop(open_set)
            
            if current == goal_id:
                # 重建路径
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                return path[::-1]
                
            for neighbor in self.connections[current]:
                # 计算到邻居的代价
                tentative_g = g_score[current] + heuristic(current, neighbor)
                
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + heuristic(neighbor, goal_id)
                    heappush(open_set, (f_score[neighbor], neighbor))
                    
        return None  # 没有找到路径
        
    def get_map_bounds(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        获取地图边界
        
        返回:
            min_bounds: [x_min, y_min, z_min]
            max_bounds: [x_max, y_max, z_max]
        """
        if not self.experiences:
            return np.zeros(3), np.zeros(3)
            
        positions = np.array([[exp.x, exp.y, exp.z] 
                             for exp in self.experiences.values()])
        
        return np.min(positions, axis=0), np.max(positions, axis=0)
        
    def get_experience_positions(self) -> np.ndarray:
        """
        获取所有经验的位置
        
        返回:
            positions: Nx3 数组
        """
        if not self.experiences:
            return np.array([]).reshape(0, 3)
            
        return np.array([[exp.x, exp.y, exp.z] 
                        for exp in self.experiences.values()])
        
    def get_connections_list(self) -> List[Tuple[int, int]]:
        """
        获取所有连接的列表
        
        返回:
            connections: [(exp1_id, exp2_id), ...] 列表
        """
        connections = []
        seen = set()
        
        for exp1_id, neighbors in self.connections.items():
            for exp2_id in neighbors:
                if (exp1_id, exp2_id) not in seen and (exp2_id, exp1_id) not in seen:
                    connections.append((exp1_id, exp2_id))
                    seen.add((exp1_id, exp2_id))
                    
        return connections
        
    def save_map(self, filename: str):
        """保存地图到文件"""
        import pickle
        
        map_data = {
            'experiences': self.experiences,
            'connections': dict(self.connections),
            'next_experience_id': self.next_experience_id,
            'visual_template_index': dict(self.visual_template_index),
            'total_distance_traveled': self.total_distance_traveled,
            'loop_closures_detected': self.loop_closures_detected
        }
        
        with open(filename, 'wb') as f:
            pickle.dump(map_data, f)
            
        self.logger.info(f"地图已保存到 {filename}")
        
    def load_map(self, filename: str):
        """从文件加载地图"""
        import pickle
        
        with open(filename, 'rb') as f:
            map_data = pickle.load(f)
            
        self.experiences = map_data['experiences']
        self.connections = defaultdict(list, map_data['connections'])
        self.next_experience_id = map_data['next_experience_id']
        self.visual_template_index = defaultdict(list, map_data['visual_template_index'])
        self.total_distance_traveled = map_data['total_distance_traveled']
        self.loop_closures_detected = map_data['loop_closures_detected']
        
        self.logger.info(f"地图已从 {filename} 加载")
