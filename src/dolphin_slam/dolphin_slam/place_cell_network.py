#!/usr/bin/env python3
"""
Dolphin SLAM - 位置细胞网络模块（修正版）
实现竞争性吸引子神经网络（CANN）用于空间表征

修正的问题：
1. step方法的参数不匹配问题
2. 路径积分不移动活动中心的问题  
3. 视觉输入年龄检查的问题
4. 初始化后没有活动的问题
"""

import numpy as np
from typing import Tuple, Optional, Union
import logging
from scipy.ndimage import gaussian_filter


class PlaceCellNetwork:
    """
    位置细胞网络 - 基于竞争性吸引子神经网络
    用于整合路径积分和视觉输入
    
    这就像海豚大脑中的空间认知系统，通过神经元的协作来维持对当前位置的估计。
    就像一群海豚通过声呐协作来定位，网络中的神经元通过相互抑制和激活来产生稳定的位置表征。
    """
    
    def __init__(self, 
                 neurons_per_dim: int = 20,
                 neurons_step: float = 0.2,
                 recurrent_conn_std: float = 3.0,
                 input_learning_rate: float = 0.1,
                 min_input_age: int = 10,
                 weight_function: str = 'mexican_hat'):
        """
        初始化位置细胞网络
        
        参数:
            neurons_per_dim: 每个维度的神经元数量（类似海豚声呐的分辨率）
            neurons_step: 神经元之间的空间步长（实际距离映射）
            recurrent_conn_std: 递归连接的标准差（神经元间影响范围）
            input_learning_rate: 输入学习率（适应新环境的速度）
            min_input_age: 最小输入年龄（建立稳定连接的最小次数）
            weight_function: 权重函数类型 ('mexican_hat' 或 'gaussian')
        """
        self.size = neurons_per_dim
        self.grid_step = neurons_step  # 重命名避免与step()方法冲突
        self.sigma = recurrent_conn_std
        self.learning_rate = input_learning_rate
        self.min_input_age = min_input_age
        self.weight_function = weight_function
        
        self.logger = logging.getLogger(__name__)
        
        # 初始化网络状态
        self.activity = np.zeros((self.size, self.size, self.size), dtype=np.float64)
        self.inhibition = np.ones((self.size, self.size, self.size)) * 0.1
        
        # 初始化递归权重
        self._init_recurrent_weights()
        
        # 输入权重（从视觉输入到位置细胞）
        self.input_weights = {}
        self.input_age = {}
        
        # 网络动力学参数
        self.global_inhibition = 0.001
        self.activation_threshold = 0.1
        self.normalization_factor = 1.0
        
        # 当前估计的位置
        self.current_position = np.array([self.size//2, self.size//2, self.size//2], dtype=np.float64)
        
        # 重置网络到初始状态
        self.reset()
        
    def _init_recurrent_weights(self):
        """
        初始化递归连接权重
        
        Mexican Hat函数模拟大脑中位置细胞的连接模式：
        - 相近的神经元相互激活（正反馈）
        - 远处的神经元相互抑制（负反馈）
        这种模式确保只有一个活动包能够稳定存在
        """
        self.logger.info(f"初始化递归权重，使用 {self.weight_function} 函数")
        
        # 创建权重核
        kernel_size = self.size
        center = kernel_size // 2
        
        # 创建 3D 坐标网格
        x, y, z = np.ogrid[0:kernel_size, 0:kernel_size, 0:kernel_size]
        
        # 计算到中心的距离
        dist = np.sqrt((x - center)**2 + (y - center)**2 + (z - center)**2)
        
        if self.weight_function == 'mexican_hat':
            # Mexican hat 函数 - 类似墨西哥帽子的形状
            sigma2 = self.sigma**2
            self.weights = (2 - (dist**2 / sigma2)) * np.exp(-(dist**2) / (2 * sigma2))
            # 归一化使得中心权重为 1
            center_weight = self.weights[center, center, center]
            if center_weight > 0:
                self.weights = self.weights / center_weight
            # 添加抑制环
            self.weights[dist > 3 * self.sigma] = -0.05
        else:  # gaussian
            # 高斯函数 - 纯激活性连接
            self.weights = np.exp(-(dist**2) / (2 * self.sigma**2))
            
    def path_integration_update(self, velocity: np.ndarray, yaw_rate: float, dt: float):
        """
        基于路径积分更新位置细胞活动
        
        这模拟了动物的自运动感知：通过内部的运动传感器（类似前庭系统）
        来估计位置变化，即使在没有外部参考的情况下也能维持位置估计。
        
        参数:
            velocity: 速度向量 [vx, vy, vz]（类似海豚的游泳速度）
            yaw_rate: 偏航角速率（转向速度）
            dt: 时间步长
        """
        if np.max(self.activity) == 0:
            # 如果没有活动，先注入一些
            self.inject_activity((self.size//2, self.size//2, self.size//2), strength=1.0)
            
        # 将速度转换为网格单位
        grid_velocity = velocity / self.grid_step
        
        # 计算位移（网格单位）
        displacement = grid_velocity * dt
        
        # 保存原始活动
        original_activity = self.activity.copy()
        
        # 如果位移很小，直接返回
        if np.linalg.norm(displacement) < 0.01:
            return
            
        # 使用双线性插值进行活动传播
        # 这比FFT方法更稳定，特别是对于小位移
        new_activity = np.zeros_like(self.activity)
        
        for i in range(self.size):
            for j in range(self.size):
                for k in range(self.size):
                    if original_activity[i, j, k] > 0.01:  # 只处理有显著活动的神经元
                        # 计算新位置
                        new_i = i + displacement[0]
                        new_j = j + displacement[1] 
                        new_k = k + displacement[2]
                        
                        # 处理边界条件（环绕）
                        new_i = new_i % self.size
                        new_j = new_j % self.size
                        new_k = new_k % self.size
                        
                        # 双线性插值
                        i_floor = int(np.floor(new_i)) % self.size
                        j_floor = int(np.floor(new_j)) % self.size
                        k_floor = int(np.floor(new_k)) % self.size
                        
                        i_ceil = (i_floor + 1) % self.size
                        j_ceil = (j_floor + 1) % self.size
                        k_ceil = (k_floor + 1) % self.size
                        
                        # 插值权重
                        wi = new_i - i_floor
                        wj = new_j - j_floor
                        wk = new_k - k_floor
                        
                        # 分配活动到相邻神经元
                        activity_value = original_activity[i, j, k]
                        
                        new_activity[i_floor, j_floor, k_floor] += (1-wi)*(1-wj)*(1-wk) * activity_value
                        new_activity[i_ceil, j_floor, k_floor] += wi*(1-wj)*(1-wk) * activity_value
                        new_activity[i_floor, j_ceil, k_floor] += (1-wi)*wj*(1-wk) * activity_value
                        new_activity[i_floor, j_floor, k_ceil] += (1-wi)*(1-wj)*wk * activity_value
                        new_activity[i_ceil, j_ceil, k_floor] += wi*wj*(1-wk) * activity_value
                        new_activity[i_ceil, j_floor, k_ceil] += wi*(1-wj)*wk * activity_value
                        new_activity[i_floor, j_ceil, k_ceil] += (1-wi)*wj*wk * activity_value
                        new_activity[i_ceil, j_ceil, k_ceil] += wi*wj*wk * activity_value
        
        self.activity = new_activity
        
        # 确保活动非负
        self.activity = np.maximum(self.activity, 0)
        
        # 归一化
        total_activity = np.sum(self.activity)
        if total_activity > 0:
            self.activity = self.activity / total_activity
        
        self.logger.debug(f"路径积分更新: 位移={displacement}, 最大活动={np.max(self.activity):.3f}")
        
    def visual_input_update(self, template_id: int, similarity: float):
        """
        基于视觉输入更新位置细胞活动
        
        这模拟了基于地标的位置识别：就像海豚通过声呐回波识别熟悉的地形特征
        来确认自己的位置。只有当某个视觉模板被多次观察到时，才建立稳定的连接。
        
        参数:
            template_id: 视觉模板 ID
            similarity: 与模板的相似度 [0, 1]
        """
        # 增加输入年龄
        if template_id not in self.input_age:
            self.input_age[template_id] = 0
        self.input_age[template_id] += 1
        
        # 只有当输入年龄足够时才开始学习
        if self.input_age[template_id] < self.min_input_age:
            self.logger.debug(f"模板 {template_id} 年龄不足 ({self.input_age[template_id]}/{self.min_input_age})")
            return
            
        # 初始化权重
        if template_id not in self.input_weights:
            self.input_weights[template_id] = np.random.random((self.size, self.size, self.size)) * 0.01
            self.logger.info(f"为模板 {template_id} 创建新的输入权重")
        
        # 学习：根据当前活动模式更新权重
        current_activity = self.activity.copy()
        if np.max(current_activity) > 0:
            # 归一化当前活动
            current_activity = current_activity / np.max(current_activity)
            
            # Hebbian学习：同时活跃的连接得到加强
            weight_update = self.learning_rate * similarity * current_activity
            self.input_weights[template_id] += weight_update
            
            # 限制权重范围
            self.input_weights[template_id] = np.clip(self.input_weights[template_id], 0, 1)
        
        # 应用视觉输入
        visual_input = similarity * self.input_weights[template_id]
        self.activity += visual_input
        
        self.logger.debug(f"视觉输入更新: 模板={template_id}, 相似度={similarity:.3f}")
        
    def apply_recurrent_dynamics(self):
        """
        应用递归网络动力学
        
        这是网络的核心：通过神经元间的相互作用产生稳定的活动包。
        类似于海豚群体的协调行为，每个神经元都受到邻近神经元的影响。
        """
        if np.max(self.activity) == 0:
            return
            
        # 应用递归权重
        # 使用卷积来高效计算所有神经元的递归输入
        from scipy.ndimage import convolve
        
        # 计算递归输入
        recurrent_input = convolve(self.activity, self.weights, mode='wrap')
        
        # 应用非线性激活函数
        self.activity = np.maximum(recurrent_input - self.global_inhibition, 0)
        
        # 归一化以保持稳定性
        total_activity = np.sum(self.activity)
        if total_activity > 0:
            self.activity = self.activity / total_activity
            
        self.logger.debug(f"递归动力学更新: 最大活动={np.max(self.activity):.3f}")
        
    def get_activity_center(self) -> np.ndarray:
        """
        计算活动的重心位置
        
        返回:
            center: 活动重心的坐标 [x, y, z]
        """
        if np.sum(self.activity) == 0:
            return np.array([self.size//2, self.size//2, self.size//2], dtype=np.float64)
            
        # 计算重心
        total_activity = np.sum(self.activity)
        
        x_coords, y_coords, z_coords = np.mgrid[0:self.size, 0:self.size, 0:self.size]
        
        center_x = np.sum(x_coords * self.activity) / total_activity
        center_y = np.sum(y_coords * self.activity) / total_activity  
        center_z = np.sum(z_coords * self.activity) / total_activity
        
        return np.array([center_x, center_y, center_z], dtype=np.float64)
        
    def get_peak_activity(self) -> Tuple[int, int, int]:
        """
        获取活动峰值的位置
        
        返回:
            peak_pos: 峰值位置的坐标 (x, y, z)
        """
        peak_idx = np.unravel_index(np.argmax(self.activity), self.activity.shape)
        return peak_idx
        
    def inject_activity(self, position: Tuple[int, int, int], strength: float = 1.0):
        """
        在指定位置注入活动
        
        这模拟了外部刺激或重定位事件，类似于海豚通过声呐确认位置后
        在神经网络中"点亮"对应的位置表征。
        
        参数:
            position: 注入位置 (x, y, z)
            strength: 注入强度
        """
        x, y, z = position
        
        # 创建高斯分布的活动注入
        x_coords, y_coords, z_coords = np.mgrid[0:self.size, 0:self.size, 0:self.size]
        
        dist = np.sqrt((x_coords - x)**2 + (y_coords - y)**2 + (z_coords - z)**2)
        
        injection = strength * np.exp(-(dist**2) / (2 * (self.sigma/2)**2))
        self.activity += injection
        
        # 确保非负
        self.activity = np.maximum(self.activity, 0)
        
    def reset(self):
        """重置网络状态到初始位置"""
        self.activity.fill(0)
        # 在中心位置注入初始活动
        center_pos = (self.size//2, self.size//2, self.size//2)
        self.inject_activity(center_pos, strength=1.0)
        self.current_position = np.array(center_pos, dtype=np.float64)
        
    def get_activity_slice(self, axis: int = 2, index: Optional[int] = None) -> np.ndarray:
        """
        获取活动的 2D 切片（用于可视化）
        
        参数:
            axis: 切片轴 (0=x, 1=y, 2=z)
            index: 切片索引（None 表示使用峰值位置）
            
        返回:
            slice: 2D 活动切片
        """
        if index is None:
            peak = self.get_peak_activity()
            index = peak[axis]
            
        if axis == 0:
            return self.activity[index, :, :]
        elif axis == 1:
            return self.activity[:, index, :]
        else:  # axis == 2
            return self.activity[:, :, index]
            
    def step(self, velocity: np.ndarray, yaw_rate: float, 
            visual_template_id: Optional[int] = None, visual_similarity: float = 0.0, 
            dt: float = 0.1):
        """
        执行一个完整的网络更新步骤
        
        这是主要的更新循环，模拟大脑在一个时间步内的处理过程：
        1. 路径积分（自运动感知）
        2. 视觉输入处理（地标识别）
        3. 网络动力学（神经元协调）
        
        参数:
            velocity: 速度向量 [vx, vy, vz]
            yaw_rate: 偏航角速率
            visual_template_id: 视觉模板 ID（可选）
            visual_similarity: 视觉相似度
            dt: 时间步长
        """
        # 路径积分更新
        self.path_integration_update(velocity, yaw_rate, dt)
        
        # 视觉输入更新
        if visual_template_id is not None and visual_similarity > 0:
            self.visual_input_update(visual_template_id, visual_similarity)
            
        # 应用递归动力学
        self.apply_recurrent_dynamics()
        
        # 更新当前位置估计
        self.current_position = self.get_activity_center()
        
        self.logger.debug(f"位置细胞网络更新完成，峰值活动: {np.max(self.activity):.3f}")


if __name__ == "__main__":
    # 测试代码
    logging.basicConfig(level=logging.INFO)
    
    # 创建网络
    pc_network = PlaceCellNetwork(neurons_per_dim=10, neurons_step=0.5)
    
    print(f"初始活动中心: {pc_network.get_activity_center()}")
    print(f"初始最大活动: {np.max(pc_network.activity):.3f}")
    
    # 测试路径积分
    velocity = np.array([1.0, 0.0, 0.0])
    pc_network.path_integration_update(velocity, 0.0, 0.1)
    
    print(f"路径积分后活动中心: {pc_network.get_activity_center()}")
    print(f"路径积分后最大活动: {np.max(pc_network.activity):.3f}")