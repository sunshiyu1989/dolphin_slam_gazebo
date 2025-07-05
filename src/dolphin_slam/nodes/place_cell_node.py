#!/usr/bin/env python3
"""
Dolphin SLAM - 位置细胞网络节点 (最终修复版)
解决了所有已知的跟踪和重置问题
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
import numpy as np
from scipy.ndimage import gaussian_filter
from typing import Optional
import time

class PlaceCellNode(Node):
    """最终修复版位置细胞网络节点"""
    
    def __init__(self):
        super().__init__('place_cell_node')
        
        # 声明参数
        self.declare_parameters(
            namespace='',
            parameters=[
                ('odometry_topic', '/dolphin_slam/odometry'),
                ('visual_match_topic', '/local_view/matches'),
                ('activity_topic', '/place_cells/activity'),
                ('neurons_per_dimension', 16),
                ('update_rate', 20.0),
                ('major_report_interval', 300),
                ('spatial_scale', 30.0),
                ('visual_similarity_threshold', 0.4),
                ('enable_path_integration', True),
                ('enable_visual_debug', False),
                # CAN参数
                ('excitation_radius', 1.3),
                ('inhibition_strength', 0.3),
                ('activity_threshold', 0.1),
                ('normalization_factor', 8.0),
                ('visual_update_cooldown', 0.5),
                ('min_visual_change_threshold', 0.03),
                # 抑制参数
                ('global_inhibition_factor', 0.5),
                ('winner_take_all_strength', 0.3),
                ('lateral_inhibition_radius', 2.0),
                ('decay_rate', 0.12),  # 🔧 增加衰减率，减少惯性
                # 输入强度参数
                ('position_input_strength', 4.0),
                ('visual_input_strength', 2.0),
                # 🔧 增强的调试参数
                ('movement_threshold', 0.05),
                ('enable_position_debug', True),
                ('position_input_override', 40.0),  # 🔧 增强输入强度
                ('center_tracking_strength', 0.7),
            ]
        )
        
        # 获取参数
        self.neurons_per_dimension = self.get_parameter('neurons_per_dimension').value
        self.spatial_scale = self.get_parameter('spatial_scale').value
        self.visual_threshold = self.get_parameter('visual_similarity_threshold').value
        self.enable_path_integration = self.get_parameter('enable_path_integration').value
        self.major_interval = self.get_parameter('major_report_interval').value
        
        # CAN参数
        self.excitation_radius = self.get_parameter('excitation_radius').value
        self.inhibition_strength = self.get_parameter('inhibition_strength').value
        self.activity_threshold = self.get_parameter('activity_threshold').value
        self.normalization_factor = self.get_parameter('normalization_factor').value
        self.visual_cooldown = self.get_parameter('visual_update_cooldown').value
        self.min_visual_change = self.get_parameter('min_visual_change_threshold').value
        
        # 抑制参数
        self.global_inhibition_factor = self.get_parameter('global_inhibition_factor').value
        self.winner_take_all_strength = self.get_parameter('winner_take_all_strength').value
        self.lateral_inhibition_radius = self.get_parameter('lateral_inhibition_radius').value
        self.decay_rate = self.get_parameter('decay_rate').value
        
        # 输入强度参数
        self.position_input_strength = self.get_parameter('position_input_strength').value
        self.visual_input_strength = self.get_parameter('visual_input_strength').value
        
        # 🔧 调试参数
        self.movement_threshold = self.get_parameter('movement_threshold').value
        self.enable_position_debug = self.get_parameter('enable_position_debug').value
        self.position_input_override = self.get_parameter('position_input_override').value
        self.center_tracking_strength = self.get_parameter('center_tracking_strength').value
        
        # 状态变量
        self.last_odometry: Optional[Odometry] = None
        self.last_position = np.array([0.0, 0.0, 0.0])
        self.origin_position = None
        self.update_count = 0
        self.position_updates = 0
        self.visual_updates = 0
        
        # 🔧 增强的位置追踪变量
        self.last_true_position = None  # 跟踪最后的真实位置
        self.center_history = []        # 活动中心历史
        self.max_center_history = 5     # 历史记录长度
        self.reset_count = 0            # 重置计数器
        self.position_history = []
        self.movement_distances = []
        self.last_injection_time = 0
        
        # 视觉更新限制
        self.last_visual_update_time = 0
        self.last_visual_similarity = 0.0
        self.visual_similarities = []
        
        # 初始化CAN网络
        self.total_neurons = self.neurons_per_dimension ** 3
        self.activity = np.zeros((self.neurons_per_dimension, self.neurons_per_dimension, self.neurons_per_dimension))
        
        # 🔧 更强的初始活动峰
        center = self.neurons_per_dimension // 2
        self._inject_gaussian_activity((center, center, center), strength=3.0, radius=1.2)
        
        # 订阅者
        self.odometry_sub = self.create_subscription(
            Odometry,
            self.get_parameter('odometry_topic').value,
            self.odometry_callback,
            10
        )
        
        self.visual_match_sub = self.create_subscription(
            Float32MultiArray,
            self.get_parameter('visual_match_topic').value,
            self.visual_match_callback,
            10
        )
        
        # 发布者
        self.activity_pub = self.create_publisher(
            Float32MultiArray,
            self.get_parameter('activity_topic').value,
            10
        )
        
        # 定时器
        self.update_timer = self.create_timer(
            1.0 / self.get_parameter('update_rate').value,
            self.update_network
        )
        
        self.get_logger().info(
            f'🧠 位置细胞网络已启动: {self.neurons_per_dimension}³神经元, '
            f'空间尺度={self.spatial_scale}m'
        )
    
    def odometry_callback(self, msg: Odometry):
        """处理里程计数据 - 增强版"""
        try:
            self.last_odometry = msg
            
            # 提取位置
            position = np.array([
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z
            ])
            
            # 🔧 保存真实位置
            self.last_true_position = position.copy()
            
            # 🔧 坐标转换验证
            if self.enable_position_debug and self.position_updates % 100 == 0:
                self.verify_coordinate_conversion(position, " (里程计)")
            
            # 设置原点
            if self.origin_position is None:
                self.origin_position = position.copy()
                self.get_logger().info(f'🎯 设置原点: ({position[0]:.2f}, {position[1]:.2f}, {position[2]:.2f})')
                return
            
            # 计算相对位置
            relative_position = position - self.origin_position
            
            # 计算移动距离
            movement_distance = np.linalg.norm(relative_position - self.last_position)
            
            # 检查是否需要更新
            if movement_distance >= self.movement_threshold:
                self.last_position = relative_position.copy()
                self.position_updates += 1
                
                # 记录移动
                self.position_history.append(relative_position.copy())
                self.movement_distances.append(movement_distance)
                if len(self.position_history) > 1000:
                    self.position_history.pop(0)
                    self.movement_distances.pop(0)
                
                # 注入位置输入
                self.inject_position_input(relative_position)
                
                if self.enable_position_debug:
                    self.get_logger().info(
                        f'🚶 位置更新#{self.position_updates}: '
                        f'移动距离={movement_distance:.3f}m, '
                        f'当前位置=({relative_position[0]:.2f}, {relative_position[1]:.2f}, {relative_position[2]:.2f})'
                    )
            
            # 🔧 添加位置跟踪监控
            self.monitor_position_tracking()
            
        except Exception as e:
            self.get_logger().error(f'里程计处理错误: {e}')
    
    def visual_match_callback(self, msg: Float32MultiArray):
        """处理视觉匹配数据"""
        if len(msg.data) == 0:
            return
            
        similarity = msg.data[0]
        current_time = time.time()
        
        # 视觉更新限制
        time_since_last_update = current_time - self.last_visual_update_time
        similarity_change = abs(similarity - self.last_visual_similarity)
        
        should_update = (
            time_since_last_update > self.visual_cooldown and
            similarity_change > self.min_visual_change and
            similarity > self.visual_threshold
        )
        
        if should_update:
            self.visual_updates += 1
            self.last_visual_update_time = current_time
            self.last_visual_similarity = similarity
            self.inject_visual_input(similarity)
        
        # 记录视觉数据
        self.visual_similarities.append(similarity)
        if len(self.visual_similarities) > 100:
            self.visual_similarities = self.visual_similarities[-100:]
    
    def inject_position_input(self, world_position):
        """强化版位置输入注入 - 解决跟踪滞后问题"""
        neuron_pos = self._world_to_neuron_coords(world_position)
        
        if self.enable_position_debug:
            self.get_logger().info(
                f'📍 坐标转换: 世界({world_position[0]:.2f}, {world_position[1]:.2f}, {world_position[2]:.2f}) '
                f'-> 神经元({neuron_pos[0]:.2f}, {neuron_pos[1]:.2f}, {neuron_pos[2]:.2f})'
            )
        
        if self._is_valid_neuron_position(neuron_pos):
            current_peak = np.max(self.activity)
            current_activation_rate = np.sum(self.activity > self.activity_threshold) / self.total_neurons
            
            # 🔧 大幅增强位置输入强度
            base_strength = 25.0  # 固定强度，确保能主导网络活动
            
            if current_peak > 10.0 or current_activation_rate > 0.4:
                input_strength = base_strength * 0.8  # 20.0
            elif current_peak < 2.0 or current_activation_rate < 0.1:
                input_strength = base_strength * 1.5  # 37.5
            else:
                input_strength = base_strength  # 25.0
            
            if self.enable_position_debug:
                self.get_logger().info(
                    f'💉 位置输入: 强度={input_strength}, 当前峰值={current_peak:.1f}, 激活率={current_activation_rate:.1%}'
                )
            
            # 🔧 强化位置注入
            self._inject_strong_position_activity(neuron_pos, input_strength)
    
    def _inject_strong_position_activity(self, center_pos, strength):
        """强化版位置活动注入 - 主导性输入"""
        # 🔧 步骤1: 在目标位置注入强活动
        self._inject_gaussian_activity(center_pos, strength=strength/3.0, radius=1.2)
        
        # 🔧 步骤2: 抑制远离目标的活动
        x, y, z = np.meshgrid(
            np.arange(self.neurons_per_dimension),
            np.arange(self.neurons_per_dimension),
            np.arange(self.neurons_per_dimension),
            indexing='ij'
        )
        
        # 计算距离目标位置的距离
        dist_from_target = np.sqrt(
            (x - center_pos[0])**2 + 
            (y - center_pos[1])**2 + 
            (z - center_pos[2])**2
        )
        
        # 对距离目标较远的区域施加抑制
        inhibition_mask = dist_from_target > 2.0  # 距离超过2个神经元单位
        inhibition_strength = 0.3
        self.activity[inhibition_mask] *= (1.0 - inhibition_strength)
        
        # 确保非负
        self.activity = np.maximum(self.activity, 0)
    
    def inject_visual_input(self, similarity):
        """注入视觉输入"""
        if similarity > self.visual_threshold:
            visual_strength = similarity * self.visual_input_strength
            
            # 在当前活动中心附近增强活动
            activity_center = self._get_activity_center()
            self._inject_gaussian_activity(activity_center, strength=visual_strength, radius=1.5)
    
    def update_network(self):
        """网络更新主循环"""
        try:
            self.update_count += 1
            
            # 应用CAN动力学
            self._apply_enhanced_can_dynamics()
            
            # 发布活动数据
            self._publish_activity()
            
            # 定期报告
            if self.update_count % self.major_interval == 0:
                stats = self._compute_activation_stats()
                self._report_network_status(stats)
                
        except Exception as e:
            self.get_logger().error(f'网络更新错误: {e}')
    
    def _apply_enhanced_can_dynamics(self):
        """增强版CAN动力学 - 减少惯性，提高响应性"""
        
        # 🔧 步骤1: 增强衰减，减少网络"记忆"
        enhanced_decay = self.decay_rate * 1.2  # 进一步增加衰减
        self.activity *= (1.0 - enhanced_decay)
        
        # 🔧 步骤2: 应用兴奋性滤波
        excitatory_input = gaussian_filter(
            self.activity, 
            sigma=self.excitation_radius, 
            mode='constant'
        )
        
        # 🔧 步骤3: 应用侧向抑制
        lateral_inhibition = gaussian_filter(
            self.activity, 
            sigma=self.lateral_inhibition_radius, 
            mode='constant'
        )
        
        # 🔧 步骤4: 全局抑制
        global_activity = np.sum(self.activity)
        global_inhibition = (
            self.inhibition_strength * 
            self.global_inhibition_factor * 
            global_activity / self.total_neurons
        )
        
        # 🔧 步骤5: 网络更新方程
        new_activity = (
            self.activity * 0.7 +                     # 保留70%原始活动
            excitatory_input * 0.8 +                  # 兴奋性输入
            -lateral_inhibition * 0.3 +               # 侧向抑制
            -global_inhibition                        # 全局抑制
        )
        
        # 🔧 步骤6: 非线性激活
        new_activity = np.maximum(0, new_activity)
        
        # 🔧 步骤7: 防止过度激活
        max_activity = np.max(new_activity)
        if max_activity > 15.0:
            new_activity = new_activity / max_activity * 12.0
        
        # 🔧 步骤8: 更新活动
        self.activity = new_activity
        
        # 🔧 步骤9: 位置校正机制
        self._apply_position_correction()
        
        # 🔧 步骤10: 最终安全检查（极少触发的重置）
        final_peak = np.max(self.activity)
        final_activation_rate = np.sum(self.activity > self.activity_threshold) / self.total_neurons
        
        if final_peak > 50.0 or final_activation_rate > 0.95:  # 极高阈值
            self._emergency_reset()
    
    def _apply_position_correction(self):
        """位置校正机制 - 防止过度偏离"""
        if self.last_true_position is None:
            return
            
        true_neuron_pos = self._world_to_neuron_coords(self.last_true_position)
        if not self._is_valid_neuron_position(true_neuron_pos):
            return
        
        # 计算当前活动中心与真实位置的偏差
        current_center = self._get_activity_center()
        deviation = np.linalg.norm(current_center - true_neuron_pos)
        
        # 如果偏差过大，施加"拉回力"
        if deviation > 1.0:  # 超过1个神经元单位的偏差
            pull_strength = min(deviation * 3.0, 10.0)  # 拉回强度
            self._inject_gaussian_activity(
                true_neuron_pos, 
                strength=pull_strength, 
                radius=0.8
            )
            
            if self.enable_position_debug and deviation > 2.0:
                decoded_world = self._neuron_to_world_coords(current_center)
                self.get_logger().info(
                    f"🎯 位置校正: 偏差={deviation:.2f}, 拉回强度={pull_strength:.1f}, "
                    f"解码位置=({decoded_world[0]:.1f},{decoded_world[1]:.1f},{decoded_world[2]:.1f}), "
                    f"真实位置=({self.last_true_position[0]:.1f},{self.last_true_position[1]:.1f},{self.last_true_position[2]:.1f})"
                )
    
    def _emergency_reset(self):
        """紧急重置 - 使用真实位置"""
        self.reset_count += 1
        
        if self.enable_position_debug:
            final_peak = np.max(self.activity)
            final_activation_rate = np.sum(self.activity > self.activity_threshold) / self.total_neurons
            self.get_logger().warn(
                f"🚨 网络紧急重置#{self.reset_count}: 峰值={final_peak:.1f}, "
                f"激活率={final_activation_rate:.1%}"
            )
        
        if self.last_true_position is not None:
            # 🔧 直接计算，避免函数调用可能的参数问题
            true_neuron_pos = self.last_true_position / self.spatial_scale + (self.neurons_per_dimension / 2)
            
            if self.enable_position_debug:
                self.get_logger().info(
                    f"🔍 重置调试: 真实位置{self.last_true_position} "
                    f"→ 神经元{true_neuron_pos} (scale={self.spatial_scale})"
                )
            
            if self._is_valid_neuron_position(true_neuron_pos):
                self.activity.fill(0)
                self._inject_gaussian_activity(true_neuron_pos, strength=4.0, radius=1.2)
                
                # 验证重置后的解码
                verification_world = (true_neuron_pos - self.neurons_per_dimension/2) * self.spatial_scale
                if self.enable_position_debug:
                    self.get_logger().info(
                        f"✅ 重置验证: 神经元{true_neuron_pos} → 世界{verification_world}"
                    )
            else:
                # 使用安全的中心位置
                center = np.array([self.neurons_per_dimension/2] * 3)
                self.activity.fill(0)
                self._inject_gaussian_activity(center, strength=4.0, radius=1.2)
        else:
            # 使用默认中心
            center = np.array([self.neurons_per_dimension/2] * 3)
            self.activity.fill(0)
            self._inject_gaussian_activity(center, strength=4.0, radius=1.2)
    
    def monitor_position_tracking(self):
        """监控位置跟踪精度"""
        if (self.last_true_position is not None and 
            self.enable_position_debug and 
            self.position_updates % 50 == 0):  # 每50次更新监控一次
            
            # 计算跟踪误差
            activity_center = self._get_activity_center()
            decoded_world = self._neuron_to_world_coords(activity_center)
            
            error = np.linalg.norm(decoded_world - self.last_true_position)
            error_x = abs(decoded_world[0] - self.last_true_position[0])
            error_y = abs(decoded_world[1] - self.last_true_position[1])
            error_z = abs(decoded_world[2] - self.last_true_position[2])
            
            self.get_logger().info(
                f"📊 跟踪精度监控: 总误差={error:.2f}m, "
                f"X误差={error_x:.2f}m, Y误差={error_y:.2f}m, Z误差={error_z:.2f}m"
            )
            
            # 如果Y方向误差过大，发出警告
            if error_y > 3.0:
                self.get_logger().warn(
                    f"⚠️ Y方向跟踪误差过大: {error_y:.2f}m，建议调整参数"
                )
    
    def verify_coordinate_conversion(self, world_pos, description=""):
        """验证坐标转换的正确性"""
        if not self.enable_position_debug:
            return
            
        neuron_pos = self._world_to_neuron_coords(world_pos)
        recovered_world = self._neuron_to_world_coords(neuron_pos)
        error = np.linalg.norm(world_pos - recovered_world)
        
        self.get_logger().info(
            f"🔍 坐标验证{description}: "
            f"世界{world_pos} → 神经元{neuron_pos} → 世界{recovered_world}, "
            f"误差={error:.6f}"
        )
        
        if error > 0.001:
            self.get_logger().error(f"❌ 坐标转换误差过大: {error:.6f}")
    
    def _publish_activity(self):
        """发布活动数据"""
        try:
            activity_msg = Float32MultiArray()
            activity_msg.data = self.activity.flatten().tolist()
            self.activity_pub.publish(activity_msg)
        except Exception as e:
            self.get_logger().error(f'发布活动数据错误: {e}')
    
    def _inject_gaussian_activity(self, center_pos, strength=1.0, radius=1.0):
        """注入高斯活动"""
        x, y, z = np.meshgrid(
            np.arange(self.neurons_per_dimension),
            np.arange(self.neurons_per_dimension),
            np.arange(self.neurons_per_dimension),
            indexing='ij'
        )
        
        dist_sq = ((x - center_pos[0])**2 + 
                   (y - center_pos[1])**2 + 
                   (z - center_pos[2])**2)
        
        gaussian = strength * np.exp(-dist_sq / (2 * radius**2))
        self.activity += gaussian
    
    def _get_activity_center(self):
        """计算活动中心 - 带历史平滑"""
        if np.max(self.activity) == 0:
            return np.array([self.neurons_per_dimension/2] * 3)
        
        x, y, z = np.meshgrid(
            np.arange(self.neurons_per_dimension),
            np.arange(self.neurons_per_dimension), 
            np.arange(self.neurons_per_dimension),
            indexing='ij'
        )
        
        total_activity = np.sum(self.activity)
        if total_activity > 0:
            center_x = np.sum(x * self.activity) / total_activity
            center_y = np.sum(y * self.activity) / total_activity
            center_z = np.sum(z * self.activity) / total_activity
            current_center = np.array([center_x, center_y, center_z])
            
            # 🔧 添加历史平滑
            self.center_history.append(current_center)
            if len(self.center_history) > self.max_center_history:
                self.center_history.pop(0)
            
            # 如果有足够历史，使用平滑后的中心
            if len(self.center_history) >= 3:
                smoothed_center = np.mean(self.center_history[-3:], axis=0)
                return smoothed_center
            
            return current_center
        else:
            return np.array([self.neurons_per_dimension/2] * 3)
    
    def _world_to_neuron_coords(self, world_pos):
        """世界坐标到神经元坐标转换"""
        neuron_pos = world_pos / self.spatial_scale
        center_offset = self.neurons_per_dimension / 2
        neuron_coords = neuron_pos + center_offset
        return neuron_coords
    
    def _neuron_to_world_coords(self, neuron_pos):
        """神经元坐标到世界坐标转换"""
        center_offset = self.neurons_per_dimension / 2
        world_pos = (neuron_pos - center_offset) * self.spatial_scale
        return world_pos
    
    def _is_valid_neuron_position(self, neuron_pos):
        """检查位置有效性"""
        return all(0 <= pos < self.neurons_per_dimension for pos in neuron_pos)
    
    def _compute_activation_stats(self):
        """计算激活统计"""
        active_neurons = np.sum(self.activity > self.activity_threshold)
        activation_rate = active_neurons / self.total_neurons
        
        activity_center = self._get_activity_center()
        world_center = self._neuron_to_world_coords(activity_center)
        
        return {
            'active_neurons': active_neurons,
            'activation_rate': activation_rate,
            'peak_activity': np.max(self.activity),
            'mean_activity': np.mean(self.activity),
            'std_activity': np.std(self.activity),
            'activity_center_neuron': activity_center,
            'activity_center_world': world_center
        }
    
    def _report_network_status(self, stats):
        """报告网络状态 - 增强调试版"""
        if self.visual_similarities:
            avg_visual = np.mean(self.visual_similarities[-20:])
            max_visual = np.max(self.visual_similarities[-20:])
        else:
            avg_visual = max_visual = 0.0
        
        # 状态指示器
        if stats['activation_rate'] < 0.05:
            activation_status = "⚠️"
        elif stats['activation_rate'] < 0.10:
            activation_status = "🟡"
        elif stats['activation_rate'] < 0.25:
            activation_status = "✅"
        else:
            activation_status = "❌"
        
        # 计算移动统计
        if len(self.position_history) > 1:
            total_distance = sum(self.movement_distances)
            avg_distance = np.mean(self.movement_distances)
        else:
            total_distance = avg_distance = 0.0
        
        self.get_logger().info(
            f'🧠 网络更新#{self.update_count}: '
            f'峰值={stats["peak_activity"]:.4f}, '
            f'均值={stats["mean_activity"]:.4f}±{stats["std_activity"]:.4f}, '
            f'活跃={stats["active_neurons"]}/{self.total_neurons}({stats["activation_rate"]:.1%}){activation_status}, '
            f'神经元中心=({stats["activity_center_neuron"][0]:.1f},{stats["activity_center_neuron"][1]:.1f},{stats["activity_center_neuron"][2]:.1f}), '
            f'对应世界=({stats["activity_center_world"][0]:.1f},{stats["activity_center_world"][1]:.1f},{stats["activity_center_world"][2]:.1f}), '
            f'位置更新={self.position_updates}次, '
            f'总移动距离={total_distance:.2f}m, '
            f'视觉更新={self.visual_updates}次'
        )
        
        # 额外的调试信息
        if self.enable_position_debug and len(self.position_history) > 0:
            latest_pos = self.position_history[-1]
            self.get_logger().info(
                f'📍 位置调试: 最新位置=({latest_pos[0]:.2f}, {latest_pos[1]:.2f}, {latest_pos[2]:.2f}), '
                f'平均移动距离={avg_distance:.3f}m'
            )

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = PlaceCellNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()