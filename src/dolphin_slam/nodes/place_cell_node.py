#!/usr/bin/env python3
"""
调试增强版place_cell_node.py - 解决神经元中心不移动的问题
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
    """调试增强版位置细胞网络节点"""
    
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
                ('spatial_scale', 2.0),
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
                ('decay_rate', 0.02),
                # 输入强度参数
                ('position_input_strength', 4.0),
                ('visual_input_strength', 2.0),
                # 🔧 新增调试参数
                ('movement_threshold', 0.05),              # 🔧 降低移动检测阈值
                ('enable_position_debug', True),           # 🔧 启用位置调试
                ('position_input_override', 8.0),          # 🔧 位置输入覆盖强度
                ('center_tracking_strength', 0.7),         # 🔧 中心跟踪强度
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
        
        # 🔧 新增：位置追踪变量
        self.position_history = []
        self.movement_distances = []
        self.last_injection_time = 0
        
        # 视觉更新限制
        self.last_visual_update_time = 0
        self.last_visual_similarity = 0.0
        
        # 初始化CAN网络
        self.total_neurons = self.neurons_per_dimension ** 3
        self.activity = np.zeros((self.neurons_per_dimension, self.neurons_per_dimension, self.neurons_per_dimension))
        
        # 🔧 更强的初始活动峰
        center = self.neurons_per_dimension // 2
        self._inject_gaussian_activity((center, center, center), strength=3.0, radius=1.2)
        
        # 视觉状态跟踪
        self.visual_similarities = []
        
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
        self.update_timer = self.create_timer(0.05, self.update_network)  # 20Hz
        
        self.get_logger().info(f'🧠 调试增强版CAN网络启动')
        self.get_logger().info(f'移动检测阈值: {self.movement_threshold}m, 位置调试: {self.enable_position_debug}')
        
    def odometry_callback(self, msg: Odometry):
        """处理里程计数据 - 增强调试版"""
        self.last_odometry = msg
        
        position = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])
        
        if self.origin_position is None:
            self.origin_position = position.copy()
            self.get_logger().info(f'设置原点: ({position[0]:.2f}, {position[1]:.2f}, {position[2]:.2f})')
        
        relative_position = position - self.origin_position
        
        # 🔧 更敏感的移动检测
        movement_distance = np.linalg.norm(relative_position - self.last_position)
        
        if movement_distance > self.movement_threshold:
            self.last_position = relative_position.copy()
            self.position_updates += 1
            
            # 🔧 记录移动历史
            self.position_history.append(relative_position.copy())
            self.movement_distances.append(movement_distance)
            
            # 保留最近50个位置
            if len(self.position_history) > 50:
                self.position_history = self.position_history[-50:]
                self.movement_distances = self.movement_distances[-50:]
            
            if self.enable_path_integration:
                self.inject_position_input(relative_position)
                
            # 🔧 调试输出
            if self.enable_position_debug:
                self.get_logger().info(
                    f'🚶 位置更新#{self.position_updates}: '
                    f'移动距离={movement_distance:.3f}m, '
                    f'当前位置=({relative_position[0]:.2f}, {relative_position[1]:.2f}, {relative_position[2]:.2f})'
                )
        
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
        """注入位置输入 - 增强调试版"""
        neuron_pos = self._world_to_neuron_coords(world_position)
        
        # 🔧 调试输出坐标转换
        if self.enable_position_debug:
            self.get_logger().info(
                f'📍 坐标转换: 世界({world_position[0]:.2f}, {world_position[1]:.2f}, {world_position[2]:.2f}) '
                f'-> 神经元({neuron_pos[0]:.2f}, {neuron_pos[1]:.2f}, {neuron_pos[2]:.2f})'
            )
        
        if self._is_valid_neuron_position(neuron_pos):
            # 🔧 使用覆盖强度
            actual_strength = self.position_input_override
            self._inject_gaussian_activity(neuron_pos, 
                                         strength=actual_strength, 
                                         radius=1.5)
            
            # 🔧 额外的中心跟踪机制
            self._apply_center_tracking(neuron_pos)
            
            if self.enable_position_debug:
                self.get_logger().info(
                    f'💉 位置输入: 注入强度={actual_strength:.1f}, 位置=({neuron_pos[0]:.1f}, {neuron_pos[1]:.1f}, {neuron_pos[2]:.1f})'
                )
        else:
            if self.enable_position_debug:
                self.get_logger().warn(
                    f'⚠️ 无效神经元位置: ({neuron_pos[0]:.2f}, {neuron_pos[1]:.2f}, {neuron_pos[2]:.2f})'
                )
    
    def _apply_center_tracking(self, target_neuron_pos):
        """应用中心跟踪机制 - 强制移动活动中心"""
        current_center = self._get_activity_center()
        
        # 计算从当前中心到目标位置的方向
        direction = np.array(target_neuron_pos) - current_center
        
        # 沿着方向移动活动峰
        for i in range(3):  # 分3步移动
            intermediate_pos = current_center + direction * (i + 1) / 3.0
            
            # 确保在有效范围内
            intermediate_pos = np.clip(intermediate_pos, 0, self.neurons_per_dimension - 1)
            
            # 在中间位置注入活动
            self._inject_gaussian_activity(
                intermediate_pos, 
                strength=self.center_tracking_strength * (i + 1), 
                radius=1.0
            )
    
    def inject_visual_input(self, visual_strength):
        """注入视觉输入"""
        if visual_strength > self.visual_threshold:
            max_pos = np.unravel_index(np.argmax(self.activity), self.activity.shape)
            self._inject_gaussian_activity(max_pos, 
                                         strength=visual_strength * self.visual_input_strength, 
                                         radius=0.8)
    
    def update_network(self):
        """更新网络"""
        try:
            self.update_count += 1
            
            # 应用CAN动力学
            self._apply_balanced_can_dynamics()
            
            # 计算统计
            stats = self._compute_activation_stats()
            
            # 发布活动
            msg = Float32MultiArray()
            msg.data = self.activity.flatten().tolist()
            self.activity_pub.publish(msg)
            
            # 定期报告
            if self.update_count % self.major_interval == 0:
                self._report_network_status(stats)
                
        except Exception as e:
            self.get_logger().error(f'网络更新错误: {e}')
    
    def _apply_balanced_can_dynamics(self):
        """应用平衡的CAN动力学"""
        
        # 步骤1: 很轻微的衰减
        self.activity *= (1.0 - self.decay_rate)
        
        # 步骤2: 局部兴奋
        excitatory_input = gaussian_filter(
            self.activity, 
            sigma=self.excitation_radius, 
            mode='constant'
        )
        
        # 步骤3: 轻微侧向抑制
        lateral_inhibition = gaussian_filter(
            self.activity, 
            sigma=self.lateral_inhibition_radius, 
            mode='constant'
        )
        
        # 步骤4: 适度全局抑制
        global_activity = np.sum(self.activity)
        global_inhibition = (
            self.inhibition_strength * 
            self.global_inhibition_factor * 
            global_activity / self.total_neurons
        )
        
        # 步骤5: 轻微胜者通吃
        max_activity = np.max(self.activity)
        if max_activity > 0:
            winner_mask = self.activity < (max_activity * self.winner_take_all_strength)
        else:
            winner_mask = np.zeros_like(self.activity, dtype=bool)
        
        # 步骤6: 更新方程
        new_activity = (
            self.activity +                              # 保持当前活动
            excitatory_input * 1.2 +                     # 🔧 增强局部兴奋
            -lateral_inhibition * 0.1 +                  # 🔧 减少侧向抑制
            -global_inhibition * 0.5 +                   # 🔧 减少全局抑制
            np.random.normal(0, 0.01, self.activity.shape)  # 适度噪声
        )
        
        # 步骤7: 轻微胜者通吃
        new_activity[winner_mask] *= 0.8  # 🔧 减少抑制强度
        
        # 步骤8: 非线性激活
        new_activity = np.maximum(0, new_activity)
        
        # 步骤9: 归一化
        if np.max(new_activity) > 0:
            new_activity = new_activity / np.max(new_activity) * self.normalization_factor
        
        # 步骤10: 保留更多活动
        new_activity[new_activity < 0.005] = 0  # 🔧 降低清零阈值
        
        self.activity = new_activity
    
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
        
        # 🔧 增强的状态指示器
        if stats['activation_rate'] < 0.05:
            activation_status = "⚠️"
        elif stats['activation_rate'] < 0.10:
            activation_status = "🟡"
        elif stats['activation_rate'] < 0.25:
            activation_status = "✅"
        else:
            activation_status = "❌"
        
        # 🔧 计算移动统计
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
        
        # 🔧 额外的调试信息
        if self.enable_position_debug and len(self.position_history) > 0:
            latest_pos = self.position_history[-1]
            self.get_logger().info(
                f'📍 位置调试: 最新位置=({latest_pos[0]:.2f}, {latest_pos[1]:.2f}, {latest_pos[2]:.2f}), '
                f'平均移动距离={avg_distance:.3f}m'
            )
    
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
        """计算活动中心"""
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
            return np.array([center_x, center_y, center_z])
        else:
            return np.array([self.neurons_per_dimension/2] * 3)
    
    def _world_to_neuron_coords(self, world_pos):
        """世界坐标到神经元坐标转换"""
        # 🔧 修正坐标转换
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