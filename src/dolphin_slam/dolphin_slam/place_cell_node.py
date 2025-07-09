#!/usr/bin/env python3
"""
简单修复版位置细胞网络节点
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
    """简单修复版位置细胞网络节点"""
    
    def __init__(self):
        super().__init__('place_cell_node')
        
        # 🔧 修复的参数 - 硬编码，避免任何错误
        self.neurons_per_dimension = 16
        self.spatial_scale = 0.3125  # 正确的空间尺度
        self.workspace_center = np.array([2.5, 0.0, -14.5])  # 正确的工作空间中心
        self.update_rate = 20.0
        self.major_interval = 100
        
        # 路径积分参数
        self.movement_threshold = 0.01
        self.path_integration_strength = 2.0
        self.activity_injection_radius = 1.2
        
        # 初始化网络
        self.total_neurons = self.neurons_per_dimension ** 3
        self.activity = np.random.random(
            (self.neurons_per_dimension, self.neurons_per_dimension, self.neurons_per_dimension)
        ) * 0.1
        
        # 在中心注入初始活动
        center_pos = 8  # 16 // 2 = 8
        self._inject_gaussian_activity([center_pos, center_pos, center_pos], strength=2.0, radius=2.0)
        
        # 状态跟踪
        self.last_position = None
        self.last_odometry_time = None
        self.update_count = 0
        self.position_updates = 0
        self.total_distance = 0.0
        self.visual_updates = 0
        
        # 订阅者
        self.odometry_sub = self.create_subscription(
            Odometry,
            '/dolphin_slam/odometry',
            self.odometry_callback,
            10
        )
        
        self.visual_match_sub = self.create_subscription(
            Float32MultiArray,
            '/local_view/matches',
            self.visual_match_callback,
            10
        )
        
        # 发布者
        self.activity_pub = self.create_publisher(
            Float32MultiArray,
            '/place_cells/activity',
            10
        )
        
        # 定时器
        self.update_timer = self.create_timer(
            1.0 / self.update_rate,
            self.update_network
        )
        
        # 启动消息
        self.get_logger().info('🎯 初始化网络，中心位置: (8, 8, 8)')
        self.get_logger().info('🧠 简单修复版位置细胞网络已启动: 16³神经元, 空间尺度=0.3125m')
        self.get_logger().info('🌍 工作空间中心: [2.5, 0.0, -14.5]')
        
    def odometry_callback(self, msg: Odometry):
        """处理里程计数据"""
        current_time = self.get_clock().now()
        
        # 提取位置信息
        current_pos = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])
        
        # 路径积分更新
        if self.last_position is not None:
            displacement = current_pos - self.last_position
            distance = np.linalg.norm(displacement)
            
            if distance > self.movement_threshold:
                self._apply_strong_path_integration(displacement)
                self.total_distance += distance
                self.position_updates += 1
                
                # 每次移动都记录
                if self.position_updates % 10 == 0:
                    center = self._get_activity_center()
                    world_center = self._neuron_to_world_coords(center)
                    self.get_logger().info(
                        f'🚶 路径积分更新#{self.position_updates}: '
                        f'位移={displacement}, 距离={distance:.3f}m, '
                        f'神经元中心={center}, 世界中心={world_center}'
                    )
        
        self.last_position = current_pos
        self.last_odometry_time = current_time
        
    def visual_match_callback(self, msg: Float32MultiArray):
        """处理视觉匹配数据"""
        if len(msg.data) > 0:
            visual_strength = max(msg.data) if msg.data else 0.0
            if visual_strength > 0.3:
                peak_pos = self._get_activity_peak()
                self._inject_gaussian_activity(
                    peak_pos, 
                    strength=visual_strength * 0.8,
                    radius=1.2
                )
                self.visual_updates += 1
    
    def _neuron_to_world_coords(self, neuron_pos):
        """🔧 修复版：神经元坐标到世界坐标转换"""
        center_offset = 8.0  # 16 / 2 = 8
        # 标准转换：(神经元坐标 - 中心偏移) × 空间尺度 + 工作空间中心
        relative_pos = (np.array(neuron_pos) - center_offset) * self.spatial_scale
        world_pos = relative_pos + self.workspace_center
        return world_pos
    
    def _world_to_neuron_coords(self, world_pos):
        """🔧 修复版：世界坐标到神经元坐标转换"""
        center_offset = 8.0  # 16 / 2 = 8
        # 逆向转换：(世界坐标 - 工作空间中心) ÷ 空间尺度 + 中心偏移
        relative_pos = np.array(world_pos) - self.workspace_center
        neuron_coords = relative_pos / self.spatial_scale + center_offset
        return neuron_coords
        
    def _apply_strong_path_integration(self, displacement):
        """强化版路径积分"""
        # 转换到神经元空间
        neuron_displacement = displacement / self.spatial_scale
        
        # 获取当前活动峰
        current_peak = self._get_activity_peak()
        
        # 计算新位置
        new_peak = np.array(current_peak) + neuron_displacement
        
        # 处理边界条件
        new_peak = np.clip(new_peak, 0, self.neurons_per_dimension - 1)
        
        # 强力注入活动
        self._inject_gaussian_activity(
            new_peak, 
            strength=self.path_integration_strength,
            radius=self.activity_injection_radius
        )
        
        # 在移动路径上注入活动
        steps = 3
        for i in range(1, steps):
            intermediate_pos = current_peak + neuron_displacement * (i / steps)
            intermediate_pos = np.clip(intermediate_pos, 0, self.neurons_per_dimension - 1)
            self._inject_gaussian_activity(
                intermediate_pos,
                strength=self.path_integration_strength * 0.5,
                radius=0.8
            )
    
    def _inject_gaussian_activity(self, center_pos, strength=1.0, radius=1.0):
        """注入高斯活动"""
        try:
            center_pos = np.array(center_pos, dtype=float)
            
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
            self.activity = np.clip(self.activity, 0, 10.0)
            
        except Exception as e:
            self.get_logger().error(f"活动注入失败: {e}")
    
    def _get_activity_center(self):
        """计算活动中心"""
        if np.max(self.activity) == 0:
            return np.array([8.0, 8.0, 8.0])  # 默认中心
        
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
            return np.array([8.0, 8.0, 8.0])
    
    def _get_activity_peak(self):
        """获取活动峰位置"""
        max_idx = np.unravel_index(np.argmax(self.activity), self.activity.shape)
        return np.array(max_idx, dtype=float)
    
    def _compute_network_stats(self):
        """计算网络统计"""
        peak = np.max(self.activity)
        mean = np.mean(self.activity)
        std = np.std(self.activity)
        active_neurons = np.sum(self.activity > 0.1)
        center = self._get_activity_center()
        world_center = self._neuron_to_world_coords(center)
        
        return {
            'peak': peak,
            'mean': mean,
            'std': std,
            'active_neurons': active_neurons,
            'activation_rate': active_neurons / self.total_neurons,
            'center': center,
            'world_center': world_center
        }
    
    def update_network(self):
        """网络主更新循环"""
        try:
            self.update_count += 1
            
            # 轻微的全局衰减
            self.activity *= 0.99
            
            # 归一化
            if np.max(self.activity) > 0:
                self.activity = self.activity / np.max(self.activity) * 8.0
            
            # 发布活动
            msg = Float32MultiArray()
            msg.data = self.activity.flatten().tolist()
            self.activity_pub.publish(msg)
            
            # 定期报告
            if self.update_count % self.major_interval == 0:
                self.report_status()
                
        except Exception as e:
            self.get_logger().error(f"网络更新失败: {e}")
    
    def report_status(self):
        """定期报告网络状态"""
        stats = self._compute_network_stats()
        
        status_icon = "✅" if stats['activation_rate'] > 0.05 else "⚠️"
        
        self.get_logger().info(
            f"🧠 网络更新#{self.update_count}: "
            f"峰值={stats['peak']:.4f}, "
            f"均值={stats['mean']:.4f}±{stats['std']:.4f}, "
            f"活跃={stats['active_neurons']}/{self.total_neurons}"
            f"({stats['activation_rate']:.1%}){status_icon}, "
            f"神经元中心=({stats['center'][0]:.1f},{stats['center'][1]:.1f},{stats['center'][2]:.1f}), "
            f"对应世界=({stats['world_center'][0]:.1f},{stats['world_center'][1]:.1f},{stats['world_center'][2]:.1f}), "
            f"位置更新={self.position_updates}次, " 
            f"总移动距离={self.total_distance:.2f}m, "
            f"视觉更新={self.visual_updates}次"
        )
        
        if self.position_updates > 0 and self.last_position is not None:
            avg_distance = self.total_distance / self.position_updates
            self.get_logger().info(
                f"📍 位置调试: 最新位置=({self.last_position[0]:.2f}, "
                f"{self.last_position[1]:.2f}, {self.last_position[2]:.2f}), "
                f"平均移动距离={avg_distance:.3f}m"
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
