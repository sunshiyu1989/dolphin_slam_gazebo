#!/usr/bin/env python3
"""
修复版位置细胞网络节点 - 解决坐标转换和位置偏差问题
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
    """修复版位置细胞网络节点"""
    
    def __init__(self):
        super().__init__('place_cell_node')
        
        # 🔧 修复版参数配置
        self.neurons_per_dimension = 16
        self.spatial_scale = 2.0
        self.workspace_center = np.array([0.0, 0.0, -15.0])
        
        # 🔧 降低运动阈值，确保位置更新
        self.movement_threshold = 0.01  # 从0.05降低到0.01，适应小步移动
        self.accumulated_distance = 0.0  # 🔧 添加累积距离
        self.path_integration_strength = 2.0
        self.activity_injection_radius = 1.5
        
        # 网络状态
        self.activity = np.zeros((self.neurons_per_dimension, self.neurons_per_dimension, self.neurons_per_dimension))
        self.last_position = None
        self.last_odometry_time = None
        self.position_history = []
        self.neuron_history = []
        
        # 统计信息
        self.update_count = 0
        self.position_updates = 0
        self.visual_updates = 0
        self.total_distance = 0.0
        
        # 🔧 优化日志输出频率
        self.major_interval = 500  # 从100增加到500
        self.minor_interval = 100   # 从20增加到100
        
        # 订阅者
        self.odom_sub = self.create_subscription(
            Odometry, 
            '/dolphin_slam/odometry',
            self.odometry_callback, 
            10
        )
        
        self.visual_sub = self.create_subscription(
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
        self.update_timer = self.create_timer(0.1, self.update_network)  # 10Hz更新频率
        
        self.get_logger().info(f'🎯 修复版位置细胞网络初始化')
        self.get_logger().info(f'🧠 网络配置: {self.neurons_per_dimension}³神经元, 空间尺度={self.spatial_scale}m/神经元')
        self.get_logger().info(f'🌍 工作空间中心: {self.workspace_center}')
        self.get_logger().info(f'📏 覆盖范围: {self.neurons_per_dimension * self.spatial_scale}m')
        self.get_logger().info(f'🔧 运动阈值: {self.movement_threshold}m (已降低)')
        
    def odometry_callback(self, msg: Odometry):
        """处理里程计数据"""
        # 🔧 大幅降低调试日志频率 - 只在每1000次回调时显示
        if not hasattr(self, '_odom_callback_count'):
            self._odom_callback_count = 0
        self._odom_callback_count += 1
        
        if self._odom_callback_count % 1000 == 1:
            self.get_logger().info(f'📡 里程计回调 #{self._odom_callback_count}: 位置=({msg.pose.pose.position.x:.2f}, {msg.pose.pose.position.y:.2f}, {msg.pose.pose.position.z:.2f})')
        
        current_time = self.get_clock().now()
        
        # 提取位置信息
        current_pos = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])
        
        # 记录位置历史
        self.position_history.append(current_pos.copy())
        if len(self.position_history) > 100:
            self.position_history.pop(0)
        
        # 路径积分更新
        if self.last_position is not None:
            displacement = current_pos - self.last_position
            distance = np.linalg.norm(displacement)
            
            # 🔧 累积移动距离
            self.accumulated_distance += distance
            
            # 🔧 大幅降低调试信息频率 - 只在每500次时显示
            if self._odom_callback_count % 500 == 1:
                self.get_logger().info(f'📏 移动距离: {distance:.3f}m, 累积: {self.accumulated_distance:.3f}m, 阈值: {self.movement_threshold}m')
            
            # 🔧 使用累积距离判断是否触发位置更新
            if self.accumulated_distance > self.movement_threshold:
                # 计算平均位移方向
                avg_displacement = displacement * (self.movement_threshold / self.accumulated_distance)
                
                self._apply_enhanced_path_integration(avg_displacement)
                self.total_distance += self.accumulated_distance
                self.position_updates += 1
                
                # 🔧 大幅降低位置更新日志频率 - 每100次更新显示一次
                if self.position_updates % 100 == 1:
                    self.get_logger().info(f'✅ 位置更新成功! 总更新次数: {self.position_updates}, 累积距离: {self.accumulated_distance:.3f}m')
                
                # 重置累积距离
                self.accumulated_distance = 0.0
                
                # 🔧 减少日志输出频率 - 每100次位置更新显示一次
                if self.position_updates % 100 == 0:
                    center = self._get_activity_center()
                    world_center = self._neuron_to_world_coords(center)
                    self.get_logger().info(
                        f'🚶 路径积分更新#{self.position_updates}: '
                        f'位移={avg_displacement}, 距离={self.movement_threshold:.3f}m, '
                        f'神经元中心={center}, 世界中心={world_center}'
                    )
            else:
                # 🔧 大幅降低调试信息频率 - 只在每1000次时显示
                if self._odom_callback_count % 1000 == 1:
                    self.get_logger().info(f'⚠️ 累积距离不足: {self.accumulated_distance:.3f}m < {self.movement_threshold}m')
        else:
            self.get_logger().info(f'🔄 初始化位置: {current_pos}')
        
        self.last_position = current_pos
        self.last_odometry_time = current_time
        
    def visual_match_callback(self, msg: Float32MultiArray):
        """处理视觉匹配数据"""
        try:
            if len(msg.data) >= 4:
                similarity = msg.data[0]
                template_id = msg.data[1]
                matched = msg.data[2]
                is_novel = msg.data[3]
                
                # 🔧 降低视觉输入阈值，增加调试信息
                if similarity > 0.2:  # 降低阈值从0.3到0.2
                    peak_pos = self._get_activity_peak()
                    visual_strength = similarity * 1.5  # 增强视觉输入强度
                    
                    self._inject_gaussian_activity(
                        peak_pos, 
                        strength=visual_strength,
                        radius=1.5
                    )
                    self.visual_updates += 1
                    
                    # 🔧 减少日志输出频率 - 每20次视觉更新显示一次
                    if self.visual_updates % 20 == 0:
                        center = self._get_activity_center()
                        world_center = self._neuron_to_world_coords(center)
                        self.get_logger().info(
                            f'👁️ 视觉更新#{self.visual_updates}: '
                            f'相似度={similarity:.3f}, '
                            f'模板ID={template_id}, '
                            f'匹配={matched}, '
                            f'新颖={is_novel}, '
                            f'强度={visual_strength:.3f}, '
                            f'神经元中心={center}, '
                            f'世界中心={world_center}'
                        )
                else:
                    if self.visual_updates % 200 == 0:  # 大幅减少低相似度日志频率
                        self.get_logger().info(f'👁️ 视觉输入相似度过低: {similarity:.3f} < 0.2')
            else:
                if self.visual_updates % 200 == 0:  # 大幅减少数据格式问题日志频率
                    self.get_logger().warn(f'⚠️ 视觉匹配数据格式异常: 长度={len(msg.data)}')
                    
        except Exception as e:
            self.get_logger().error(f'❌ 视觉匹配回调失败: {e}')
    
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
        
    def _apply_enhanced_path_integration(self, displacement):
        """🔧 增强版路径积分"""
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
        
        # 🔧 在移动路径上注入活动，增强连续性
        steps = 5  # 增加路径点数量
        for i in range(1, steps):
            intermediate_pos = current_peak + neuron_displacement * (i / steps)
            intermediate_pos = np.clip(intermediate_pos, 0, self.neurons_per_dimension - 1)
            self._inject_gaussian_activity(
                intermediate_pos,
                strength=self.path_integration_strength * 0.6,
                radius=1.0
            )
        
        # 记录神经元历史
        self.neuron_history.append(new_peak.copy())
        if len(self.neuron_history) > 50:
            self.neuron_history.pop(0)
    
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
            self.activity = np.clip(self.activity, 0, 15.0)  # 🔧 增加上限
            
        except Exception as e:
            self.get_logger().error(f"活动注入失败: {e}")
    
    def _get_activity_center(self):
        """获取活动中心"""
        try:
            # 使用加权平均计算活动中心
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
                return [center_x, center_y, center_z]
            else:
                return [8.0, 8.0, 8.0]  # 默认中心
        except Exception as e:
            self.get_logger().error(f"计算活动中心失败: {e}")
            return [8.0, 8.0, 8.0]
    
    def _get_activity_peak(self):
        """获取活动峰值位置"""
        peak_idx = np.unravel_index(np.argmax(self.activity), self.activity.shape)
        return list(peak_idx)
    
    def _compute_network_stats(self):
        """计算网络统计信息"""
        try:
            center = self._get_activity_center()
            peak = self._get_activity_peak()
            max_activity = np.max(self.activity)
            mean_activity = np.mean(self.activity)
            total_activity = np.sum(self.activity)
            
            return {
                'center': center,
                'peak': peak,
                'max_activity': max_activity,
                'mean_activity': mean_activity,
                'total_activity': total_activity
            }
        except Exception as e:
            self.get_logger().error(f"计算网络统计失败: {e}")
            return None
    
    def update_network(self):
        """更新网络"""
        try:
            # 应用衰减
            self.activity *= 0.95  # 🔧 调整衰减率
            
            # 应用高斯平滑
            self.activity = gaussian_filter(self.activity, sigma=0.3)
            
            # 发布活动
            activity_msg = Float32MultiArray()
            activity_msg.data = self.activity.flatten().tolist()
            self.activity_pub.publish(activity_msg)
            
            self.update_count += 1
            
            # 定期报告状态
            if self.update_count % self.major_interval == 0:
                self.report_status()
                
        except Exception as e:
            self.get_logger().error(f"网络更新失败: {e}")
    
    def report_status(self):
        """报告网络状态 - 大幅精简"""
        try:
            stats = self._compute_network_stats()
            if stats:
                center = stats['center']
                world_center = self._neuron_to_world_coords(center)
                
                # 🔧 大幅精简状态报告 - 只显示关键信息
                if self.update_count % 500 == 0:  # 大幅减少状态报告频率
                    self.get_logger().info(
                        f'📊 网络状态: 更新{self.update_count}, 位置更新{self.position_updates}, '
                        f'视觉更新{self.visual_updates}, 总距离{self.total_distance:.2f}m, '
                        f'最大活动{stats["max_activity"]:.3f}, 平均活动{stats["mean_activity"]:.3f}'
                    )
                
                # 🔧 大幅降低位置偏差计算频率
                if self.update_count % 2000 == 0:  # 进一步减少位置偏差计算频率
                    if len(self.position_history) > 10:
                        recent_positions = np.array(self.position_history[-10:])
                        mean_position = np.mean(recent_positions, axis=0)
                        position_error = np.linalg.norm(mean_position - world_center)
                        self.get_logger().info(f'🎯 位置偏差: {position_error:.3f}m')
                        
        except Exception as e:
            self.get_logger().error(f"状态报告失败: {e}")

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
