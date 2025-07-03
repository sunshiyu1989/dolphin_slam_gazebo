#!/usr/bin/env python3
"""
修复版 place_cell_node.py - 实现真正的空间表征和视觉集成
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import MarkerArray, Marker
import numpy as np
from typing import Optional
import time

class PlaceCellNode(Node):
    """位置细胞网络 ROS2 节点 - 真实空间表征版"""
    
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
                ('major_report_interval', 1000),
                ('spatial_scale', 2.0),                    # 真实世界米 -> 神经元网格
                ('visual_similarity_threshold', 0.1),      # 降低视觉阈值
                ('enable_path_integration', True),         # 启用路径积分
                ('enable_visual_debug', True),             # 启用视觉调试
            ]
        )
        
        # 获取参数
        self.update_rate = self.get_parameter('update_rate').value
        self.neurons_per_dimension = self.get_parameter('neurons_per_dimension').value
        self.major_interval = self.get_parameter('major_report_interval').value
        self.spatial_scale = self.get_parameter('spatial_scale').value
        self.visual_threshold = self.get_parameter('visual_similarity_threshold').value
        self.enable_path_integration = self.get_parameter('enable_path_integration').value
        self.visual_debug = self.get_parameter('enable_visual_debug').value
        
        # 状态变量
        self.last_odometry: Optional[Odometry] = None
        self.last_position = np.array([0.0, 0.0, 0.0])
        self.origin_position = None  # 记录起始位置
        self.update_count = 0
        self.position_updates = 0
        self.significant_updates = 0
        
        # 初始化真正的位置细胞网络
        total_neurons = self.neurons_per_dimension ** 3
        self.activity_data = np.zeros(total_neurons)
        
        # 在网格中心创建初始活动热点
        center_3d = (self.neurons_per_dimension // 2, 
                     self.neurons_per_dimension // 2, 
                     self.neurons_per_dimension // 2)
        self._inject_activity_3d(center_3d, strength=1.0, radius=2.0)
        
        # 记录视觉匹配统计
        self.visual_match_count = 0
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
        
        self.stats_pub = self.create_publisher(
            MarkerArray,
            '/place_cells/statistics',
            10
        )
        
        # 定时器
        self.update_timer = self.create_timer(
            1.0 / self.update_rate,
            self.update_network
        )
        
        self.get_logger().info(f'位置细胞网络节点已启动: {self.neurons_per_dimension}³ 神经元')
        self.get_logger().info(f'空间缩放比例: {self.spatial_scale} 米/神经元, 视觉阈值: {self.visual_threshold}')
        
    def odometry_callback(self, msg: Odometry):
        """处理里程计数据 - 实现真正的路径积分"""
        self.last_odometry = msg
        
        # 提取当前位置
        current_position = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])
        
        # 设置原点
        if self.origin_position is None:
            self.origin_position = current_position.copy()
            self.get_logger().info(f'设置原点位置: ({current_position[0]:.2f}, {current_position[1]:.2f}, {current_position[2]:.2f})')
            return
        
        # 计算相对于原点的位移
        relative_position = current_position - self.origin_position
        displacement = relative_position - self.last_position
        displacement_magnitude = np.linalg.norm(displacement)
        
        if displacement_magnitude > 0.01:  # 有显著移动
            if self.enable_path_integration:
                self.apply_path_integration(relative_position, displacement)
            
            self.last_position = relative_position.copy()
            self.position_updates += 1
            
            # 位置更新播报
            if self.position_updates % 100 == 0:
                center = self.get_activity_center()
                world_center = self._neuron_to_world_coords(center)
                
                self.get_logger().info(
                    f'📍 位置更新#{self.position_updates}: '
                    f'世界坐标({current_position[0]:.2f}, {current_position[1]:.2f}, {current_position[2]:.2f}), '
                    f'相对位置({relative_position[0]:.2f}, {relative_position[1]:.2f}, {relative_position[2]:.2f}), '
                    f'神经元中心({center[0]:.1f}, {center[1]:.1f}, {center[2]:.1f}), '
                    f'对应世界({world_center[0]:.1f}, {world_center[1]:.1f}, {world_center[2]:.1f})'
                )
        
    def visual_match_callback(self, msg: Float32MultiArray):
        """处理视觉匹配数据 - 增强调试信息"""
        self.visual_match_count += 1
        
        if len(msg.data) > 0:
            similarity = msg.data[0]
            self.visual_similarities.append(similarity)
            
            # 调试信息：显示所有接收到的视觉匹配
            if self.visual_debug and self.visual_match_count % 100 == 0:
                recent_sims = self.visual_similarities[-10:]  # 最近10个
                avg_sim = np.mean(recent_sims)
                max_sim = np.max(recent_sims)
                
                self.get_logger().info(
                    f'👁️  视觉调试#{self.visual_match_count}: 当前相似度={similarity:.3f}, '
                    f'最近10次平均={avg_sim:.3f}, 最大={max_sim:.3f}, 阈值={self.visual_threshold}'
                )
            
            # 使用更低的阈值检测视觉输入
            if similarity > self.visual_threshold:
                self.apply_visual_input(similarity)
                self.significant_updates += 1
                
                if self.visual_debug and self.significant_updates % 50 == 0:
                    self.get_logger().info(f'✅ 视觉输入生效: 相似度={similarity:.3f} > 阈值={self.visual_threshold}')
        else:
            if self.visual_debug and self.visual_match_count % 100 == 0:
                self.get_logger().warn(f'⚠️ 接收到空的视觉匹配数据 (#{self.visual_match_count})')
    
    def apply_path_integration(self, relative_position, displacement):
        """实现真正的路径积分"""
        # 将世界坐标转换为神经元网格坐标
        neuron_position = self._world_to_neuron_coords(relative_position)
        
        # 检查是否在神经元网格范围内
        if self._is_valid_neuron_position(neuron_position):
            # 在新位置创建活动
            self._inject_activity_3d(neuron_position, strength=0.8, radius=1.5)
            
            # 全局活动衰减
            self.activity_data *= 0.95
        else:
            # 超出范围时的处理
            self.get_logger().debug(f'位置超出神经网络范围: {neuron_position}')
    
    def apply_visual_input(self, similarity):
        """基于视觉输入增强当前活动区域"""
        # 找到当前活动最强的区域
        activity_3d = self.activity_data.reshape((self.neurons_per_dimension,) * 3)
        max_pos = np.unravel_index(np.argmax(activity_3d), activity_3d.shape)
        
        # 在活动峰值周围增强
        enhancement = similarity * 0.3  # 增强强度
        self._inject_activity_3d(max_pos, strength=enhancement, radius=2.0)
    
    def _world_to_neuron_coords(self, world_pos):
        """世界坐标到神经元坐标的转换"""
        # 应用空间缩放
        neuron_pos = world_pos / self.spatial_scale
        
        # 平移到网格中心
        center_offset = self.neurons_per_dimension / 2
        neuron_coords = neuron_pos + center_offset
        
        return neuron_coords
    
    def _neuron_to_world_coords(self, neuron_pos):
        """神经元坐标到世界坐标的转换"""
        center_offset = self.neurons_per_dimension / 2
        world_pos = (neuron_pos - center_offset) * self.spatial_scale
        return world_pos
    
    def _is_valid_neuron_position(self, neuron_pos):
        """检查神经元位置是否在有效范围内"""
        return (0 <= neuron_pos[0] < self.neurons_per_dimension and
                0 <= neuron_pos[1] < self.neurons_per_dimension and
                0 <= neuron_pos[2] < self.neurons_per_dimension)
    
    def _inject_activity_3d(self, center_pos, strength=1.0, radius=1.0):
        """在3D位置注入高斯分布的活动"""
        activity_3d = self.activity_data.reshape((self.neurons_per_dimension,) * 3)
        
        # 创建3D高斯分布
        x, y, z = np.meshgrid(
            np.arange(self.neurons_per_dimension),
            np.arange(self.neurons_per_dimension),
            np.arange(self.neurons_per_dimension),
            indexing='ij'
        )
        
        # 计算到中心的距离
        dist_sq = ((x - center_pos[0])**2 + 
                   (y - center_pos[1])**2 + 
                   (z - center_pos[2])**2)
        
        # 高斯活动分布
        gaussian_activity = strength * np.exp(-dist_sq / (2 * radius**2))
        
        # 添加到现有活动
        activity_3d += gaussian_activity
        
        # 更新1D数组
        self.activity_data = activity_3d.flatten()
    
    def update_network(self):
        """更新神经网络"""
        try:
            self.update_count += 1
            
            # 应用网络动力学
            self.activity_data *= 0.99  # 轻微衰减
            
            # 添加小量噪声维持活动
            noise = np.random.normal(0, 0.001, len(self.activity_data))
            self.activity_data += noise
            
            # 防止负值
            self.activity_data = np.maximum(0, self.activity_data)
            
            # 发布活动数据
            msg = Float32MultiArray()
            msg.data = self.activity_data.tolist()
            self.activity_pub.publish(msg)
            
            # 同步播报
            self._handle_synchronized_reporting()
                
        except Exception as e:
            self.get_logger().error(f'网络更新错误: {e}')
    
    def _handle_synchronized_reporting(self):
        """处理同步播报"""
        # 主要播报
        if self.update_count % self.major_interval == 0:
            stats = self.get_network_stats()
            center = self.get_activity_center()
            world_center = self._neuron_to_world_coords(center)
            
            # 视觉匹配统计
            if len(self.visual_similarities) > 0:
                avg_visual = np.mean(self.visual_similarities[-100:])  # 最近100次的平均
                max_visual = np.max(self.visual_similarities[-100:])
                visual_info = f'视觉: 平均={avg_visual:.3f}, 最大={max_visual:.3f}'
            else:
                visual_info = '视觉: 无数据'
            
            self.get_logger().info(
                f'🧠 网络更新#{self.update_count}: {stats}, '
                f'神经元中心=({center[0]:.1f},{center[1]:.1f},{center[2]:.1f}), '
                f'对应世界=({world_center[0]:.1f},{world_center[1]:.1f},{world_center[2]:.1f}), '
                f'位置更新={self.position_updates}次, 视觉更新={self.significant_updates}次, {visual_info}'
            )
            
            self.publish_statistics()
    
    def get_network_stats(self):
        """获取网络统计信息"""
        max_activity = np.max(self.activity_data)
        mean_activity = np.mean(self.activity_data)
        std_activity = np.std(self.activity_data)
        active_neurons = np.sum(self.activity_data > 0.001)  # 降低活跃阈值
        total_neurons = len(self.activity_data)
        active_percentage = 100 * active_neurons / total_neurons
        
        return (f'峰值={max_activity:.4f}, 均值={mean_activity:.4f}±{std_activity:.4f}, '
                f'活跃={active_neurons}/{total_neurons}({active_percentage:.1f}%)')
    
    def get_activity_center(self):
        """计算3D活动中心"""
        try:
            activity_3d = self.activity_data.reshape((self.neurons_per_dimension,) * 3)
            
            total_activity = np.sum(activity_3d)
            if total_activity > 0:
                indices = np.indices(activity_3d.shape)
                center_x = np.sum(indices[0] * activity_3d) / total_activity
                center_y = np.sum(indices[1] * activity_3d) / total_activity
                center_z = np.sum(indices[2] * activity_3d) / total_activity
                return np.array([center_x, center_y, center_z])
            else:
                return np.array([self.neurons_per_dimension/2] * 3)
        except Exception:
            return np.array([self.neurons_per_dimension/2] * 3)
    
    def publish_statistics(self):
        """发布可视化统计信息"""
        try:
            markers = MarkerArray()
            center = self.get_activity_center()
            world_center = self._neuron_to_world_coords(center)
            
            # 活动中心标记
            center_marker = Marker()
            center_marker.header.frame_id = "map"
            center_marker.header.stamp = self.get_clock().now().to_msg()
            center_marker.ns = "place_cell_center"
            center_marker.id = 0
            center_marker.type = Marker.SPHERE
            center_marker.action = Marker.ADD
            
            # 使用世界坐标显示
            center_marker.pose.position.x = float(world_center[0])
            center_marker.pose.position.y = float(world_center[1])
            center_marker.pose.position.z = float(world_center[2])
            center_marker.pose.orientation.w = 1.0
            
            center_marker.scale.x = 1.0
            center_marker.scale.y = 1.0
            center_marker.scale.z = 1.0
            
            max_activity = np.max(self.activity_data)
            center_marker.color.r = min(1.0, max_activity * 20)
            center_marker.color.g = 0.5
            center_marker.color.b = 1.0 - min(1.0, max_activity * 20)
            center_marker.color.a = 0.8
            
            markers.markers.append(center_marker)
            self.stats_pub.publish(markers)
            
        except Exception as e:
            self.get_logger().warn(f'发布统计信息失败: {e}')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = PlaceCellNode()
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