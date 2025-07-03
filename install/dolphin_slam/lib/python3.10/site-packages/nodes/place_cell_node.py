#!/usr/bin/env python3
"""
Dolphin SLAM - 位置细胞网络 ROS2 节点 (完全修复版)
正确导入和使用PlaceCellNetwork类
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
import numpy as np
from typing import Optional
import sys
import os
import traceback
from pathlib import Path

class PlaceCellNode(Node):
    """位置细胞网络 ROS2 节点 (完全修复版)"""
    
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
                ('update_rate', 5.0),  # 降低到5Hz避免过载
                ('activation_threshold', 0.01),
                ('debug_mode', True),
            ]
        )
        
        # 获取参数
        self.update_rate = self.get_parameter('update_rate').value
        self.neurons_per_dimension = self.get_parameter('neurons_per_dimension').value
        self.activation_threshold = self.get_parameter('activation_threshold').value
        self.debug_mode = self.get_parameter('debug_mode').value
        
        # 尝试导入PlaceCellNetwork
        self.place_cell_network = None
        self.import_success = False
        
        try:
            # 方法1: 直接导入
            sys.path.insert(0, str(Path(__file__).parent.parent / 'dolphin_slam'))
            from place_cell_network import PlaceCellNetwork
            
            self.place_cell_network = PlaceCellNetwork(
                neurons_per_dim=self.neurons_per_dimension,
                neurons_step=0.25,
                recurrent_conn_std=2.0,
                input_learning_rate=0.1,
                min_input_age=10,
                weight_function='mexican_hat'
            )
            
            # 强制重置网络并设置初始活动
            self.place_cell_network.reset()
            self.import_success = True
            
            initial_max = np.max(self.place_cell_network.activity)
            active_count = np.sum(self.place_cell_network.activity > self.activation_threshold)
            
            self.get_logger().info(f'✅ 成功导入PlaceCellNetwork!')
            self.get_logger().info(f'   网络尺寸: {self.neurons_per_dimension}³ = {self.neurons_per_dimension**3} 神经元')
            self.get_logger().info(f'   初始最大活动: {initial_max:.3f}')
            self.get_logger().info(f'   活跃神经元数: {active_count}')
            
        except ImportError as e:
            self.get_logger().error(f'❌ 导入PlaceCellNetwork失败: {e}')
            self.get_logger().error(f'   Python路径: {sys.path}')
            self.get_logger().info('🔄 使用备用实现...')
            self.create_fallback_network()
            
        except Exception as e:
            self.get_logger().error(f'❌ 创建PlaceCellNetwork失败: {e}')
            self.get_logger().error(f'   错误详情: {traceback.format_exc()}')
            self.get_logger().info('🔄 使用备用实现...')
            self.create_fallback_network()
        
        # 状态变量
        self.last_odometry: Optional[Odometry] = None
        self.last_position = np.zeros(3)
        self.last_timestamp = None
        self.update_count = 0
        self.odometry_received = False
        
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
        
        self.visualization_pub = self.create_publisher(
            MarkerArray,
            '/place_cells/visualization',
            10
        )
        
        # 定时器
        self.update_timer = self.create_timer(
            1.0 / self.update_rate,
            self.update_network
        )
        
        self.stats_timer = self.create_timer(5.0, self.publish_statistics)
        
        self.get_logger().info(f'位置细胞网络节点已启动: {self.neurons_per_dimension}³ 神经元')
        
    def create_fallback_network(self):
        """创建备用网络实现"""
        self.get_logger().info('🔧 创建备用位置细胞网络...')
        
        # 简单但有效的网络实现
        total_neurons = self.neurons_per_dimension ** 3
        
        # 创建具有初始活动的网络
        self.fallback_activity = np.zeros(total_neurons, dtype=np.float32)
        
        # 在中心区域创建高斯分布的活动
        center_idx = total_neurons // 2
        sigma = total_neurons * 0.1
        
        for i in range(total_neurons):
            dist = abs(i - center_idx)
            self.fallback_activity[i] = np.exp(-dist**2 / (2 * sigma**2)) * 0.8
        
        # 添加一些随机活动
        self.fallback_activity += np.random.random(total_neurons) * 0.2
        
        # 归一化
        self.fallback_activity = np.clip(self.fallback_activity, 0, 1)
        
        initial_max = np.max(self.fallback_activity)
        active_count = np.sum(self.fallback_activity > self.activation_threshold)
        
        self.get_logger().info(f'✅ 备用网络已创建')
        self.get_logger().info(f'   初始最大活动: {initial_max:.3f}')
        self.get_logger().info(f'   活跃神经元数: {active_count}')
        
    def odometry_callback(self, msg: Odometry):
        """处理里程计数据"""
        self.last_odometry = msg
        self.odometry_received = True
        
        current_position = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])
        
        velocity = np.array([
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z
        ])
        
        # 🔧 修复：使用消息时间戳
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        if self.last_timestamp is not None:
            dt = current_time - self.last_timestamp
            
            if 0.001 < dt < 2.0 and self.import_success and self.place_cell_network:
                try:
                    angular_velocity = 0.0
                    self.place_cell_network.path_integration_update(velocity, angular_velocity, dt)
                    
                    if self.debug_mode and self.update_count % 50 == 0:
                        self.get_logger().info(f'🔄 路径积分更新: dt={dt:.3f}s, 速度={np.linalg.norm(velocity):.3f}m/s')
                        
                except Exception as e:
                    self.get_logger().debug(f'路径积分更新失败: {e}')
            
            elif 0.001 < dt < 2.0 and not self.import_success:
                self.update_fallback_network(velocity, dt)
        
        self.last_position = current_position
        self.last_timestamp = current_time
        
    def update_fallback_network(self, velocity, dt):
        """更新备用网络"""
        # 简单的活动传播
        speed = np.linalg.norm(velocity)
        if speed > 0.01:
            # 添加一些动态性
            self.fallback_activity *= 0.98  # 衰减
            
            # 添加新活动
            shift = int(speed * dt * 100) % len(self.fallback_activity)
            if shift > 0:
                self.fallback_activity = np.roll(self.fallback_activity, shift)
                
            # 添加噪声
            self.fallback_activity += np.random.random(len(self.fallback_activity)) * 0.05
            self.fallback_activity = np.clip(self.fallback_activity, 0, 1)
        
    def visual_match_callback(self, msg: Float32MultiArray):
        """处理视觉匹配数据"""
        if len(msg.data) >= 2 and self.import_success and self.place_cell_network:
            template_id = int(msg.data[0])
            similarity = msg.data[1]
            
            try:
                self.place_cell_network.visual_input_update(template_id, similarity)
            except Exception as e:
                self.get_logger().debug(f'视觉输入更新失败: {e}')
        
    def update_network(self):
        """更新神经网络"""
        try:
            self.update_count += 1
            
            if self.import_success and self.place_cell_network:
                # 使用真正的PlaceCellNetwork
                self.place_cell_network.apply_recurrent_dynamics()
                activity_data = self.place_cell_network.activity.flatten()
            else:
                # 使用备用网络
                activity_data = self.fallback_activity
            
            # 发布活动数据
            msg = Float32MultiArray()
            msg.data = activity_data.astype(float).tolist()
            self.activity_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f'网络更新错误: {e}')
            
    def publish_statistics(self):
        """发布统计信息"""
        try:
            if self.import_success and self.place_cell_network:
                activity_data = self.place_cell_network.activity.flatten()
                center = self.place_cell_network.get_activity_center()
            else:
                activity_data = self.fallback_activity
                center = [8, 8, 8]  # 默认中心
                
            max_activity = np.max(activity_data)
            active_neurons = np.sum(activity_data > self.activation_threshold)
            total_neurons = len(activity_data)
            
            status = "真实PlaceCellNetwork" if self.import_success else "备用网络"
            odom_status = "有数据" if self.odometry_received else "无数据"
            
            self.get_logger().info(
                f'网络状态({status}): {active_neurons}/{total_neurons} 神经元活跃, '
                f'最大活动: {max_activity:.3f}, 里程计: {odom_status}, '
                f'活动中心: [{center[0]:.1f}, {center[1]:.1f}, {center[2]:.1f}]'
            )
            
        except Exception as e:
            self.get_logger().error(f'统计发布错误: {e}')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = PlaceCellNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n🛑 用户中断")
    except Exception as e:
        print(f'❌ 节点错误: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
