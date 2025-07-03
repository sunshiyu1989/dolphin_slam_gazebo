#!/usr/bin/env python3
"""
Dolphin SLAM - Robot State Node 修复版本
修复 'x' 键错误，正确处理 latitude/longitude 到 x/y/z 的转换

主要修复：
1. 正确的经纬度到XY坐标转换
2. 确保current_pose字典包含所有必需键  
3. 添加数据验证和错误处理
4. 修复ROS2消息类型转换问题
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
import numpy as np
from scipy.spatial.transform import Rotation
import pandas as pd
import os
from typing import Optional
import time

# 导入RobotState类
try:
    import sys
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'dolphin_slam'))
    from robot_state import RobotState
    print("✅ 成功导入 RobotState 类")
except ImportError as e:
    print(f"❌ 导入 RobotState 失败: {e}")
    RobotState = None

class RobotStateNode(Node):
    """机器人状态 ROS2 节点 (修复版)"""
    
    def __init__(self):
        super().__init__('robot_state_node')
        
        # 声明参数
        self.declare_parameters(
            namespace='',
            parameters=[
                ('navigation_csv', ''),
                ('publish_rate', 10.0),
                ('odom_frame', 'odom'),
                ('base_frame', 'base_link'),
                ('use_ekf', False),
                ('process_noise_std', 0.1),
                ('measurement_noise_std', 0.05),
                ('playback_speed', 1.0),
                ('sync_tolerance', 0.1),
            ]
        )
        
        # 获取参数
        self.navigation_csv = self.get_parameter('navigation_csv').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.playback_speed = self.get_parameter('playback_speed').value
        self.sync_tolerance = self.get_parameter('sync_tolerance').value
        
        # 创建RobotState实例
        if RobotState:
            self.robot_state = RobotState(
                dvl_position=np.zeros(3),
                dvl_orientation=np.zeros(3),
                use_ekf=self.get_parameter('use_ekf').value,
                process_noise_std=self.get_parameter('process_noise_std').value,
                measurement_noise_std=self.get_parameter('measurement_noise_std').value
            )
        else:
            self.robot_state = None
            self.get_logger().error('RobotState类未能导入，使用简化实现')
        
        # TF广播器
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # 导航数据相关
        self.navigation_data = None
        self.nav_data_index = 0
        self.data_loaded = False
        self.playback_start_wall_time = None
        self.playback_start_data_time = None
        self.processed_count = 0
        
        # 🔧 修复：正确初始化 current_pose，确保包含所有必需的键
        self.current_pose = {
            'x': 0.0, 'y': 0.0, 'z': 0.0,
            'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0
        }
        self.current_velocity = {
            'vx': 0.0, 'vy': 0.0, 'vz': 0.0,
            'wx': 0.0, 'wy': 0.0, 'wz': 0.0
        }
        
        # 坐标转换参考点（初始化为None）
        self.origin_lat = None
        self.origin_lon = None
        
        # 加载导航数据
        if self.navigation_csv:
            self.load_navigation_data()
        
        # 发布者
        self.odometry_pub = self.create_publisher(
            Odometry,
            '/dolphin_slam/odometry',
            10
        )
        
        # 定时器
        self.publish_timer = self.create_timer(
            1.0 / self.publish_rate,
            self.publish_state
        )
        
        # 导航数据播放定时器
        if self.data_loaded:
            self.nav_timer = self.create_timer(
                0.02,  # 50Hz检查
                self.update_navigation_playback
            )
            
        self.get_logger().info('机器人状态节点已启动 - 修复版')
        
    def load_navigation_data(self):
        """加载导航数据"""
        try:
            if not os.path.exists(self.navigation_csv):
                self.get_logger().error(f'导航文件不存在: {self.navigation_csv}')
                return
                
            # 读取CSV数据
            self.navigation_data = pd.read_csv(self.navigation_csv)
            
            # 🔧 修复：强制转换数据类型
            required_columns = ['timestamp', 'latitude', 'longitude', 'depth', 'roll', 'pitch', 'yaw']
            missing_columns = [col for col in required_columns if col not in self.navigation_data.columns]
            
            if missing_columns:
                self.get_logger().error(f'缺少必需列: {missing_columns}')
                return
            
            # 数据类型转换 - 修复 ROS2 消息创建错误
            for col in required_columns:
                self.navigation_data[col] = pd.to_numeric(self.navigation_data[col], errors='coerce')
                
            # 转换速度列
            velocity_columns = ['velocity_x', 'velocity_y', 'velocity_z']
            for col in velocity_columns:
                if col in self.navigation_data.columns:
                    self.navigation_data[col] = pd.to_numeric(self.navigation_data[col], errors='coerce')
            
            # 删除无效行
            self.navigation_data = self.navigation_data.dropna()
            
            # 按时间戳排序
            self.navigation_data = self.navigation_data.sort_values('timestamp').reset_index(drop=True)
            
            self.data_loaded = True
            
            # 🔧 修复：初始化坐标转换参考点
            if len(self.navigation_data) > 0:
                first_row = self.navigation_data.iloc[0]
                self.origin_lat = float(first_row['latitude'])
                self.origin_lon = float(first_row['longitude'])
                self.get_logger().info(f'坐标转换原点: lat={self.origin_lat:.6f}, lon={self.origin_lon:.6f}')
            
            self.get_logger().info(f'成功加载导航数据: {len(self.navigation_data)} 条记录')
            self.get_logger().info(f'数据类型转换完成')
            
            # 如果有RobotState，也加载到那里
            if self.robot_state:
                self.robot_state.load_navigation_data(self.navigation_csv)
            
        except Exception as e:
            self.get_logger().error(f'加载导航数据失败: {e}')
            self.data_loaded = False
    
    def convert_lat_lon_to_xy(self, latitude, longitude, depth):
        """
        将经纬度转换为局部XY坐标
        
        参数:
            latitude: 纬度 (度)
            longitude: 经度 (度) 
            depth: 深度 (米)
            
        返回:
            (x, y, z): 局部坐标 (米)
        """
        if self.origin_lat is None or self.origin_lon is None:
            return 0.0, 0.0, 0.0
            
        # 简化的墨卡托投影（适用于小范围）
        R_earth = 6371000  # 地球半径（米）
        
        lat_rad = np.radians(float(latitude))
        origin_lat_rad = np.radians(self.origin_lat)
        
        x = R_earth * np.radians(float(longitude) - self.origin_lon) * np.cos(origin_lat_rad)
        y = R_earth * np.radians(float(latitude) - self.origin_lat)
        z = -float(depth)  # 深度为负值
        
        return x, y, z
            
    def update_navigation_playback(self):
        """更新导航数据播放 - 修复版本"""
        if not self.data_loaded or self.navigation_data is None:
            return
        
        # 初始化播放时间
        if self.playback_start_wall_time is None:
            self.playback_start_wall_time = time.time()
            self.playback_start_data_time = self.navigation_data['timestamp'].iloc[0]
            self.get_logger().info(f'开始播放导航数据，起始时间戳: {self.playback_start_data_time}')
        
        # 计算当前应该播放到的数据时间
        current_wall_time = time.time()
        elapsed_wall_time = (current_wall_time - self.playback_start_wall_time) * self.playback_speed
        target_data_time = self.playback_start_data_time + elapsed_wall_time
        
        # 播放所有应该播放的数据点
        updates_this_cycle = 0
        max_updates_per_cycle = 10
        
        while (self.nav_data_index < len(self.navigation_data) and 
               updates_this_cycle < max_updates_per_cycle):
            
            current_data_time = self.navigation_data['timestamp'].iloc[self.nav_data_index]
            
            # 检查是否到了播放时间
            if current_data_time <= target_data_time + self.sync_tolerance:
                # 更新状态
                row = self.navigation_data.iloc[self.nav_data_index]
                
                # 🔧 修复：正确的坐标转换
                x, y, z = self.convert_lat_lon_to_xy(
                    row['latitude'], 
                    row['longitude'], 
                    row['depth']
                )
                
                # 🔧 修复：确保 current_pose 包含所有必需的键
                self.current_pose = {
                    'x': float(x),
                    'y': float(y), 
                    'z': float(z),
                    'roll': float(row['roll']),
                    'pitch': float(row['pitch']),
                    'yaw': float(row['yaw'])
                }
                
                # 计算速度（简化版本）
                if self.nav_data_index > 0:
                    prev_row = self.navigation_data.iloc[self.nav_data_index - 1]
                    dt = current_data_time - prev_row['timestamp']
                    
                    if dt > 0:
                        # 使用CSV中的速度数据（如果有）
                        if all(col in row for col in ['velocity_x', 'velocity_y', 'velocity_z']):
                            self.current_velocity = {
                                'vx': float(row['velocity_x']),
                                'vy': float(row['velocity_y']),
                                'vz': float(row['velocity_z']),
                                'wx': 0.0, 'wy': 0.0, 'wz': 0.0
                            }
                        else:
                            # 从位置差分计算速度
                            prev_x, prev_y, prev_z = self.convert_lat_lon_to_xy(
                                prev_row['latitude'], 
                                prev_row['longitude'], 
                                prev_row['depth']
                            )
                            self.current_velocity = {
                                'vx': (x - prev_x) / dt,
                                'vy': (y - prev_y) / dt,
                                'vz': (z - prev_z) / dt,
                                'wx': 0.0, 'wy': 0.0, 'wz': 0.0
                            }
                
                self.nav_data_index += 1
                updates_this_cycle += 1
                self.processed_count += 1
                
                # 定期报告进度
                if self.processed_count % 100 == 0:
                    progress = (self.nav_data_index / len(self.navigation_data)) * 100
                    self.get_logger().info(
                        f'已处理 {self.processed_count} 条导航记录 ({progress:.1f}%)'
                    )
                
            else:
                # 还没到播放时间
                break
        
        # 检查播放完成
        if self.nav_data_index >= len(self.navigation_data):
            self.get_logger().info('导航数据播放完成')
            self.nav_timer.cancel()
            
    def publish_state(self):
        """发布机器人状态 - 修复版本"""
        try:
            # 🔧 修复：确保 current_pose 和 current_velocity 都存在所需的键
            if not all(key in self.current_pose for key in ['x', 'y', 'z', 'roll', 'pitch', 'yaw']):
                self.get_logger().debug('current_pose 缺少必要的键，跳过发布')
                return
                
            if not all(key in self.current_velocity for key in ['vx', 'vy', 'vz', 'wx', 'wy', 'wz']):
                self.get_logger().debug('current_velocity 缺少必要的键，跳过发布')
                return
            
            # 创建里程计消息
            odom_msg = Odometry()
            odom_msg.header.stamp = self.get_clock().now().to_msg()
            odom_msg.header.frame_id = self.odom_frame
            odom_msg.child_frame_id = self.base_frame
            
            # 位置
            odom_msg.pose.pose.position.x = float(self.current_pose['x'])
            odom_msg.pose.pose.position.y = float(self.current_pose['y'])
            odom_msg.pose.pose.position.z = float(self.current_pose['z'])
            
            # 姿态
            q = Rotation.from_euler('xyz', [
                float(self.current_pose['roll']),
                float(self.current_pose['pitch']), 
                float(self.current_pose['yaw'])
            ]).as_quat()
            
            odom_msg.pose.pose.orientation.x = float(q[0])
            odom_msg.pose.pose.orientation.y = float(q[1])
            odom_msg.pose.pose.orientation.z = float(q[2])
            odom_msg.pose.pose.orientation.w = float(q[3])
            
            # 速度
            odom_msg.twist.twist.linear.x = float(self.current_velocity['vx'])
            odom_msg.twist.twist.linear.y = float(self.current_velocity['vy'])
            odom_msg.twist.twist.linear.z = float(self.current_velocity['vz'])
            odom_msg.twist.twist.angular.x = float(self.current_velocity['wx'])
            odom_msg.twist.twist.angular.y = float(self.current_velocity['wy'])
            odom_msg.twist.twist.angular.z = float(self.current_velocity['wz'])
            
            # 发布
            self.odometry_pub.publish(odom_msg)
            
        except Exception as e:
            self.get_logger().error(f'发布状态失败: {e}')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = RobotStateNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n🛑 用户中断")
    except Exception as e:
        print(f'❌ 节点错误: {e}')
        import traceback
        traceback.print_exc()
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
