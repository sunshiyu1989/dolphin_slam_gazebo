#!/usr/bin/env python3
"""
Dolphin SLAM - 数据集播放器节点
从 AUV-Based Multi-Sensor Dataset 读取并发布数据
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import TwistWithCovarianceStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import numpy as np
import pandas as pd
import os
from typing import Optional, Dict, List
import time
from threading import Thread, Lock
import queue

class DatasetPlayerNode(Node):
    """从 AUV 数据集读取并发布传感器数据"""
    
    def __init__(self):
        super().__init__('dataset_player_node')
        
        # 声明参数
        self.declare_parameters(
            namespace='',
            parameters=[
                ('dataset_path', ''),
                ('playback_speed', 1.0),
                ('loop', False),
                ('start_time', 0.0),
                ('end_time', -1.0),
                ('publish_compressed', False),
                ('camera_topic', '/camera/image_raw'),
                ('sonar_topic', '/sonar/image_raw'),
                ('odometry_topic', '/robot/odometry'),
                ('imu_topic', '/imu/data'),
            ]
        )
        
        # 获取参数
        self.dataset_path = self.get_parameter('dataset_path').value
        self.playback_speed = self.get_parameter('playback_speed').value
        self.loop = self.get_parameter('loop').value
        self.start_time = self.get_parameter('start_time').value
        self.end_time = self.get_parameter('end_time').value
        self.publish_compressed = self.get_parameter('publish_compressed').value
        
        if not self.dataset_path:
            self.get_logger().error('数据集路径未指定！')
            return
            
        # CV Bridge
        self.bridge = CvBridge()
        
        # 发布者
        if self.publish_compressed:
            self.camera_pub = self.create_publisher(
                CompressedImage,
                self.get_parameter('camera_topic').value + '/compressed',
                10
            )
            self.sonar_pub = self.create_publisher(
                CompressedImage,
                self.get_parameter('sonar_topic').value + '/compressed',
                10
            )
        else:
            self.camera_pub = self.create_publisher(
                Image,
                self.get_parameter('camera_topic').value,
                10
            )
            self.sonar_pub = self.create_publisher(
                Image,
                self.get_parameter('sonar_topic').value,
                10
            )
            
        self.odometry_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            self.get_parameter('odometry_topic').value,
            10
        )
        
        self.imu_pub = self.create_publisher(
            Imu,
            self.get_parameter('imu_topic').value,
            10
        )
        
        # 数据队列
        self.data_queue = queue.PriorityQueue()
        self.is_playing = False
        self.playback_thread = None
        self.lock = Lock()
        
        # 加载数据集
        self.load_dataset()
        
        # 开始播放
        self.start_playback()
        
    def load_dataset(self):
        """加载数据集文件"""
        self.get_logger().info(f'加载数据集: {self.dataset_path}')
        
        try:
            # 加载 CSV 文件
            self.camera_csv = pd.read_csv(
                os.path.join(self.dataset_path, 'camera.csv')
            )
            self.sonar_csv = pd.read_csv(
                os.path.join(self.dataset_path, 'sonar.csv')
            )
            self.navigation_csv = pd.read_csv(
                os.path.join(self.dataset_path, 'navigation.csv')
            )
            
            # 验证时间戳列
            for df, name in [(self.camera_csv, 'camera'),
                           (self.sonar_csv, 'sonar'),
                           (self.navigation_csv, 'navigation')]:
                if 'timestamp' not in df.columns:
                    raise ValueError(f'{name}.csv 缺少 timestamp 列')
                    
            # 获取时间范围
            self.min_time = min(
                self.camera_csv['timestamp'].min(),
                self.sonar_csv['timestamp'].min(),
                self.navigation_csv['timestamp'].min()
            )
            
            self.max_time = max(
                self.camera_csv['timestamp'].max(),
                self.sonar_csv['timestamp'].max(),
                self.navigation_csv['timestamp'].max()
            )
            
            # 设置播放范围
            if self.start_time == 0.0:
                self.start_time = self.min_time
            else:
                self.start_time = max(self.min_time, self.start_time)
                
            if self.end_time < 0:
                self.end_time = self.max_time
            else:
                self.end_time = min(self.max_time, self.end_time)
                
            self.get_logger().info(
                f'数据集时间范围: {self.min_time:.2f} - {self.max_time:.2f} 秒'
            )
            self.get_logger().info(
                f'播放范围: {self.start_time:.2f} - {self.end_time:.2f} 秒'
            )
            
            # 构建数据队列
            self._build_data_queue()
            
        except Exception as e:
            self.get_logger().error(f'加载数据集失败: {e}')
            raise
            
    def _build_data_queue(self):
        """构建按时间排序的数据队列"""
        # 相机数据
        for _, row in self.camera_csv.iterrows():
            if self.start_time <= row['timestamp'] <= self.end_time:
                self.data_queue.put((
                    row['timestamp'],
                    'camera',
                    row['filename']
                ))
                
        # 声呐数据
        for _, row in self.sonar_csv.iterrows():
            if self.start_time <= row['timestamp'] <= self.end_time:
                self.data_queue.put((
                    row['timestamp'],
                    'sonar',
                    row['filename']
                ))
                
        # 导航数据
        for _, row in self.navigation_csv.iterrows():
            if self.start_time <= row['timestamp'] <= self.end_time:
                self.data_queue.put((
                    row['timestamp'],
                    'navigation',
                    row
                ))
                
        self.get_logger().info(f'数据队列大小: {self.data_queue.qsize()}')
        
    def start_playback(self):
        """开始播放数据"""
        with self.lock:
            if not self.is_playing:
                self.is_playing = True
                self.playback_thread = Thread(target=self._playback_loop)
                self.playback_thread.start()
                self.get_logger().info('开始播放数据集')
                
    def stop_playback(self):
        """停止播放"""
        with self.lock:
            self.is_playing = False
            
        if self.playback_thread:
            self.playback_thread.join()
            
        self.get_logger().info('停止播放数据集')
        
    def _playback_loop(self):
        """播放循环"""
        start_wall_time = time.time()
        start_data_time = self.start_time
        
        while self.is_playing and not self.data_queue.empty():
            # 获取下一个数据项
            timestamp, data_type, data = self.data_queue.get()
            
            # 计算等待时间
            elapsed_wall_time = time.time() - start_wall_time
            elapsed_data_time = (timestamp - start_data_time) / self.playback_speed
            
            wait_time = elapsed_data_time - elapsed_wall_time
            if wait_time > 0:
                time.sleep(wait_time)
                
            # 发布数据
            if data_type == 'camera':
                self._publish_camera_image(timestamp, data)
            elif data_type == 'sonar':
                self._publish_sonar_image(timestamp, data)
            elif data_type == 'navigation':
                self._publish_navigation(timestamp, data)
                
        # 如果循环播放
        if self.loop and self.is_playing:
            self.get_logger().info('重新开始播放')
            self._build_data_queue()
            self._playback_loop()
        else:
            self.get_logger().info('播放完成')
            self.is_playing = False
            
    def _publish_camera_image(self, timestamp: float, filename: str):
        """发布相机图像"""
        try:
            # 读取图像
            image_path = os.path.join(self.dataset_path, 'camera', filename)
            if not os.path.exists(image_path):
                self.get_logger().warning(f'图像文件不存在: {image_path}')
                return
                
            image = cv2.imread(image_path)
            if image is None:
                self.get_logger().warning(f'无法读取图像: {image_path}')
                return
                
            # 创建消息
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = 'camera_link'
            
            if self.publish_compressed:
                # 压缩图像
                _, compressed = cv2.imencode('.jpg', image)
                msg = CompressedImage()
                msg.header = header
                msg.format = 'jpeg'
                msg.data = compressed.tobytes()
            else:
                # 原始图像
                msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
                msg.header = header
                
            self.camera_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f'发布相机图像失败: {e}')
            
    def _publish_sonar_image(self, timestamp: float, filename: str):
        """发布声呐图像"""
        try:
            # 读取图像
            image_path = os.path.join(self.dataset_path, 'sonar', filename)
            if not os.path.exists(image_path):
                self.get_logger().warning(f'声呐文件不存在: {image_path}')
                return
                
            # 声呐数据可能是特殊格式，这里假设是标准图像
            image = cv2.imread(image_path, cv2.IMREAD_ANYDEPTH)
            if image is None:
                self.get_logger().warning(f'无法读取声呐图像: {image_path}')
                return
                
            # 创建消息
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = 'sonar_link'
            
            if self.publish_compressed:
                # 归一化到 8 位用于压缩
                image_norm = cv2.normalize(image, None, 0, 255, cv2.NORM_MINMAX)
                image_8bit = image_norm.astype(np.uint8)
                _, compressed = cv2.imencode('.png', image_8bit)
                msg = CompressedImage()
                msg.header = header
                msg.format = 'png'
                msg.data = compressed.tobytes()
            else:
                # 原始图像
                if image.dtype == np.uint16:
                    msg = self.bridge.cv2_to_imgmsg(image, encoding='16UC1')
                else:
                    msg = self.bridge.cv2_to_imgmsg(image, encoding='32FC1')
                msg.header = header
                
            self.sonar_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f'发布声呐图像失败: {e}')
            
    def _publish_navigation(self, timestamp: float, nav_data):
        """发布导航数据"""
        try:
            # 创建里程计消息
            odom_msg = PoseWithCovarianceStamped()
            odom_msg.header.stamp = self.get_clock().now().to_msg()
            odom_msg.header.frame_id = 'odom'
            
            # 位置（需要先转换经纬度到局部坐标）
            # 这里使用简化的转换，实际应该使用 UTM 或其他投影
            R_earth = 6371000
            origin_lat = self.navigation_csv['latitude'].iloc[0]
            origin_lon = self.navigation_csv['longitude'].iloc[0]
            
            lat_rad = np.radians(nav_data['latitude'])
            lon_rad = np.radians(nav_data['longitude'])
            origin_lat_rad = np.radians(origin_lat)
            origin_lon_rad = np.radians(origin_lon)
            
            x = R_earth * (lon_rad - origin_lon_rad) * np.cos(origin_lat_rad)
            y = R_earth * (lat_rad - origin_lat_rad)
            z = -nav_data['depth']
            
            odom_msg.pose.pose.position.x = x
            odom_msg.pose.pose.position.y = y
            odom_msg.pose.pose.position.z = z
            
            # 姿态
            from scipy.spatial.transform import Rotation
            r = Rotation.from_euler('xyz', [
                np.radians(nav_data['roll']),
                np.radians(nav_data['pitch']),
                np.radians(nav_data['yaw'])
            ])
            q = r.as_quat()  # [x, y, z, w]
            
            odom_msg.pose.pose.orientation.x = q[0]
            odom_msg.pose.pose.orientation.y = q[1]
            odom_msg.pose.pose.orientation.z = q[2]
            odom_msg.pose.pose.orientation.w = q[3]
            
            # 协方差（简化）
            odom_msg.pose.covariance = [0.1] * 36
            
            self.odometry_pub.publish(odom_msg)
            
            # IMU 消息
            imu_msg = Imu()
            imu_msg.header = odom_msg.header
            imu_msg.header.frame_id = 'imu_link'
            
            # 姿态
            imu_msg.orientation = odom_msg.pose.pose.orientation
            
            # 角速度（从导航数据计算，如果可用）
            imu_msg.angular_velocity.x = 0.0
            imu_msg.angular_velocity.y = 0.0
            imu_msg.angular_velocity.z = 0.0
            
            # 线加速度（暂时设为 0）
            imu_msg.linear_acceleration.x = 0.0
            imu_msg.linear_acceleration.y = 0.0
            imu_msg.linear_acceleration.z = 9.81
            
            self.imu_pub.publish(imu_msg)
            
        except Exception as e:
            self.get_logger().error(f'发布导航数据失败: {e}')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = DatasetPlayerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'错误: {e}')
    finally:
        if rclpy.ok():
            node.stop_playback()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
