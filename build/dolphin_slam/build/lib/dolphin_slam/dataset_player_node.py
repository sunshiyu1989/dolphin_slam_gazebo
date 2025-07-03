#!/usr/bin/env python3
"""
修复版本的数据集播放器节点 - Dolphin SLAM
解决声呐图像编码问题，支持多种图像格式
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

import os
import time
import queue
import threading
import pandas as pd
import numpy as np
import cv2
from threading import Thread, Lock

# ROS2 消息类型
from sensor_msgs.msg import Image, CompressedImage, Imu
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Header

# CV Bridge
from cv_bridge import CvBridge


class DatasetPlayerNode(Node):
    """
    数据集播放器节点
    
    功能:
    - 同步播放相机、声呐和导航数据
    - 支持多种图像格式
    - 可配置播放速度和循环模式
    """
    
    def __init__(self):
        super().__init__('dataset_player_node')
        
        # 声明参数
        self.declare_parameter('dataset_path', '')
        self.declare_parameter('camera_csv', '')
        self.declare_parameter('sonar_csv', '') 
        self.declare_parameter('navigation_csv', '')
        self.declare_parameter('playback_speed', 1.0)
        self.declare_parameter('loop', False)
        self.declare_parameter('start_time', 0.0)
        self.declare_parameter('end_time', -1.0)
        self.declare_parameter('publish_compressed', False)
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('sonar_topic', '/sonar/image_raw')
        self.declare_parameter('odometry_topic', '/dolphin_slam/pose')
        self.declare_parameter('imu_topic', '/robot/imu')
        # use_sim_time 可能已经被声明，安全声明
        try:
            self.declare_parameter('use_sim_time', True)
        except Exception:
            # 参数已存在，忽略
            pass
        
        # 获取参数
        self.dataset_path = self.get_parameter('dataset_path').value
        self.camera_csv_path = self.get_parameter('camera_csv').value
        self.sonar_csv_path = self.get_parameter('sonar_csv').value
        self.navigation_csv_path = self.get_parameter('navigation_csv').value
        self.playback_speed = self.get_parameter('playback_speed').value
        self.loop = self.get_parameter('loop').value
        self.start_time = self.get_parameter('start_time').value
        self.end_time = self.get_parameter('end_time').value
        self.publish_compressed = self.get_parameter('publish_compressed').value
        self.use_sim_time = self.get_parameter('use_sim_time').value
        
        # 验证数据集路径
        if not self.dataset_path:
            self.get_logger().error('数据集路径未指定！')
            raise ValueError('数据集路径未指定')
            
        if not os.path.exists(self.dataset_path):
            self.get_logger().error(f'数据集路径不存在: {self.dataset_path}')
            raise ValueError(f'数据集路径不存在: {self.dataset_path}')
        
        # 推断CSV文件路径（如果未明确指定）
        if not self.camera_csv_path:
            self.camera_csv_path = os.path.join(self.dataset_path, 'camera.csv')
        if not self.sonar_csv_path:
            self.sonar_csv_path = os.path.join(self.dataset_path, 'sonar.csv')
        if not self.navigation_csv_path:
            nav_paths = [
                os.path.join(self.dataset_path, 'navigation.csv'),
                os.path.join(self.dataset_path, 'navigation', 'navigation.csv')
            ]
            for path in nav_paths:
                if os.path.exists(path):
                    self.navigation_csv_path = path
                    break
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # QoS配置
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # 发布器
        if self.publish_compressed:
            self.camera_pub = self.create_publisher(
                CompressedImage,
                self.get_parameter('camera_topic').value + '/compressed',
                qos_profile
            )
            self.sonar_pub = self.create_publisher(
                CompressedImage,
                self.get_parameter('sonar_topic').value + '/compressed',
                qos_profile
            )
        else:
            self.camera_pub = self.create_publisher(
                Image,
                self.get_parameter('camera_topic').value,
                qos_profile
            )
            self.sonar_pub = self.create_publisher(
                Image,
                self.get_parameter('sonar_topic').value,
                qos_profile
            )
            
        self.odometry_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            self.get_parameter('odometry_topic').value,
            qos_profile
        )
        
        self.imu_pub = self.create_publisher(
            Imu,
            self.get_parameter('imu_topic').value,
            qos_profile
        )
        
        # 数据存储
        self.camera_csv = None
        self.sonar_csv = None
        self.navigation_csv = None
        
        # 播放控制
        self.data_queue = queue.PriorityQueue()
        self.is_playing = False
        self.playback_thread = None
        self.lock = Lock()
        self.start_time_actual = 0.0
        self.end_time_actual = 0.0
        
        # 统计信息
        self.published_count = {'camera': 0, 'sonar': 0, 'navigation': 0}
        self.error_count = {'camera': 0, 'sonar': 0, 'navigation': 0}
        
        # 加载数据集
        self.load_dataset()
        
        # 开始播放
        self.start_playback()
        
    def load_dataset(self):
        """加载数据集文件"""
        self.get_logger().info(f'加载数据集: {self.dataset_path}')
        
        try:
            # 加载 CSV 文件
            csv_files = {
                'camera': self.camera_csv_path,
                'sonar': self.sonar_csv_path,
                'navigation': self.navigation_csv_path
            }
            
            for name, path in csv_files.items():
                if path and os.path.exists(path):
                    df = pd.read_csv(path)
                    setattr(self, f'{name}_csv', df)
                    self.get_logger().info(f'✅ 加载 {name}.csv: {len(df)} 条记录')
                    
                    # 验证时间戳列
                    if 'timestamp' not in df.columns:
                        self.get_logger().warning(f'{name}.csv 缺少 timestamp 列')
                else:
                    self.get_logger().warning(f'❌ 未找到 {name}.csv: {path}')
                    
            # 检查必需文件
            if self.navigation_csv is None:
                raise ValueError('未找到navigation.csv文件')
                
            # 计算时间范围
            time_sources = []
            if self.camera_csv is not None:
                time_sources.append(self.camera_csv['timestamp'])
            if self.sonar_csv is not None:
                time_sources.append(self.sonar_csv['timestamp'])
            if self.navigation_csv is not None:
                time_sources.append(self.navigation_csv['timestamp'])
                
            if time_sources:
                all_times = pd.concat(time_sources)
                self.min_time = all_times.min()
                self.max_time = all_times.max()
            else:
                raise ValueError('没有找到有效的时间戳数据')
                
            # 设置播放范围
            if self.start_time == 0.0:
                self.start_time_actual = self.min_time
            else:
                self.start_time_actual = max(self.min_time, self.start_time)
                
            if self.end_time < 0:
                self.end_time_actual = self.max_time
            else:
                self.end_time_actual = min(self.max_time, self.end_time)
                
            self.get_logger().info(
                f'数据集时间范围: {self.min_time:.2f} - {self.max_time:.2f} 秒'
            )
            self.get_logger().info(
                f'播放范围: {self.start_time_actual:.2f} - {self.end_time_actual:.2f} 秒'
            )
            
            # 构建数据队列
            self._build_data_queue()
            
        except Exception as e:
            self.get_logger().error(f'加载数据集失败: {e}')
            raise
            
    def _build_data_queue(self):
        """构建按时间排序的数据队列"""
        # 相机数据
        if self.camera_csv is not None:
            for _, row in self.camera_csv.iterrows():
                if self.start_time_actual <= row['timestamp'] <= self.end_time_actual:
                    self.data_queue.put((
                        row['timestamp'],
                        'camera',
                        row['filename'] if 'filename' in row else None
                    ))
                    
        # 声呐数据
        if self.sonar_csv is not None:
            for _, row in self.sonar_csv.iterrows():
                if self.start_time_actual <= row['timestamp'] <= self.end_time_actual:
                    self.data_queue.put((
                        row['timestamp'],
                        'sonar',
                        row['filename'] if 'filename' in row else None
                    ))
                    
        # 导航数据
        if self.navigation_csv is not None:
            for _, row in self.navigation_csv.iterrows():
                if self.start_time_actual <= row['timestamp'] <= self.end_time_actual:
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
        last_timestamp = None
        playback_start_time = time.time()
        
        while self.is_playing:
            try:
                if self.data_queue.empty():
                    if self.loop:
                        # 重新构建队列
                        self._build_data_queue()
                        continue
                    else:
                        break
                        
                # 获取下一个数据项
                timestamp, data_type, data = self.data_queue.get(timeout=1.0)
                
                # 计算等待时间
                if last_timestamp is not None:
                    time_diff = (timestamp - last_timestamp) / self.playback_speed
                    if time_diff > 0:
                        time.sleep(time_diff)
                        
                # 发布数据
                if data_type == 'camera':
                    self._publish_camera_image(timestamp, data)
                elif data_type == 'sonar':
                    self._publish_sonar_image(timestamp, data)
                elif data_type == 'navigation':
                    self._publish_navigation(timestamp, data)
                    
                last_timestamp = timestamp
                
            except queue.Empty:
                continue
            except Exception as e:
                self.get_logger().error(f'播放循环错误: {e}')
                break
                
        self.get_logger().info('播放完成')
        
        # 打印统计信息
        total_published = sum(self.published_count.values())
        total_errors = sum(self.error_count.values())
        self.get_logger().info(
            f'播放统计 - 发布: {total_published}, 错误: {total_errors}'
        )
        
    def _publish_camera_image(self, timestamp: float, filename: str):
        """发布相机图像"""
        if not filename:
            return
            
        try:
            # 读取图像
            image_path = os.path.join(self.dataset_path, 'camera', filename)
            if not os.path.exists(image_path):
                self.get_logger().warning(f'相机文件不存在: {image_path}')
                return
                
            # 读取图像
            image = cv2.imread(image_path, cv2.IMREAD_COLOR)
            if image is None:
                self.get_logger().warning(f'无法读取相机图像: {image_path}')
                return
                
            # 创建消息
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = 'camera_link'
            
            if self.publish_compressed:
                # 压缩图像
                _, compressed = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 85])
                msg = CompressedImage()
                msg.header = header
                msg.format = 'jpeg'
                msg.data = compressed.tobytes()
            else:
                # 原始图像
                msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
                msg.header = header
                
            self.camera_pub.publish(msg)
            self.published_count['camera'] += 1
            
        except Exception as e:
            self.get_logger().error(f'发布相机图像失败: {e}')
            self.error_count['camera'] += 1
            
    def _publish_sonar_image(self, timestamp: float, filename: str):
        """发布声呐图像 - 修复版本"""
        if not filename:
            return
            
        try:
            # 读取图像
            image_path = os.path.join(self.dataset_path, 'sonar', filename)
            if not os.path.exists(image_path):
                self.get_logger().warning(f'声呐文件不存在: {image_path}')
                return
                
            # 尝试不同的读取方式
            image = cv2.imread(image_path, cv2.IMREAD_ANYDEPTH)
            if image is None:
                # 如果ANYDEPTH失败，尝试普通读取
                image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
                if image is None:
                    self.get_logger().warning(f'无法读取声呐图像: {image_path}')
                    return
                    
            # 创建消息
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = 'sonar_link'
            
            if self.publish_compressed:
                # 压缩图像 - 确保是8位格式
                if image.dtype != np.uint8:
                    image_norm = cv2.normalize(image, None, 0, 255, cv2.NORM_MINMAX)
                    image_8bit = image_norm.astype(np.uint8)
                else:
                    image_8bit = image
                    
                _, compressed = cv2.imencode('.png', image_8bit)
                msg = CompressedImage()
                msg.header = header
                msg.format = 'png'
                msg.data = compressed.tobytes()
            else:
                # 原始图像 - 根据实际数据类型选择编码
                if image.dtype == np.uint8:
                    # 8位图像 - 最常见的PNG/JPG格式
                    if len(image.shape) == 3:
                        # 彩色图像
                        msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
                    else:
                        # 灰度图像
                        msg = self.bridge.cv2_to_imgmsg(image, encoding='mono8')
                elif image.dtype == np.uint16:
                    # 16位图像
                    msg = self.bridge.cv2_to_imgmsg(image, encoding='16UC1')
                elif image.dtype == np.float32:
                    # 32位浮点图像
                    msg = self.bridge.cv2_to_imgmsg(image, encoding='32FC1')
                elif image.dtype == np.float64:
                    # 64位浮点图像，转换为32位
                    image_32f = image.astype(np.float32)
                    msg = self.bridge.cv2_to_imgmsg(image_32f, encoding='32FC1')
                else:
                    # 其他格式，归一化为8位
                    self.get_logger().warning(f'未知图像格式 {image.dtype}，转换为8位')
                    image_norm = cv2.normalize(image, None, 0, 255, cv2.NORM_MINMAX)
                    image_8bit = image_norm.astype(np.uint8)
                    msg = self.bridge.cv2_to_imgmsg(image_8bit, encoding='mono8')
                    
                msg.header = header
                
            self.sonar_pub.publish(msg)
            self.published_count['sonar'] += 1
            
            # 定期报告成功
            if self.published_count['sonar'] % 100 == 1:
                self.get_logger().info(f'已发布 {self.published_count["sonar"]} 张声呐图像')
            
        except Exception as e:
            self.get_logger().error(f'发布声呐图像失败: {e}')
            self.error_count['sonar'] += 1
            
    def _publish_navigation(self, timestamp: float, nav_data):
        """发布导航数据"""
        try:
            # 创建里程计消息
            odom_msg = PoseWithCovarianceStamped()
            odom_msg.header.stamp = self.get_clock().now().to_msg()
            odom_msg.header.frame_id = 'odom'
            
            # 检查可用的位置列
            pos_columns = ['x', 'y', 'z', 'latitude', 'longitude', 'depth']
            available_pos = {col: col for col in pos_columns if col in nav_data}
            
            if 'x' in available_pos and 'y' in available_pos:
                # 直接使用x, y坐标
                odom_msg.pose.pose.position.x = float(nav_data['x'])
                odom_msg.pose.pose.position.y = float(nav_data['y'])
                odom_msg.pose.pose.position.z = float(nav_data.get('z', 0.0))
            elif 'latitude' in available_pos and 'longitude' in available_pos:
                # 转换经纬度到局部坐标（简化版本）
                if not hasattr(self, 'origin_lat'):
                    self.origin_lat = float(nav_data['latitude'])
                    self.origin_lon = float(nav_data['longitude'])
                    
                # 简化的经纬度转换（适用于小范围）
                R_earth = 6371000  # 地球半径（米）
                lat_rad = np.radians(float(nav_data['latitude']))
                origin_lat_rad = np.radians(self.origin_lat)
                
                x = R_earth * np.radians(float(nav_data['longitude']) - self.origin_lon) * np.cos(origin_lat_rad)
                y = R_earth * np.radians(float(nav_data['latitude']) - self.origin_lat)
                z = -float(nav_data.get('depth', 0.0))  # 深度为负值
                
                odom_msg.pose.pose.position.x = x
                odom_msg.pose.pose.position.y = y
                odom_msg.pose.pose.position.z = z
            else:
                self.get_logger().warning('未找到有效的位置数据列')
                return
                
            # 设置方向（如果有的话）
            orientation_columns = ['roll', 'pitch', 'yaw', 'heading']
            if any(col in nav_data for col in orientation_columns):
                roll = float(nav_data.get('roll', 0.0))
                pitch = float(nav_data.get('pitch', 0.0))
                yaw = float(nav_data.get('yaw', nav_data.get('heading', 0.0)))
                
                # 简化的欧拉角到四元数转换
                cy = np.cos(yaw * 0.5)
                sy = np.sin(yaw * 0.5)
                cp = np.cos(pitch * 0.5)
                sp = np.sin(pitch * 0.5)
                cr = np.cos(roll * 0.5)
                sr = np.sin(roll * 0.5)
                
                odom_msg.pose.pose.orientation.w = cr * cp * cy + sr * sp * sy
                odom_msg.pose.pose.orientation.x = sr * cp * cy - cr * sp * sy
                odom_msg.pose.pose.orientation.y = cr * sp * cy + sr * cp * sy
                odom_msg.pose.pose.orientation.z = cr * cp * sy - sr * sp * cy
            else:
                # 默认方向
                odom_msg.pose.pose.orientation.w = 1.0
                
            # 设置协方差（简化版本）
            odom_msg.pose.covariance = [0.1] * 36  # 简化的协方差矩阵
            
            self.odometry_pub.publish(odom_msg)
            self.published_count['navigation'] += 1
            
            # 如果有IMU数据，也发布IMU消息
            imu_columns = ['accel_x', 'accel_y', 'accel_z', 'gyro_x', 'gyro_y', 'gyro_z']
            if any(col in nav_data for col in imu_columns):
                imu_msg = Imu()
                imu_msg.header = odom_msg.header
                imu_msg.header.frame_id = 'imu_link'
                
                # 加速度
                imu_msg.linear_acceleration.x = float(nav_data.get('accel_x', 0.0))
                imu_msg.linear_acceleration.y = float(nav_data.get('accel_y', 0.0))
                imu_msg.linear_acceleration.z = float(nav_data.get('accel_z', 0.0))
                
                # 角速度
                imu_msg.angular_velocity.x = float(nav_data.get('gyro_x', 0.0))
                imu_msg.angular_velocity.y = float(nav_data.get('gyro_y', 0.0))
                imu_msg.angular_velocity.z = float(nav_data.get('gyro_z', 0.0))
                
                # 方向（复用里程计的方向）
                imu_msg.orientation = odom_msg.pose.pose.orientation
                
                # 协方差矩阵
                imu_msg.orientation_covariance = [0.1] * 9
                imu_msg.angular_velocity_covariance = [0.1] * 9
                imu_msg.linear_acceleration_covariance = [0.1] * 9
                
                self.imu_pub.publish(imu_msg)
                
        except Exception as e:
            self.get_logger().error(f'发布导航数据失败: {e}')
            self.error_count['navigation'] += 1


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = DatasetPlayerNode()
        
        # 使用多线程执行器
        from rclpy.executors import MultiThreadedExecutor
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        
        node.get_logger().info('数据集播放器节点开始运行...')
        executor.spin()
        
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'节点错误: {e}')
    finally:
        if 'node' in locals():
            node.stop_playback()
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()