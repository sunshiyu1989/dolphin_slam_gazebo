#!/usr/bin/env python3
"""
统一数据播放控制器 - 解决数据同步和结束检测问题
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import pandas as pd
import numpy as np
import os
import time
import threading
from threading import Lock

# ROS2 消息类型
from std_msgs.msg import Bool, Header
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from builtin_interfaces.msg import Time
from rosgraph_msgs.msg import Clock
import cv2
from cv_bridge import CvBridge

class UnifiedDataController(Node):
    """
    统一数据播放控制器
    
    功能:
    1. 统一控制所有数据源的播放
    2. 确保时间戳严格同步
    3. 检测播放结束并发送停止信号
    4. 支持暂停/恢复/停止控制
    """
    
    def __init__(self):
        super().__init__('unified_data_controller')
        
        # 参数声明
        self.declare_parameter('dataset_path', '')
        self.declare_parameter('playback_speed', 1.0)
        self.declare_parameter('start_time_offset', 0.0)  # 从数据开始后多少秒开始播放
        self.declare_parameter('end_time_offset', 0.0)    # 在数据结束前多少秒停止
        self.declare_parameter('sync_tolerance', 0.1)     # 时间同步容差（秒）
        
        # 获取参数
        self.dataset_path = self.get_parameter('dataset_path').value
        self.playback_speed = self.get_parameter('playback_speed').value
        self.start_time_offset = self.get_parameter('start_time_offset').value
        self.end_time_offset = self.get_parameter('end_time_offset').value
        self.sync_tolerance = self.get_parameter('sync_tolerance').value
        
        # 发布器
        self.camera_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.sonar_pub = self.create_publisher(Image, '/sonar/image_raw', 10)
        self.odom_pub = self.create_publisher(Odometry, '/dolphin_slam/odometry', 10)

        # 时钟发布器（仿真时间支持）
        self.clock_pub = self.create_publisher(Clock, '/clock', 10)
        
        # 系统控制信号
        self.playback_status_pub = self.create_publisher(Bool, '/data_controller/playing', 10)
        self.playback_finished_pub = self.create_publisher(Bool, '/data_controller/finished', 10)
        
        # 控制订阅器
        self.create_subscription(Bool, '/data_controller/pause', self.pause_callback, 10)
        self.create_subscription(Bool, '/data_controller/stop', self.stop_callback, 10)
        
        # 状态变量
        self.is_playing = False
        self.is_paused = False
        self.should_stop = False
        self.lock = Lock()
        
        # 数据存储
        self.navigation_data = None
        self.camera_data = None
        self.sonar_data = None
        self.synchronized_data = []
        
        # 播放控制
        self.current_index = 0
        self.playback_thread = None
        self.start_real_time = None
        self.start_data_time = None
        
        # 工具
        self.bridge = CvBridge()
        
        # 统计信息
        self.stats = {
            'total_frames': 0,
            'published_frames': 0,
            'sync_errors': 0,
            'missing_files': 0
        }
        
        # 加载和同步数据
        self.load_and_synchronize_data()
        
        # 开始播放
        self.start_playback()
        
    def load_and_synchronize_data(self):
        """加载并同步所有数据源"""
        self.get_logger().info(f'🔄 加载数据集: {self.dataset_path}')
        
        try:
            # 加载CSV文件
            nav_path = os.path.join(self.dataset_path, 'navigation.csv')
            camera_path = os.path.join(self.dataset_path, 'camera.csv')
            sonar_path = os.path.join(self.dataset_path, 'sonar.csv')
            
            self.navigation_data = pd.read_csv(nav_path)
            self.camera_data = pd.read_csv(camera_path)
            self.sonar_data = pd.read_csv(sonar_path)
            
            self.get_logger().info(f'✅ 加载完成: nav={len(self.navigation_data)}, '
                                 f'cam={len(self.camera_data)}, sonar={len(self.sonar_data)}')
            
            # 验证导航数据列
            required_nav_columns = ['timestamp', 'latitude', 'longitude', 'depth', 'yaw', 'pitch', 'roll']
            missing_columns = [col for col in required_nav_columns if col not in self.navigation_data.columns]
            if missing_columns:
                raise ValueError(f'导航数据缺少必要列: {missing_columns}')
            
            # 转换经纬度到局部坐标
            self._convert_navigation_coordinates()
            
            # 找出共同时间范围
            min_time = max(
                self.navigation_data['timestamp'].min(),
                self.camera_data['timestamp'].min(),
                self.sonar_data['timestamp'].min()
            ) + self.start_time_offset
            
            max_time = min(
                self.navigation_data['timestamp'].max(),
                self.camera_data['timestamp'].max(),
                self.sonar_data['timestamp'].max()
            ) - self.end_time_offset
            
            self.get_logger().info(f'📊 同步时间范围: {min_time:.2f} - {max_time:.2f} '
                                 f'({max_time - min_time:.1f}秒)')
            
            # 创建同步的数据列表
            self._create_synchronized_timeline(min_time, max_time)
            
        except Exception as e:
            self.get_logger().error(f'❌ 数据加载失败: {e}')
            raise
            
    def _convert_navigation_coordinates(self):
        """将导航数据的经纬度转换为局部笛卡尔坐标"""
        try:
            self.get_logger().info('🗺️ 转换经纬度到局部坐标...')
            
            # 使用第一个点作为原点
            origin_lat = float(self.navigation_data['latitude'].iloc[0])
            origin_lon = float(self.navigation_data['longitude'].iloc[0])
            
            self.get_logger().info(f'坐标转换原点: lat={origin_lat}, lon={origin_lon}')
            
            # 地球半径（米）
            R_earth = 6371000
            
            # 转换为弧度
            lat_rad = np.radians(self.navigation_data['latitude'].astype(float))
            lon_rad = np.radians(self.navigation_data['longitude'].astype(float))
            origin_lat_rad = np.radians(origin_lat)
            origin_lon_rad = np.radians(origin_lon)
            
            # 计算局部坐标 (简化的墨卡托投影)
            self.navigation_data['x'] = R_earth * (lon_rad - origin_lon_rad) * np.cos(origin_lat_rad)
            self.navigation_data['y'] = R_earth * (lat_rad - origin_lat_rad)
            self.navigation_data['z'] = -self.navigation_data['depth'].astype(float)  # 深度为负z
            
            # 确保角度数据为浮点型
            self.navigation_data['yaw'] = self.navigation_data['yaw'].astype(float)
            self.navigation_data['pitch'] = self.navigation_data['pitch'].astype(float)
            self.navigation_data['roll'] = self.navigation_data['roll'].astype(float)
            
            # 如果有速度数据，也转换类型
            for vel_col in ['velocity_x', 'velocity_y', 'velocity_z']:
                if vel_col in self.navigation_data.columns:
                    self.navigation_data[vel_col] = self.navigation_data[vel_col].astype(float)
            
            self.get_logger().info('✅ 坐标转换完成')
            
            # 打印坐标范围验证
            x_range = self.navigation_data['x'].max() - self.navigation_data['x'].min()
            y_range = self.navigation_data['y'].max() - self.navigation_data['y'].min()
            z_range = self.navigation_data['z'].max() - self.navigation_data['z'].min()
            
            self.get_logger().info(f'轨迹范围: X={x_range:.1f}m, Y={y_range:.1f}m, Z={z_range:.1f}m')
            
        except Exception as e:
            self.get_logger().error(f'❌ 坐标转换失败: {e}')
            raise
            
    def _create_synchronized_timeline(self, start_time, end_time):
        """创建严格同步的时间线"""
        self.get_logger().info('🔗 创建同步时间线...')
        
        # 以导航数据的时间戳为基准（最稳定）
        nav_subset = self.navigation_data[
            (self.navigation_data['timestamp'] >= start_time) &
            (self.navigation_data['timestamp'] <= end_time)
        ].copy()
        
        sync_count = 0
        for _, nav_row in nav_subset.iterrows():
            timestamp = nav_row['timestamp']
            
            # 找到最接近的相机和声呐数据
            camera_row = self._find_closest_data(self.camera_data, timestamp)
            sonar_row = self._find_closest_data(self.sonar_data, timestamp)
            
            # 检查时间同步容差
            if (camera_row is not None and 
                sonar_row is not None and
                abs(camera_row['timestamp'] - timestamp) <= self.sync_tolerance and
                abs(sonar_row['timestamp'] - timestamp) <= self.sync_tolerance):
                
                self.synchronized_data.append({
                    'timestamp': timestamp,
                    'navigation': nav_row,
                    'camera': camera_row,
                    'sonar': sonar_row
                })
                sync_count += 1
            else:
                self.stats['sync_errors'] += 1
        
        self.stats['total_frames'] = len(self.synchronized_data)
        self.get_logger().info(f'✅ 同步完成: {sync_count} 帧数据，'
                             f'{self.stats["sync_errors"]} 个同步错误')
        
    def _find_closest_data(self, dataframe, target_timestamp):
        """找到最接近目标时间戳的数据行"""
        time_diffs = np.abs(dataframe['timestamp'] - target_timestamp)
        closest_idx = time_diffs.idxmin()
        
        if time_diffs.iloc[closest_idx] <= self.sync_tolerance:
            return dataframe.iloc[closest_idx]
        return None
        
    def start_playback(self):
        """开始播放"""
        with self.lock:
            if not self.is_playing and len(self.synchronized_data) > 0:
                self.is_playing = True
                self.should_stop = False
                self.current_index = 0
                
                self.playback_thread = threading.Thread(target=self._playback_loop)
                self.playback_thread.start()
                
                self.get_logger().info(f'🚀 开始播放 {len(self.synchronized_data)} 帧数据')
                
                # 发布播放状态
                status_msg = Bool()
                status_msg.data = True
                self.playback_status_pub.publish(status_msg)
                
    def _playback_loop(self):
        """主播放循环"""
        self.start_real_time = time.time()
        self.start_data_time = self.synchronized_data[0]['timestamp']
        
        while self.is_playing and not self.should_stop and self.current_index < len(self.synchronized_data):
            try:
                # 暂停检查
                while self.is_paused and not self.should_stop:
                    time.sleep(0.1)
                    
                if self.should_stop:
                    break
                    
                # 获取当前数据帧
                data_frame = self.synchronized_data[self.current_index]
                current_data_time = data_frame['timestamp']
                
                # 计算应该等待的时间
                elapsed_real_time = time.time() - self.start_real_time
                elapsed_data_time = (current_data_time - self.start_data_time) / self.playback_speed
                
                wait_time = elapsed_data_time - elapsed_real_time
                if wait_time > 0:
                    time.sleep(wait_time)
                
                # 发布数据
                self._publish_frame(data_frame)
                
                self.current_index += 1
                self.stats['published_frames'] += 1
                
                # 进度报告
                if self.current_index % 100 == 0:
                    progress = self.current_index / len(self.synchronized_data) * 100
                    self.get_logger().info(f'📈 播放进度: {progress:.1f}% '
                                         f'({self.current_index}/{len(self.synchronized_data)})')
                
            except Exception as e:
                self.get_logger().error(f'❌ 播放错误: {e}')
                break
        
        # 播放结束处理
        self._finish_playback()
        
    def _publish_frame(self, data_frame):
        """发布一帧数据 - 修复版：使用数据集的时间戳创建相对时间"""
        dataset_timestamp = data_frame['timestamp']
        
        # 🔧 修复：创建基于数据集时间戳的相对ROS时间
        # 计算相对于开始时间的偏移
        time_offset_seconds = dataset_timestamp - self.start_data_time
        
        # 创建ROS时间戳：开始时的ROS时间 + 数据集时间偏移
        if not hasattr(self, 'ros_start_time'):
            # 记录播放开始时的ROS时间
            self.ros_start_time = self.get_clock().now()
        
        # 计算当前应该的ROS时间
        current_ros_time_ns = self.ros_start_time.nanoseconds + int(time_offset_seconds * 1e9)
        
        # 创建ROS时间戳消息
        ros_time = Time()
        ros_time.sec = int(current_ros_time_ns // 1e9)
        ros_time.nanosec = int(current_ros_time_ns % 1e9)
        
        # 🔧 调试信息：显示时间戳转换
        if self.current_index % 500 == 0:  # 每500帧显示一次
            self.get_logger().info(
                f'🕐 时间戳转换: 数据集={dataset_timestamp:.3f}, '
                f'偏移={time_offset_seconds:.3f}s, '
                f'ROS时间={ros_time.sec}.{ros_time.nanosec:09d}'
            )
        
        try:
            # 发布相机图像
            if self._publish_image(data_frame['camera'], self.camera_pub, 'camera', ros_time):
                pass
            else:
                self.stats['missing_files'] += 1
                
            # 发布声呐图像
            if self._publish_image(data_frame['sonar'], self.sonar_pub, 'sonar', ros_time):
                pass
            else:
                self.stats['missing_files'] += 1
                
            # 发布导航数据
            self._publish_navigation(data_frame['navigation'], ros_time)
            
            # 🔧 如果启用仿真时间，发布时钟信号
            if self.get_parameter('use_sim_time').value:
                self._publish_clock_signal(ros_time)
            
        except Exception as e:
            self.get_logger().error(f'❌ 发布数据失败: {e}')

    def _publish_clock_signal(self, ros_time):
        """发布时钟信号以支持仿真时间"""
        if not hasattr(self, 'clock_pub'):
            from rosgraph_msgs.msg import Clock
            self.clock_pub = self.create_publisher(Clock, '/clock', 10)
        
        from rosgraph_msgs.msg import Clock
        clock_msg = Clock()
        clock_msg.clock = ros_time
        self.clock_pub.publish(clock_msg)
            
    def _publish_image(self, image_row, publisher, image_type, ros_time):
        """发布图像数据 - 修复版：使用传入的时间戳"""
        try:
            filename = image_row['filename']
            image_path = os.path.join(self.dataset_path, image_type, filename)
            
            if not os.path.exists(image_path):
                return False
                
            # 读取图像
            cv_image = cv2.imread(image_path, cv2.IMREAD_COLOR)
            if cv_image is None:
                return False
                
            # 转换为ROS消息
            ros_image = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            ros_image.header.stamp = ros_time  # 🔧 使用修复后的时间戳
            ros_image.header.frame_id = f'{image_type}_link'
            
            publisher.publish(ros_image)
            return True
            
        except Exception as e:
            self.get_logger().error(f'❌ 图像发布失败 ({image_type}): {e}')
            return False
            
    def _publish_navigation(self, nav_row, ros_time):
        """发布导航数据 - 修复版：使用传入的时间戳"""
        try:
            odom_msg = Odometry()
            odom_msg.header.stamp = ros_time  # 🔧 使用修复后的时间戳
            odom_msg.header.frame_id = 'odom'
            odom_msg.child_frame_id = 'base_link'
            
            # 位置 (使用转换后的局部坐标)
            odom_msg.pose.pose.position.x = float(nav_row['x'])
            odom_msg.pose.pose.position.y = float(nav_row['y'])
            odom_msg.pose.pose.position.z = float(nav_row['z'])
            
            # 转换角度到四元数
            yaw = np.radians(float(nav_row['yaw']))
            pitch = np.radians(float(nav_row['pitch']))
            roll = np.radians(float(nav_row['roll']))
            
            # 简化的欧拉角到四元数转换 (ZYX顺序)
            cy = np.cos(yaw * 0.5)
            sy = np.sin(yaw * 0.5)
            cp = np.cos(pitch * 0.5)
            sp = np.sin(pitch * 0.5)
            cr = np.cos(roll * 0.5)
            sr = np.sin(roll * 0.5)

            qw = cr * cp * cy + sr * sp * sy
            qx = sr * cp * cy - cr * sp * sy
            qy = cr * sp * cy + sr * cp * sy
            qz = cr * cp * sy - sr * sp * cy
            
            odom_msg.pose.pose.orientation.x = qx
            odom_msg.pose.pose.orientation.y = qy
            odom_msg.pose.pose.orientation.z = qz
            odom_msg.pose.pose.orientation.w = qw
            
            # 速度 (如果有的话)
            if all(col in nav_row for col in ['velocity_x', 'velocity_y', 'velocity_z']):
                odom_msg.twist.twist.linear.x = float(nav_row['velocity_x'])
                odom_msg.twist.twist.linear.y = float(nav_row['velocity_y'])
                odom_msg.twist.twist.linear.z = float(nav_row['velocity_z'])
            
            # 设置协方差 (简化)
            odom_msg.pose.covariance[0] = 0.1   # x
            odom_msg.pose.covariance[7] = 0.1   # y
            odom_msg.pose.covariance[14] = 0.1  # z
            odom_msg.pose.covariance[21] = 0.05 # roll
            odom_msg.pose.covariance[28] = 0.05 # pitch
            odom_msg.pose.covariance[35] = 0.05 # yaw
            
            self.odom_pub.publish(odom_msg)
            
        except Exception as e:
            self.get_logger().error(f'❌ 导航数据发布失败: {e}')

            
    def _finish_playback(self):
        """完成播放"""
        with self.lock:
            self.is_playing = False
            
        # 发布完成信号
        finished_msg = Bool()
        finished_msg.data = True
        self.playback_finished_pub.publish(finished_msg)
        
        status_msg = Bool()
        status_msg.data = False
        self.playback_status_pub.publish(status_msg)
        
        # 打印统计信息
        self.get_logger().info('🏁 播放完成!')
        self.get_logger().info(f'📊 统计信息:')
        self.get_logger().info(f'   总帧数: {self.stats["total_frames"]}')
        self.get_logger().info(f'   已发布: {self.stats["published_frames"]}')
        self.get_logger().info(f'   同步错误: {self.stats["sync_errors"]}')
        self.get_logger().info(f'   缺失文件: {self.stats["missing_files"]}')
        
        completion_rate = self.stats["published_frames"] / self.stats["total_frames"] * 100
        self.get_logger().info(f'   完成率: {completion_rate:.1f}%')
        
    def pause_callback(self, msg):
        """暂停/恢复回调"""
        with self.lock:
            self.is_paused = msg.data
            status = "暂停" if msg.data else "恢复"
            self.get_logger().info(f'⏸️ 播放{status}')
            
    def stop_callback(self, msg):
        """停止回调"""
        if msg.data:
            with self.lock:
                self.should_stop = True
            self.get_logger().info('⏹️ 收到停止信号')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        controller = UnifiedDataController()
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"错误: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()