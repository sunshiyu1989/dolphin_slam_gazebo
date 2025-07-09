#!/usr/bin/env python3
"""
图像同步节点 - 确保相机和声呐图像的同步显示
专为水下无人机SLAM设计
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from message_filters import ApproximateTimeSynchronizer, Subscriber
import cv2
import numpy as np
from cv_bridge import CvBridge
import threading
import time
from collections import deque

class ImageSyncNode(Node):
    def __init__(self):
        super().__init__('image_sync_node')
        
        # 参数声明
        self.declare_parameter('sync_tolerance', 0.1)  # 同步容差（秒）
        self.declare_parameter('max_queue_size', 50)
        self.declare_parameter('publish_combined', True)
        self.declare_parameter('publish_side_by_side', True)
        self.declare_parameter('use_compressed', False)
        
        # 获取参数
        self.sync_tolerance = self.get_parameter('sync_tolerance').get_parameter_value().double_value
        self.max_queue_size = self.get_parameter('max_queue_size').get_parameter_value().integer_value
        self.publish_combined = self.get_parameter('publish_combined').get_parameter_value().bool_value
        self.publish_side_by_side = self.get_parameter('publish_side_by_side').get_parameter_value().bool_value
        self.use_compressed = self.get_parameter('use_compressed').get_parameter_value().bool_value
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # QoS配置
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=self.max_queue_size,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # 图像订阅器（使用消息过滤器进行同步）
        if self.use_compressed:
            self.camera_sub = Subscriber(self, CompressedImage, '/dolphin_slam/camera/image_raw/compressed', qos_profile=qos_profile)
            self.sonar_sub = Subscriber(self, CompressedImage, '/dolphin_slam/sonar/image_raw/compressed', qos_profile=qos_profile)
        else:
            self.camera_sub = Subscriber(self, Image, '/dolphin_slam/camera/image_raw', qos_profile=qos_profile)
            self.sonar_sub = Subscriber(self, Image, '/dolphin_slam/sonar/image_raw', qos_profile=qos_profile)
        
        # 时间同步器
        self.sync = ApproximateTimeSynchronizer(
            [self.camera_sub, self.sonar_sub],
            queue_size=self.max_queue_size,
            slop=self.sync_tolerance
        )
        self.sync.registerCallback(self.sync_callback)
        
        # 发布器
        self.sync_pub = self.create_publisher(Image, '/sync/camera_sonar', qos_profile)
        self.side_by_side_pub = self.create_publisher(Image, '/sync/side_by_side', qos_profile)
        self.overlay_pub = self.create_publisher(Image, '/sync/overlay', qos_profile)
        
        # 性能统计
        self.sync_count = 0
        self.last_sync_time = time.time()
        self.sync_stats = deque(maxlen=100)
        
        # 定时器用于发布统计信息
        self.stats_timer = self.create_timer(5.0, self.publish_stats)
        
        self.get_logger().info(f'图像同步节点已启动')
        self.get_logger().info(f'同步容差: {self.sync_tolerance}s, 队列大小: {self.max_queue_size}')
        
    def sync_callback(self, camera_msg, sonar_msg):
        """同步回调函数"""
        try:
            # 转换图像
            if self.use_compressed:
                camera_image = self.compressed_to_cv2(camera_msg)
                sonar_image = self.compressed_to_cv2(sonar_msg)
            else:
                camera_image = self.bridge.imgmsg_to_cv2(camera_msg, 'bgr8')
                sonar_image = self.bridge.imgmsg_to_cv2(sonar_msg, 'bgr8')
            
            # 确保图像格式一致
            if len(sonar_image.shape) == 2:  # 单通道声呐图像
                sonar_image = cv2.cvtColor(sonar_image, cv2.COLOR_GRAY2BGR)
            
            # 时间戳同步检查
            camera_time = camera_msg.header.stamp.sec + camera_msg.header.stamp.nanosec * 1e-9
            sonar_time = sonar_msg.header.stamp.sec + sonar_msg.header.stamp.nanosec * 1e-9
            time_diff = abs(camera_time - sonar_time)
            
            # 创建统一的header
            sync_header = Header()
            sync_header.stamp = self.get_clock().now().to_msg()
            sync_header.frame_id = 'synchronized_sensors'
            
            # 统计信息
            self.sync_count += 1
            current_time = time.time()
            freq = 1.0 / (current_time - self.last_sync_time) if self.last_sync_time > 0 else 0
            self.sync_stats.append({'freq': freq, 'time_diff': time_diff})
            self.last_sync_time = current_time
            
            # 发布组合图像
            if self.publish_combined:
                combined_image = self.create_combined_view(camera_image, sonar_image)
                combined_msg = self.bridge.cv2_to_imgmsg(combined_image, 'bgr8')
                combined_msg.header = sync_header
                self.sync_pub.publish(combined_msg)
            
            # 发布并排显示
            if self.publish_side_by_side:
                side_by_side_image = self.create_side_by_side_view(camera_image, sonar_image, time_diff)
                side_by_side_msg = self.bridge.cv2_to_imgmsg(side_by_side_image, 'bgr8')
                side_by_side_msg.header = sync_header
                self.side_by_side_pub.publish(side_by_side_msg)
            
            # 发布叠加显示（用于特征匹配可视化）
            overlay_image = self.create_overlay_view(camera_image, sonar_image)
            overlay_msg = self.bridge.cv2_to_imgmsg(overlay_image, 'bgr8')
            overlay_msg.header = sync_header
            self.overlay_pub.publish(overlay_msg)
            
            if self.sync_count % 50 == 0:  # 每50帧记录一次
                self.get_logger().info(f'已同步 {self.sync_count} 帧, 时间差: {time_diff:.3f}s, 频率: {freq:.1f}Hz')
                
        except Exception as e:
            self.get_logger().error(f'图像同步处理失败: {e}')
    
    def compressed_to_cv2(self, compressed_msg):
        """压缩图像转换为OpenCV格式"""
        np_arr = np.frombuffer(compressed_msg.data, np.uint8)
        return cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    
    def create_combined_view(self, camera_image, sonar_image):
        """创建组合视图"""
        # 调整图像大小到相同尺寸
        target_height = 400
        cam_h, cam_w = camera_image.shape[:2]
        sonar_h, sonar_w = sonar_image.shape[:2]
        
        # 计算缩放比例
        cam_scale = target_height / cam_h
        sonar_scale = target_height / sonar_h
        
        # 缩放图像
        camera_resized = cv2.resize(camera_image, (int(cam_w * cam_scale), target_height))
        sonar_resized = cv2.resize(sonar_image, (int(sonar_w * sonar_scale), target_height))
        
        # 创建组合图像 (上下排列)
        combined = np.vstack([camera_resized, sonar_resized])
        
        # 添加标签
        cv2.putText(combined, 'Camera', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(combined, 'Sonar', (10, target_height + 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
        
        return combined
    
    def create_side_by_side_view(self, camera_image, sonar_image, time_diff):
        """创建并排视图"""
        # 调整图像大小
        target_height = 300
        cam_h, cam_w = camera_image.shape[:2]
        sonar_h, sonar_w = sonar_image.shape[:2]
        
        cam_scale = target_height / cam_h
        sonar_scale = target_height / sonar_h
        
        camera_resized = cv2.resize(camera_image, (int(cam_w * cam_scale), target_height))
        sonar_resized = cv2.resize(sonar_image, (int(sonar_w * sonar_scale), target_height))
        
        # 并排放置
        side_by_side = np.hstack([camera_resized, sonar_resized])
        
        # 添加同步信息
        sync_color = (0, 255, 0) if time_diff < self.sync_tolerance else (0, 0, 255)
        cv2.putText(side_by_side, f'Sync Diff: {time_diff:.3f}s', 
                   (10, side_by_side.shape[0] - 10), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, sync_color, 2)
        
        # 分割线
        cv2.line(side_by_side, (camera_resized.shape[1], 0), 
                (camera_resized.shape[1], target_height), (255, 255, 255), 2)
        
        return side_by_side
    
    def create_overlay_view(self, camera_image, sonar_image):
        """创建叠加视图（用于特征匹配）"""
        # 将声呐图像转为热图
        if len(sonar_image.shape) == 3:
            sonar_gray = cv2.cvtColor(sonar_image, cv2.COLOR_BGR2GRAY)
        else:
            sonar_gray = sonar_image.copy()
        
        # 应用伪彩色映射
        sonar_colored = cv2.applyColorMap(sonar_gray, cv2.COLORMAP_JET)
        
        # 调整尺寸匹配相机图像
        camera_resized = cv2.resize(camera_image, (640, 480))
        sonar_resized = cv2.resize(sonar_colored, (640, 480))
        
        # 创建半透明叠加
        overlay = cv2.addWeighted(camera_resized, 0.7, sonar_resized, 0.3, 0)
        
        # 添加图例
        cv2.putText(overlay, 'Camera + Sonar Overlay', (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        
        return overlay
    
    def publish_stats(self):
        """发布同步统计信息"""
        if len(self.sync_stats) > 0:
            avg_freq = np.mean([s['freq'] for s in self.sync_stats])
            avg_time_diff = np.mean([s['time_diff'] for s in self.sync_stats])
            max_time_diff = np.max([s['time_diff'] for s in self.sync_stats])
            
            self.get_logger().info(
                f'同步统计 - 平均频率: {avg_freq:.1f}Hz, '
                f'平均时间差: {avg_time_diff:.3f}s, '
                f'最大时间差: {max_time_diff:.3f}s, '
                f'总同步帧数: {self.sync_count}'
            )

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = ImageSyncNode()
        
        # 使用多线程执行器
        from rclpy.executors import MultiThreadedExecutor
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        
        node.get_logger().info('图像同步节点开始运行...')
        executor.spin()
        
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'节点错误: {e}')
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()