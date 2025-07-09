#!/usr/bin/env python3
"""
Dolphin SLAM - 轨迹评估节点
对比 GPS Ground Truth 和 SLAM 估计轨迹
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Float64
import numpy as np
import math
from collections import deque

class TrajectoryEvaluatorNode(Node):
    def __init__(self):
        super().__init__('trajectory_evaluator_node')
        
        # 参数
        self.declare_parameter('max_path_length', 1000)
        self.declare_parameter('evaluation_rate', 1.0)
        self.declare_parameter('origin_lat', 29.5014)  # Red Sea 附近
        self.declare_parameter('origin_lon', 34.9167)
        
        self.max_path_length = self.get_parameter('max_path_length').value
        self.origin_lat = self.get_parameter('origin_lat').value
        self.origin_lon = self.get_parameter('origin_lon').value
        
        # GPS Ground Truth 路径
        self.gps_path = Path()
        self.gps_path.header.frame_id = 'map'
        self.gps_poses = deque(maxlen=self.max_path_length)
        
        # SLAM 估计路径
        self.slam_path = Path()
        self.slam_path.header.frame_id = 'map'
        self.slam_poses = deque(maxlen=self.max_path_length)
        
        # 发布器
        self.gps_path_pub = self.create_publisher(Path, '/evaluation/gps_path', 10)
        self.slam_path_pub = self.create_publisher(Path, '/evaluation/slam_path', 10)
        self.error_pub = self.create_publisher(Float64, '/evaluation/position_error', 10)
        self.rmse_pub = self.create_publisher(Float64, '/evaluation/rmse', 10)
        
        # 订阅器
        self.gps_sub = self.create_subscription(
            NavSatFix, '/gps/fix', self.gps_callback, 10)
        self.slam_sub = self.create_subscription(
            Odometry, '/robot/odometry', self.slam_callback, 10)
        
        # 定时器用于发布路径和计算误差
        self.timer = self.create_timer(
            1.0 / self.get_parameter('evaluation_rate').value,
            self.evaluation_callback)
        
        # 误差计算
        self.position_errors = deque(maxlen=self.max_path_length)
        
        self.get_logger().info('轨迹评估节点已启动')
        self.get_logger().info(f'GPS 原点: ({self.origin_lat:.4f}, {self.origin_lon:.4f})')

    def gps_to_local(self, lat, lon):
        """将GPS坐标转换为局部笛卡尔坐标"""
        # 简化的平面投影（对于小范围区域足够精确）
        R = 6371000.0  # 地球半径（米）
        
        # 转换为弧度
        lat_rad = math.radians(lat)
        lon_rad = math.radians(lon)
        origin_lat_rad = math.radians(self.origin_lat)
        origin_lon_rad = math.radians(self.origin_lon)
        
        # 计算局部坐标
        x = R * (lon_rad - origin_lon_rad) * math.cos(origin_lat_rad)
        y = R * (lat_rad - origin_lat_rad)
        
        return x, y

    def gps_callback(self, msg):
        """处理GPS数据"""
        if msg.status.status < 0:  # GPS 无效
            return
            
        # 转换为局部坐标
        x, y = self.gps_to_local(msg.latitude, msg.longitude)
        
        # 创建位姿
        pose = PoseStamped()
        pose.header = msg.header
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = msg.altitude if not math.isnan(msg.altitude) else 0.0
        
        # 保持朝向（没有GPS偏航，使用单位四元数）
        pose.pose.orientation.w = 1.0
        
        # 添加到路径
        self.gps_poses.append(pose)
        
        # 更新路径消息
        self.gps_path.header.stamp = self.get_clock().now().to_msg()
        self.gps_path.poses = list(self.gps_poses)

    def slam_callback(self, msg):
        """处理SLAM里程计数据"""
        # 创建位姿
        pose = PoseStamped()
        pose.header = msg.header
        pose.header.frame_id = 'map'
        pose.pose = msg.pose.pose
        
        # 添加到路径
        self.slam_poses.append(pose)
        
        # 更新路径消息
        self.slam_path.header.stamp = self.get_clock().now().to_msg()
        self.slam_path.poses = list(self.slam_poses)

    def calculate_position_error(self):
        """计算当前位置误差"""
        if len(self.gps_poses) == 0 or len(self.slam_poses) == 0:
            return None
            
        # 获取最新位置
        gps_pos = self.gps_poses[-1].pose.position
        slam_pos = self.slam_poses[-1].pose.position
        
        # 计算欧式距离
        dx = gps_pos.x - slam_pos.x
        dy = gps_pos.y - slam_pos.y
        dz = gps_pos.z - slam_pos.z
        
        error = math.sqrt(dx*dx + dy*dy + dz*dz)
        return error

    def calculate_rmse(self):
        """计算总体RMSE"""
        if len(self.position_errors) == 0:
            return None
            
        errors_array = np.array(self.position_errors)
        rmse = np.sqrt(np.mean(errors_array**2))
        return rmse

    def evaluation_callback(self):
        """定期评估回调"""
        # 发布路径
        if len(self.gps_poses) > 0:
            self.gps_path_pub.publish(self.gps_path)
            
        if len(self.slam_poses) > 0:
            self.slam_path_pub.publish(self.slam_path)
        
        # 计算并发布误差
        current_error = self.calculate_position_error()
        if current_error is not None:
            # 发布当前误差
            error_msg = Float64()
            error_msg.data = current_error
            self.error_pub.publish(error_msg)
            
            # 保存误差用于RMSE计算
            self.position_errors.append(current_error)
            
            # 计算并发布RMSE
            rmse = self.calculate_rmse()
            if rmse is not None:
                rmse_msg = Float64()
                rmse_msg.data = rmse
                self.rmse_pub.publish(rmse_msg)
                
            # 日志信息
            if len(self.position_errors) % 50 == 0:  # 每50次输出一次
                self.get_logger().info(
                    f'位置误差: {current_error:.3f}m, RMSE: {rmse:.3f}m, '
                    f'GPS点数: {len(self.gps_poses)}, SLAM点数: {len(self.slam_poses)}'
                )

def main(args=None):
    rclpy.init(args=args)
    
    node = TrajectoryEvaluatorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('轨迹评估节点正在关闭...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
