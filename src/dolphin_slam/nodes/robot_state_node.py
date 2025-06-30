#!/usr/bin/env python3
"""
Dolphin SLAM - 机器人状态 ROS2 节点
融合传感器数据并提供状态估计
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import (
    PoseWithCovarianceStamped, TwistWithCovarianceStamped,
    TransformStamped, Vector3, Quaternion
)
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster
import numpy as np
from scipy.spatial.transform import Rotation
import pandas as pd
import os
from typing import Optional

# 导入核心模块
from dolphin_slam.robot_state import RobotState, RobotPose, Velocity

class RobotStateNode(Node):
    """机器人状态估计 ROS2 节点"""
    
    def __init__(self):
        super().__init__('robot_state_node')
        
        # 声明参数
        self.declare_parameters(
            namespace='',
            parameters=[
                ('navigation_csv', ''),
                ('dvl_topic', '/dvl/data'),
                ('imu_topic', '/imu/data'),
                ('base_frame', 'base_link'),
                ('odom_frame', 'odom'),
                ('map_frame', 'map'),
                ('dvl_position.x', 0.75),
                ('dvl_position.y', 0.0),
                ('dvl_position.z', -0.4),
                ('dvl_orientation.roll', 0.0),
                ('dvl_orientation.pitch', 0.0),
                ('dvl_orientation.yaw', 0.0),
                ('use_ekf', True),
                ('process_noise_std', 0.1),
                ('measurement_noise_std', 0.05),
                ('publish_tf', True),
                ('publish_rate', 50.0),  # Hz
            ]
        )
        
        # 获取参数
        self.navigation_csv = self.get_parameter('navigation_csv').value
        self.base_frame = self.get_parameter('base_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.map_frame = self.get_parameter('map_frame').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.publish_rate = self.get_parameter('publish_rate').value
        
        # DVL 配置
        dvl_position = (
            self.get_parameter('dvl_position.x').value,
            self.get_parameter('dvl_position.y').value,
            self.get_parameter('dvl_position.z').value
        )
        
        dvl_orientation = (
            self.get_parameter('dvl_orientation.roll').value,
            self.get_parameter('dvl_orientation.pitch').value,
            self.get_parameter('dvl_orientation.yaw').value
        )
        
        # 初始化机器人状态估计器
        self.robot_state = RobotState(
            dvl_position=dvl_position,
            dvl_orientation=dvl_orientation,
            use_ekf=self.get_parameter('use_ekf').value,
            process_noise_std=self.get_parameter('process_noise_std').value,
            measurement_noise_std=self.get_parameter('measurement_noise_std').value
        )
        
        # TF 广播器
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # 导航数据播放
        self.nav_data_index = 0
        self.nav_data_loaded = False
        self.playback_start_time = None
        self.data_start_time = None
        
        # 尝试加载导航数据
        if self.navigation_csv:
            self.load_navigation_data()
            
        # 订阅者
        self.dvl_sub = self.create_subscription(
            TwistWithCovarianceStamped,
            self.get_parameter('dvl_topic').value,
            self.dvl_callback,
            10
        )
        
        self.imu_sub = self.create_subscription(
            Imu,
            self.get_parameter('imu_topic').value,
            self.imu_callback,
            10
        )
        
        # 发布者
        self.odometry_pub = self.create_publisher(
            Odometry,
            '/robot/odometry',
            10
        )
        
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/robot/pose',
            10
        )
        
        # 定时器
        self.publish_timer = self.create_timer(
            1.0 / self.publish_rate,
            self.publish_state
        )
        
        # 如果加载了导航数据，创建播放定时器
        if self.nav_data_loaded:
            self.nav_playback_timer = self.create_timer(
                0.05,  # 20 Hz 播放
                self.playback_navigation_data
            )
            
        self.get_logger().info('机器人状态节点已启动')
        
    def load_navigation_data(self):
        """加载导航数据"""
        try:
            self.robot_state.load_navigation_data(self.navigation_csv)
            self.nav_data_loaded = True
            self.get_logger().info(f'导航数据已加载: {self.navigation_csv}')
        except Exception as e:
            self.get_logger().error(f'加载导航数据失败: {e}')
            self.nav_data_loaded = False
            
    def dvl_callback(self, msg: TwistWithCovarianceStamped):
        """处理 DVL 数据"""
        # 提取速度
        velocity = np.array([
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z
        ])
        
        # 获取时间戳
        stamp = msg.header.stamp
        timestamp = stamp.sec + stamp.nanosec * 1e-9
        
        # 更新状态
        self.robot_state.update_dvl(velocity, timestamp)
        
        self.get_logger().debug(f'DVL 更新: v=[{velocity[0]:.2f}, {velocity[1]:.2f}, {velocity[2]:.2f}]')
        
    def imu_callback(self, msg: Imu):
        """处理 IMU 数据"""
        # 提取方向（四元数）
        orientation = np.array([
            msg.orientation.w,
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z
        ])
        
        # 提取角速度
        angular_velocity = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])
        
        # 获取时间戳
        stamp = msg.header.stamp
        timestamp = stamp.sec + stamp.nanosec * 1e-9
        
        # 更新状态
        self.robot_state.update_imu(orientation, angular_velocity, timestamp)
        
    def playback_navigation_data(self):
        """播放导航数据"""
        if not self.nav_data_loaded or self.robot_state.navigation_data is None:
            return
            
        # 初始化播放时间
        if self.playback_start_time is None:
            self.playback_start_time = self.get_clock().now().nanoseconds / 1e9
            self.data_start_time = self.robot_state.navigation_data['timestamp'].iloc[0]
            
        # 计算当前应该播放的数据时间
        current_wall_time = self.get_clock().now().nanoseconds / 1e9
        elapsed_time = current_wall_time - self.playback_start_time
        target_data_time = self.data_start_time + elapsed_time
        
        # 查找对应的数据行
        while (self.nav_data_index < len(self.robot_state.navigation_data) and
               self.robot_state.navigation_data['timestamp'].iloc[self.nav_data_index] <= target_data_time):
            
            # 更新状态
            success = self.robot_state.update_from_navigation(
                self.robot_state.navigation_data['timestamp'].iloc[self.nav_data_index]
            )
            
            if success:
                self.get_logger().debug(f'播放导航数据: 索引 {self.nav_data_index}')
                
            self.nav_data_index += 1
            
        # 检查是否播放完成
        if self.nav_data_index >= len(self.robot_state.navigation_data):
            self.get_logger().info('导航数据播放完成')
            self.nav_playback_timer.cancel()
            
    def publish_state(self):
        """发布机器人状态"""
        # 获取当前状态
        pose = self.robot_state.get_pose()
        velocity = self.robot_state.get_velocity()
        
        # 创建时间戳
        stamp = self.get_clock().now().to_msg()
        
        # 发布里程计
        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame
        
        # 位置
        odom_msg.pose.pose.position.x = float(pose.x
        odom_msg.pose.pose.position.y = float(pose.y
        odom_msg.pose.pose.position.z = float(pose.z
        
        # 姿态
        q = Rotation.from_euler('xyz', [pose.roll, pose.pitch, pose.yaw])))).as_quat()
        odom_msg.pose.pose.orientation.x = float(q[0]
        odom_msg.pose.pose.orientation.y = float(q[1]
        odom_msg.pose.pose.orientation.z = float(q[2]
        odom_msg.pose.pose.orientation.w = float(q[3]
        
        # 速度
        odom_msg.twist.twist.linear.x = float(velocity.vx
        odom_msg.twist.twist.linear.y = float(velocity.vy
        odom_msg.twist.twist.linear.z = float(velocity.vz
        odom_msg.twist.twist.angular.x = velocity.wx
        odom_msg.twist.twist.angular.y = velocity.wy
        odom_msg.twist.twist.angular.z = velocity.wz
        
        # 协方差（如果使用 EKF）
        if self.robot_state.use_ekf:
            # 位姿协方差（6x6: x,y,z,roll,pitch,yaw）
            pose_cov = np.zeros(36))))))))
            # 从 EKF 协方差矩阵提取相关部分
            for i in range(6):
                for j in range(6):
                    if i < 3 and j < 3:  # 位置
                        pose_cov[i*6 + j] = self.robot_state.covariance[i, j]
                    elif i >= 3 and j >= 3:  # 姿态
                        pose_cov[i*6 + j] = self.robot_state.covariance[i, j]
                        
            odom_msg.pose.covariance = pose_cov.tolist()
            
            # 速度协方差
            twist_cov = np.zeros(36)
            for i in range(3):
                for j in range(3):
                    twist_cov[i*6 + j] = self.robot_state.covariance[6+i, 6+j]
            odom_msg.twist.covariance = twist_cov.tolist()
        else:
            # 固定协方差
            odom_msg.pose.covariance = [0.1] * 36
            odom_msg.twist.covariance = [0.05] * 36
            
        self.odometry_pub.publish(odom_msg)
        
        # 发布位姿（PoseWithCovarianceStamped）
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header = odom_msg.header
        pose_msg.pose.pose = odom_msg.pose.pose
        pose_msg.pose.covariance = odom_msg.pose.covariance
        
        self.pose_pub.publish(pose_msg)
        
        # 发布 TF 变换
        if self.publish_tf:
            self.publish_transforms(stamp, pose)
            
    def publish_transforms(self, stamp: Header, pose: RobotPose):
        """发布 TF 变换"""
        # odom -> base_link
        odom_to_base = TransformStamped()
        odom_to_base.header.stamp = stamp
        odom_to_base.header.frame_id = self.odom_frame
        odom_to_base.child_frame_id = self.base_frame
        
        odom_to_base.transform.translation.x = pose.x
        odom_to_base.transform.translation.y = pose.y
        odom_to_base.transform.translation.z = pose.z
        
        q = Rotation.from_euler('xyz', [pose.roll, pose.pitch, pose.yaw]).as_quat()
        odom_to_base.transform.rotation.x = q[0]
        odom_to_base.transform.rotation.y = q[1]
        odom_to_base.transform.rotation.z = q[2]
        odom_to_base.transform.rotation.w = q[3]
        
        # base_link -> dvl_link
        base_to_dvl = TransformStamped()
        base_to_dvl.header.stamp = stamp
        base_to_dvl.header.frame_id = self.base_frame
        base_to_dvl.child_frame_id = 'dvl_link'
        
        base_to_dvl.transform.translation.x = self.robot_state.dvl_position[0]
        base_to_dvl.transform.translation.y = self.robot_state.dvl_position[1]
        base_to_dvl.transform.translation.z = self.robot_state.dvl_position[2]
        
        dvl_q = self.robot_state.dvl_orientation.as_quat()
        base_to_dvl.transform.rotation.x = dvl_q[0]
        base_to_dvl.transform.rotation.y = dvl_q[1]
        base_to_dvl.transform.rotation.z = dvl_q[2]
        base_to_dvl.transform.rotation.w = dvl_q[3]
        
        # base_link -> camera_link
        base_to_camera = TransformStamped()
        base_to_camera.header.stamp = stamp
        base_to_camera.header.frame_id = self.base_frame
        base_to_camera.child_frame_id = 'camera_link'
        
        base_to_camera.transform.translation.x = 0.5  # 假设相机在前方 0.5m
        base_to_camera.transform.translation.y = 0.0
        base_to_camera.transform.translation.z = 0.1
        base_to_camera.transform.rotation.w = 1.0
        
        # base_link -> sonar_link
        base_to_sonar = TransformStamped()
        base_to_sonar.header.stamp = stamp
        base_to_sonar.header.frame_id = self.base_frame
        base_to_sonar.child_frame_id = 'sonar_link'
        
        base_to_sonar.transform.translation.x = 0.6
        base_to_sonar.transform.translation.y = 0.0
        base_to_sonar.transform.translation.z = -0.2
        base_to_sonar.transform.rotation.w = 1.0
        
        # base_link -> imu_link
        base_to_imu = TransformStamped()
        base_to_imu.header.stamp = stamp
        base_to_imu.header.frame_id = self.base_frame
        base_to_imu.child_frame_id = 'imu_link'
        
        base_to_imu.transform.translation.x = 0.0
        base_to_imu.transform.translation.y = 0.0
        base_to_imu.transform.translation.z = 0.0
        base_to_imu.transform.rotation.w = 1.0
        
        # 发布所有变换
        self.tf_broadcaster.sendTransform([
            odom_to_base,
            base_to_dvl,
            base_to_camera,
            base_to_sonar,
            base_to_imu
        ])
        
    def reset_state(self, pose: Optional[RobotPose] = None):
        """重置状态估计"""
        self.robot_state.reset(pose)
        self.get_logger().info('机器人状态已重置')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = RobotStateNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'错误: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
