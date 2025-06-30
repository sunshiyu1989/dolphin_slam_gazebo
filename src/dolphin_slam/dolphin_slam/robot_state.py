#!/usr/bin/env python3
"""
Dolphin SLAM - 机器人状态估计模块
融合 DVL、IMU 和导航数据，提供里程计信息
"""

import numpy as np
from typing import Tuple, Optional, Dict
import logging
from dataclasses import dataclass
from scipy.spatial.transform import Rotation
import pandas as pd
from collections import deque

@dataclass
class RobotPose:
    """机器人位姿数据结构"""
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float
    timestamp: float
    
    def to_matrix(self) -> np.ndarray:
        """转换为 4x4 变换矩阵"""
        R = Rotation.from_euler('xyz', [self.roll, self.pitch, self.yaw]).as_matrix()
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = [self.x, self.y, self.z]
        return T
        
    @classmethod
    def from_matrix(cls, T: np.ndarray, timestamp: float):
        """从变换矩阵创建"""
        R = T[:3, :3]
        t = T[:3, 3]
        r = Rotation.from_matrix(R)
        euler = r.as_euler('xyz')
        return cls(x=t[0], y=t[1], z=t[2],
                  roll=euler[0], pitch=euler[1], yaw=euler[2],
                  timestamp=timestamp)

@dataclass
class Velocity:
    """速度数据结构"""
    vx: float  # 前向速度
    vy: float  # 侧向速度
    vz: float  # 垂直速度
    wx: float  # 滚转角速度
    wy: float  # 俯仰角速度
    wz: float  # 偏航角速度
    
class RobotState:
    """
    机器人状态估计器
    融合多传感器数据提供状态估计
    """
    
    def __init__(self,
                 dvl_position: Tuple[float, float, float] = (0.75, 0, -0.4),
                 dvl_orientation: Tuple[float, float, float] = (0, 0, 0),
                 use_ekf: bool = True,
                 process_noise_std: float = 0.1,
                 measurement_noise_std: float = 0.05):
        """
        初始化机器人状态估计器
        
        参数:
            dvl_position: DVL 相对于机器人中心的位置
            dvl_orientation: DVL 相对于机器人的方向（欧拉角）
            use_ekf: 是否使用扩展卡尔曼滤波
            process_noise_std: 过程噪声标准差
            measurement_noise_std: 测量噪声标准差
        """
        self.dvl_position = np.array(dvl_position)
        self.dvl_orientation = Rotation.from_euler('xyz', dvl_orientation)
        self.use_ekf = use_ekf
        
        self.logger = logging.getLogger(__name__)
        
        # 当前状态
        self.current_pose = RobotPose(0, 0, 0, 0, 0, 0, 0)
        self.current_velocity = Velocity(0, 0, 0, 0, 0, 0)
        
        # 导航数据（从 CSV 加载）
        self.navigation_data: Optional[pd.DataFrame] = None
        self.nav_index = 0
        
        # EKF 状态
        if self.use_ekf:
            # 状态向量: [x, y, z, roll, pitch, yaw, vx, vy, vz]
            self.state = np.zeros(9)
            self.covariance = np.eye(9) * 0.1
            
            # 过程噪声
            self.Q = np.eye(9) * process_noise_std**2
            
            # 测量噪声
            self.R_dvl = np.eye(3) * measurement_noise_std**2
            self.R_imu = np.eye(3) * measurement_noise_std**2
            
        # 历史记录
        self.pose_history = deque(maxlen=1000)
        self.velocity_history = deque(maxlen=100)
        
        # 里程计
        self.total_distance = 0.0
        self.last_update_time = None
        
    def load_navigation_data(self, csv_path: str):
        """
        加载导航数据 CSV 文件
        
        参数:
            csv_path: navigation.csv 的路径
        """
        try:
            self.navigation_data = pd.read_csv(csv_path)
            self.logger.info(f"加载了 {len(self.navigation_data)} 条导航记录")
            
            # 🔧 强制转换所有数值列为 float 类型（修复 ROS2 消息创建错误）
            self.logger.info("开始强制数据类型转换...")
            numeric_columns = ['latitude', 'longitude', 'altitude', 'depth',
                             'yaw', 'pitch', 'roll', 'velocity_x', 'velocity_y', 'velocity_z']
            
            for col in numeric_columns:
                if col in self.navigation_data.columns:
                    try:
                        # 强制转换为 float64，处理字符串和 NaN
                        original_type = self.navigation_data[col].dtype
                        self.navigation_data[col] = pd.to_numeric(
                            self.navigation_data[col], 
                            errors='coerce'
                        ).astype(float)
                        self.logger.debug(f"列 {col}: {original_type} -> float64")
                    except Exception as e:
                        self.logger.warning(f"转换列 {col} 失败: {e}")
                        # 如果转换失败，填充为 0.0
                        self.navigation_data[col] = 0.0
            
            # 强制填充所有 NaN 值为 0.0
            nan_count_before = self.navigation_data.isnull().sum().sum()
            self.navigation_data.fillna(0.0, inplace=True)
            self.logger.info(f"数据类型转换完成，填充了 {nan_count_before} 个 NaN 值")
            
            # 验证必要的列
            required_columns = ['timestamp', 'latitude', 'longitude', 'altitude',
                              'yaw', 'pitch', 'roll', 'velocity_x', 'velocity_y',
                              'velocity_z', 'depth']
            
            for col in required_columns:
                if col not in self.navigation_data.columns:
                    raise ValueError(f"缺少必要的列: {col}")
                    
            # 转换经纬度到局部坐标（简化处理）
            self._convert_to_local_coordinates()
            
        except Exception as e:
            self.logger.error(f"加载导航数据失败: {e}")
            raise


    def _convert_to_local_coordinates(self):
        """将经纬度转换为局部笛卡尔坐标"""
        if self.navigation_data is None:
            return
            
        # 使用第一个点作为原点
        origin_lat = self.navigation_data['latitude'].iloc[0]
        origin_lon = self.navigation_data['longitude'].iloc[0]
        
        # 简化的墨卡托投影（适用于小范围）
        R_earth = 6371000  # 地球半径（米）
        
        # 转换为弧度
        lat_rad = np.radians(self.navigation_data['latitude'])
        lon_rad = np.radians(self.navigation_data['longitude'])
        origin_lat_rad = np.radians(origin_lat)
        origin_lon_rad = np.radians(origin_lon)
        
        # 计算局部坐标
        self.navigation_data['x'] = R_earth * (lon_rad - origin_lon_rad) * np.cos(origin_lat_rad)
        self.navigation_data['y'] = R_earth * (lat_rad - origin_lat_rad)
        self.navigation_data['z'] = -self.navigation_data['depth']  # 深度转为 z 坐标
        
    def update_from_navigation(self, timestamp: float) -> bool:
        """
        从导航数据更新状态
        
        参数:
            timestamp: 当前时间戳
            
        返回:
            是否成功更新
        """
        if self.navigation_data is None:
            return False
            
        # 查找最接近的导航数据
        time_diff = np.abs(self.navigation_data['timestamp'] - timestamp)
        nearest_idx = np.argmin(time_diff)
        
        if time_diff[nearest_idx] > 0.5:  # 超过 0.5 秒认为数据太旧
            return False
            
        nav_row = self.navigation_data.iloc[nearest_idx]
        
        # 更新位姿
        self.current_pose = RobotPose(
            x=nav_row['x'],
            y=nav_row['y'],
            z=nav_row['z'],
            roll=np.radians(nav_row['roll']),
            pitch=np.radians(nav_row['pitch']),
            yaw=np.radians(nav_row['yaw']),
            timestamp=timestamp
        )
        
        # 更新速度
        self.current_velocity = Velocity(
            vx=nav_row['velocity_x'],
            vy=nav_row['velocity_y'],
            vz=nav_row['velocity_z'],
            wx=0, wy=0, wz=0  # 角速度需要从 IMU 获取
        )
        
        return True
        
    def update_dvl(self, velocity_body: np.ndarray, timestamp: float):
        """
        更新 DVL 测量
        
        参数:
            velocity_body: 机体坐标系下的速度 [vx, vy, vz]
            timestamp: 时间戳
        """
        if self.last_update_time is None:
            self.last_update_time = timestamp
            return
            
        dt = timestamp - self.last_update_time
        if dt <= 0:
            return
            
        # 将 DVL 速度转换到机器人坐标系
        dvl_to_robot = self.dvl_orientation.as_matrix()
        velocity_robot = dvl_to_robot @ velocity_body
        
        # 更新速度
        self.current_velocity.vx = velocity_robot[0]
        self.current_velocity.vy = velocity_robot[1]
        self.current_velocity.vz = velocity_robot[2]
        
        if self.use_ekf:
            self._ekf_predict(dt)
            self._ekf_update_dvl(velocity_robot)
        else:
            # 简单积分
            self._dead_reckoning_update(dt)
            
        # 更新里程计
        distance = np.linalg.norm(velocity_robot[:2]) * dt
        self.total_distance += distance
        
        self.last_update_time = timestamp
        
    def update_imu(self, orientation: np.ndarray, angular_velocity: np.ndarray,
                   timestamp: float):
        """
        更新 IMU 测量
        
        参数:
            orientation: 四元数 [w, x, y, z]
            angular_velocity: 角速度 [wx, wy, wz]
            timestamp: 时间戳
        """
        # 从四元数获取欧拉角
        r = Rotation.from_quat(orientation[[1, 2, 3, 0]])  # scipy 使用 x,y,z,w 顺序
        euler = r.as_euler('xyz')
        
        # 更新角速度
        self.current_velocity.wx = angular_velocity[0]
        self.current_velocity.wy = angular_velocity[1]
        self.current_velocity.wz = angular_velocity[2]
        
        if self.use_ekf:
            self._ekf_update_imu(euler)
        else:
            # 直接更新姿态
            self.current_pose.roll = euler[0]
            self.current_pose.pitch = euler[1]
            self.current_pose.yaw = euler[2]
            
    def _dead_reckoning_update(self, dt: float):
        """简单的航位推算更新"""
        # 获取当前姿态的旋转矩阵
        R = Rotation.from_euler('xyz', [self.current_pose.roll,
                                       self.current_pose.pitch,
                                       self.current_pose.yaw]).as_matrix()
        
        # 将机体速度转换到世界坐标系
        velocity_world = R @ np.array([self.current_velocity.vx,
                                      self.current_velocity.vy,
                                      self.current_velocity.vz])
        
        # 更新位置
        self.current_pose.x += velocity_world[0] * dt
        self.current_pose.y += velocity_world[1] * dt
        self.current_pose.z += velocity_world[2] * dt
        
        # 更新姿态（使用角速度）
        self.current_pose.roll += self.current_velocity.wx * dt
        self.current_pose.pitch += self.current_velocity.wy * dt
        self.current_pose.yaw += self.current_velocity.wz * dt
        
        # 归一化角度
        self.current_pose.yaw = np.arctan2(np.sin(self.current_pose.yaw),
                                          np.cos(self.current_pose.yaw))
        
    def _ekf_predict(self, dt: float):
        """EKF 预测步骤"""
        # 状态转移
        # 位置更新
        R = Rotation.from_euler('xyz', self.state[3:6]).as_matrix()
        velocity_world = R @ self.state[6:9]
        
        self.state[0:3] += velocity_world * dt
        
        # 姿态更新（简化，假设角速度恒定）
        # 实际应该使用四元数避免万向锁
        
        # 状态转移雅可比
        F = np.eye(9)
        F[0:3, 6:9] = R * dt
        
        # 协方差更新
        self.covariance = F @ self.covariance @ F.T + self.Q * dt
        
    def _ekf_update_dvl(self, velocity_measurement: np.ndarray):
        """EKF DVL 更新步骤"""
        # 测量模型：直接测量速度
        H = np.zeros((3, 9))
        H[0:3, 6:9] = np.eye(3)
        
        # 计算卡尔曼增益
        S = H @ self.covariance @ H.T + self.R_dvl
        K = self.covariance @ H.T @ np.linalg.inv(S)
        
        # 更新状态
        innovation = velocity_measurement - self.state[6:9]
        self.state += K @ innovation
        
        # 更新协方差
        self.covariance = (np.eye(9) - K @ H) @ self.covariance
        
        # 更新当前位姿
        self._update_pose_from_state()
        
    def _ekf_update_imu(self, euler_measurement: np.ndarray):
        """EKF IMU 更新步骤"""
        # 测量模型：直接测量姿态
        H = np.zeros((3, 9))
        H[0:3, 3:6] = np.eye(3)
        
        # 计算卡尔曼增益
        S = H @ self.covariance @ H.T + self.R_imu
        K = self.covariance @ H.T @ np.linalg.inv(S)
        
        # 更新状态
        innovation = euler_measurement - self.state[3:6]
        # 处理角度环绕
        innovation = np.arctan2(np.sin(innovation), np.cos(innovation))
        
        self.state += K @ innovation
        
        # 更新协方差
        self.covariance = (np.eye(9) - K @ H) @ self.covariance
        
        # 更新当前位姿
        self._update_pose_from_state()
        
    def _update_pose_from_state(self):
        """从 EKF 状态更新当前位姿"""
        if self.use_ekf:
            self.current_pose.x = self.state[0]
            self.current_pose.y = self.state[1]
            self.current_pose.z = self.state[2]
            self.current_pose.roll = self.state[3]
            self.current_pose.pitch = self.state[4]
            self.current_pose.yaw = self.state[5]
            self.current_velocity.vx = self.state[6]
            self.current_velocity.vy = self.state[7]
            self.current_velocity.vz = self.state[8]
            
    def get_pose(self) -> RobotPose:
        """获取当前位姿"""
        return self.current_pose
        
    def get_velocity(self) -> Velocity:
        """获取当前速度"""
        return self.current_velocity
        
    def get_odometry_delta(self, from_time: float, to_time: float) -> Tuple[np.ndarray, np.ndarray]:
        """
        获取两个时间点之间的里程计增量
        
        返回:
            (translation, rotation): 平移向量和旋转矩阵
        """
        # 简化实现，实际应该积分历史数据
        dt = to_time - from_time
        translation = np.array([
            self.current_velocity.vx * dt,
            self.current_velocity.vy * dt,
            self.current_velocity.vz * dt
        ])
        
        rotation = Rotation.from_euler('xyz', [
            self.current_velocity.wx * dt,
            self.current_velocity.wy * dt,
            self.current_velocity.wz * dt
        ]).as_matrix()
        
        return translation, rotation
        
    def reset(self, pose: Optional[RobotPose] = None):
        """重置状态"""
        if pose is None:
            pose = RobotPose(0, 0, 0, 0, 0, 0, 0)
            
        self.current_pose = pose
        self.current_velocity = Velocity(0, 0, 0, 0, 0, 0)
        
        if self.use_ekf:
            self.state = np.zeros(9)
            self.state[0:3] = [pose.x, pose.y, pose.z]
            self.state[3:6] = [pose.roll, pose.pitch, pose.yaw]
            self.covariance = np.eye(9) * 0.1
            
        self.pose_history.clear()
        self.velocity_history.clear()
        self.total_distance = 0.0
        self.last_update_time = None
