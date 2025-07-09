#!/usr/bin/env python3
"""
Dolphin SLAM - 工具函数
通用的辅助函数和工具
"""

import numpy as np
import cv2
from typing import Tuple, List, Optional, Union, Dict
import yaml
import os
import logging
from datetime import datetime
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle
import pandas as pd


def setup_logging(log_level: str = "INFO", log_file: Optional[str] = None):
    """
    设置日志系统
    
    参数:
        log_level: 日志级别
        log_file: 日志文件路径（可选）
    """
    log_format = '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    
    handlers = [logging.StreamHandler()]
    if log_file:
        handlers.append(logging.FileHandler(log_file))
        
    logging.basicConfig(
        level=getattr(logging, log_level.upper()),
        format=log_format,
        handlers=handlers
    )
    

def load_config(config_file: str) -> Dict:
    """
    加载 YAML 配置文件
    
    参数:
        config_file: 配置文件路径
        
    返回:
        config: 配置字典
    """
    with open(config_file, 'r') as f:
        config = yaml.safe_load(f)
    return config
    

def save_config(config: Dict, config_file: str):
    """保存配置到 YAML 文件"""
    with open(config_file, 'w') as f:
        yaml.dump(config, f, default_flow_style=False)
        

def normalize_angle(angle: float) -> float:
    """
    将角度归一化到 [-pi, pi]
    
    参数:
        angle: 输入角度（弧度）
        
    返回:
        normalized_angle: 归一化后的角度
    """
    while angle > np.pi:
        angle -= 2 * np.pi
    while angle < -np.pi:
        angle += 2 * np.pi
    return angle
    

def angle_difference(angle1: float, angle2: float) -> float:
    """
    计算两个角度之间的最小差值
    
    参数:
        angle1, angle2: 角度（弧度）
        
    返回:
        diff: 角度差（-pi 到 pi）
    """
    diff = angle1 - angle2
    return normalize_angle(diff)
    

def quaternion_to_euler(q: Union[List, np.ndarray]) -> Tuple[float, float, float]:
    """
    四元数转欧拉角
    
    参数:
        q: 四元数 [w, x, y, z] 或 [x, y, z, w]
        
    返回:
        (roll, pitch, yaw): 欧拉角（弧度）
    """
    if len(q) != 4:
        raise ValueError("四元数必须有 4 个元素")
        
    # 假设输入是 [x, y, z, w] 格式
    if abs(q[3]) > abs(q[0]):  # w 分量较大，可能是 [x,y,z,w]
        x, y, z, w = q
    else:  # 否则假设是 [w,x,y,z]
        w, x, y, z = q
        
    # 转换为欧拉角
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1, 1))
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    
    return roll, pitch, yaw
    

def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """
    欧拉角转四元数
    
    参数:
        roll, pitch, yaw: 欧拉角（弧度）
        
    返回:
        q: 四元数 [w, x, y, z]
    """
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)
    
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    
    return np.array([w, x, y, z])
    

def transform_points(points: np.ndarray, transform: np.ndarray) -> np.ndarray:
    """
    使用 4x4 变换矩阵变换点
    
    参数:
        points: Nx3 点数组
        transform: 4x4 变换矩阵
        
    返回:
        transformed_points: 变换后的点
    """
    # 转换为齐次坐标
    ones = np.ones((points.shape[0], 1))
    points_h = np.hstack([points, ones])
    
    # 应用变换
    transformed_h = points_h @ transform.T
    
    # 转换回 3D 坐标
    return transformed_h[:, :3]
    

def compute_transform_matrix(position: np.ndarray, 
                           orientation: Union[np.ndarray, List]) -> np.ndarray:
    """
    从位置和方向计算 4x4 变换矩阵
    
    参数:
        position: [x, y, z]
        orientation: 四元数 [w, x, y, z] 或欧拉角 [roll, pitch, yaw]
        
    返回:
        transform: 4x4 变换矩阵
    """
    T = np.eye(4)
    T[:3, 3] = position
    
    if len(orientation) == 4:
        # 四元数
        q = orientation
        if abs(q[0]) > abs(q[3]):  # 确保是 [w,x,y,z] 格式
            w, x, y, z = q
        else:
            x, y, z, w = q
            
        # 四元数到旋转矩阵
        T[0, 0] = 1 - 2 * (y*y + z*z)
        T[0, 1] = 2 * (x*y - w*z)
        T[0, 2] = 2 * (x*z + w*y)
        T[1, 0] = 2 * (x*y + w*z)
        T[1, 1] = 1 - 2 * (x*x + z*z)
        T[1, 2] = 2 * (y*z - w*x)
        T[2, 0] = 2 * (x*z - w*y)
        T[2, 1] = 2 * (y*z + w*x)
        T[2, 2] = 1 - 2 * (x*x + y*y)
    else:
        # 欧拉角
        roll, pitch, yaw = orientation
        from scipy.spatial.transform import Rotation
        R = Rotation.from_euler('xyz', [roll, pitch, yaw]).as_matrix()
        T[:3, :3] = R
        
    return T
    

def resize_image_aspect_ratio(image: np.ndarray, max_size: int = 640) -> np.ndarray:
    """
    保持纵横比缩放图像
    
    参数:
        image: 输入图像
        max_size: 最大边长
        
    返回:
        resized_image: 缩放后的图像
    """
    h, w = image.shape[:2]
    
    if max(h, w) <= max_size:
        return image
        
    if h > w:
        new_h = max_size
        new_w = int(w * max_size / h)
    else:
        new_w = max_size
        new_h = int(h * max_size / w)
        
    return cv2.resize(image, (new_w, new_h), interpolation=cv2.INTER_AREA)
    

def draw_matches(img1: np.ndarray, kp1: List, img2: np.ndarray, kp2: List,
                matches: List, num_matches: int = 50) -> np.ndarray:
    """
    绘制特征匹配
    
    参数:
        img1, img2: 两幅图像
        kp1, kp2: 关键点列表
        matches: 匹配列表
        num_matches: 要绘制的匹配数
        
    返回:
        result: 匹配结果图像
    """
    # 选择最佳匹配
    matches = sorted(matches, key=lambda x: x.distance)[:num_matches]
    
    # 绘制匹配
    result = cv2.drawMatches(img1, kp1, img2, kp2, matches, None,
                           flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
    
    return result
    

def create_circular_mask(shape: Tuple[int, int], center: Tuple[int, int], 
                        radius: int) -> np.ndarray:
    """
    创建圆形掩码
    
    参数:
        shape: 图像形状 (height, width)
        center: 圆心 (x, y)
        radius: 半径
        
    返回:
        mask: 二值掩码
    """
    mask = np.zeros(shape, dtype=np.uint8)
    cv2.circle(mask, center, radius, 255, -1)
    return mask
    

def compute_overlap_ratio(bbox1: Tuple, bbox2: Tuple) -> float:
    """
    计算两个边界框的重叠率（IoU）
    
    参数:
        bbox1, bbox2: 边界框 (x, y, w, h)
        
    返回:
        iou: 交并比
    """
    x1, y1, w1, h1 = bbox1
    x2, y2, w2, h2 = bbox2
    
    # 计算交集
    xi = max(x1, x2)
    yi = max(y1, y2)
    wi = min(x1 + w1, x2 + w2) - xi
    hi = min(y1 + h1, y2 + h2) - yi
    
    if wi <= 0 or hi <= 0:
        return 0.0
        
    # 计算面积
    intersection = wi * hi
    area1 = w1 * h1
    area2 = w2 * h2
    union = area1 + area2 - intersection
    
    return intersection / union if union > 0 else 0.0
    

def sliding_window_stats(data: np.ndarray, window_size: int, 
                        step_size: int = 1) -> Dict[str, np.ndarray]:
    """
    计算滑动窗口统计
    
    参数:
        data: 输入数据
        window_size: 窗口大小
        step_size: 步长
        
    返回:
        stats: 包含 mean, std, min, max 的字典
    """
    n = len(data)
    num_windows = (n - window_size) // step_size + 1
    
    stats = {
        'mean': np.zeros(num_windows),
        'std': np.zeros(num_windows),
        'min': np.zeros(num_windows),
        'max': np.zeros(num_windows)
    }
    
    for i in range(num_windows):
        start = i * step_size
        end = start + window_size
        window_data = data[start:end]
        
        stats['mean'][i] = np.mean(window_data)
        stats['std'][i] = np.std(window_data)
        stats['min'][i] = np.min(window_data)
        stats['max'][i] = np.max(window_data)
        
    return stats
    

def create_video_from_images(image_files: List[str], output_path: str, 
                           fps: int = 10, codec: str = 'mp4v'):
    """
    从图像序列创建视频
    
    参数:
        image_files: 图像文件列表
        output_path: 输出视频路径
        fps: 帧率
        codec: 视频编码器
    """
    if not image_files:
        return
        
    # 读取第一张图像获取尺寸
    first_img = cv2.imread(image_files[0])
    height, width = first_img.shape[:2]
    
    # 创建视频写入器
    fourcc = cv2.VideoWriter_fourcc(*codec)
    out = cv2.VideoWriter(output_path, fourcc, fps, (width, height))
    
    # 写入所有图像
    for img_file in image_files:
        img = cv2.imread(img_file)
        if img is not None:
            out.write(img)
            
    out.release()
    

def interpolate_poses(pose1: Dict, pose2: Dict, alpha: float) -> Dict:
    """
    在两个位姿之间插值
    
    参数:
        pose1, pose2: 位姿字典 {'position': [x,y,z], 'orientation': [w,x,y,z]}
        alpha: 插值系数 (0-1)
        
    返回:
        interpolated_pose: 插值后的位姿
    """
    # 位置线性插值
    pos1 = np.array(pose1['position'])
    pos2 = np.array(pose2['position'])
    interp_pos = (1 - alpha) * pos1 + alpha * pos2
    
    # 方向球面线性插值（SLERP）
    from scipy.spatial.transform import Rotation, Slerp
    
    q1 = pose1['orientation']
    q2 = pose2['orientation']
    
    # 创建旋转对象
    rotations = Rotation.from_quat([q1, q2])
    slerp = Slerp([0, 1], rotations)
    interp_rot = slerp(alpha)
    
    return {
        'position': interp_pos.tolist(),
        'orientation': interp_rot.as_quat().tolist()
    }
    

def compute_trajectory_error(gt_trajectory: np.ndarray, 
                           est_trajectory: np.ndarray) -> Dict[str, float]:
    """
    计算轨迹误差指标
    
    参数:
        gt_trajectory: 真实轨迹 Nx3
        est_trajectory: 估计轨迹 Nx3
        
    返回:
        errors: 包含 ATE, RPE 等误差指标
    """
    assert len(gt_trajectory) == len(est_trajectory)
    
    # 绝对轨迹误差 (ATE)
    ate = np.sqrt(np.mean(np.sum((gt_trajectory - est_trajectory)**2, axis=1)))
    
    # 相对位姿误差 (RPE)
    if len(gt_trajectory) > 1:
        gt_deltas = np.diff(gt_trajectory, axis=0)
        est_deltas = np.diff(est_trajectory, axis=0)
        rpe = np.sqrt(np.mean(np.sum((gt_deltas - est_deltas)**2, axis=1)))
    else:
        rpe = 0.0
        
    # 最大误差
    max_error = np.max(np.linalg.norm(gt_trajectory - est_trajectory, axis=1))
    
    # 最终位置误差
    final_error = np.linalg.norm(gt_trajectory[-1] - est_trajectory[-1])
    
    return {
        'ate': ate,
        'rpe': rpe,
        'max_error': max_error,
        'final_error': final_error
    }
    

def save_trajectory_plot(trajectory: np.ndarray, output_path: str, 
                        title: str = "Robot Trajectory"):
    """
    保存轨迹图
    
    参数:
        trajectory: Nx3 轨迹数据
        output_path: 输出路径
        title: 图标题
    """
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    ax.plot(trajectory[:, 0], trajectory[:, 1], trajectory[:, 2], 'b-')
    ax.scatter(trajectory[0, 0], trajectory[0, 1], trajectory[0, 2], 
              c='g', s=100, label='Start')
    ax.scatter(trajectory[-1, 0], trajectory[-1, 1], trajectory[-1, 2], 
              c='r', s=100, label='End')
    
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title(title)
    ax.legend()
    
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close()
    

def create_occupancy_grid(points: np.ndarray, resolution: float = 0.1,
                         bounds: Optional[Tuple] = None) -> Tuple[np.ndarray, Tuple]:
    """
    从点云创建占用栅格地图
    
    参数:
        points: Nx2 或 Nx3 点云
        resolution: 栅格分辨率（米）
        bounds: ((x_min, y_min), (x_max, y_max))
        
    返回:
        grid: 占用栅格
        grid_bounds: 栅格边界
    """
    # 提取 XY 坐标
    xy_points = points[:, :2]
    
    # 确定边界
    if bounds is None:
        min_bounds = np.min(xy_points, axis=0) - resolution
        max_bounds = np.max(xy_points, axis=0) + resolution
    else:
        min_bounds, max_bounds = bounds
        
    # 计算栅格大小
    grid_size = ((max_bounds - min_bounds) / resolution).astype(int)
    grid = np.zeros(grid_size, dtype=np.uint8)
    
    # 填充栅格
    for point in xy_points:
        idx = ((point - min_bounds) / resolution).astype(int)
        if 0 <= idx[0] < grid_size[0] and 0 <= idx[1] < grid_size[1]:
            grid[idx[0], idx[1]] = 255
            
    return grid, (min_bounds, max_bounds)


class Timer:
    """简单的计时器类"""
    
    def __init__(self):
        self.times = {}
        
    def start(self, name: str):
        """开始计时"""
        import time
        self.times[name] = time.time()
        
    def stop(self, name: str) -> float:
        """停止计时并返回经过的时间"""
        import time
        if name in self.times:
            elapsed = time.time() - self.times[name]
            del self.times[name]
            return elapsed
        return 0.0
        
    def __enter__(self):
        import time
        self.start_time = time.time()
        return self
        
    def __exit__(self, *args):
        import time
        self.elapsed = time.time() - self.start_time
