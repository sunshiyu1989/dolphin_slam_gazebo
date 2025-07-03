#!/usr/bin/env python3
"""
Dolphin SLAM 综合修复脚本
=========================

修复两个关键问题：
1. 时间同步问题：robot_state_node播放速度过快，与图像数据不匹配
2. 位置细胞网络问题：PlaceCellNetwork导入失败，神经元未激活

问题分析：
- robot_state_node以20Hz固定频率播放数据（太快！）
- place_cell_node没有正确导入PlaceCellNetwork类
- 需要基于时间戳的真正同步播放
"""

import os
import sys
import shutil
from pathlib import Path
import subprocess
from datetime import datetime

class DolphinSLAMComprehensiveFixer:
    """Dolphin SLAM综合修复器"""
    
    def __init__(self, workspace_path: str = "~/dolphin_slam_ws"):
        self.workspace_path = Path(workspace_path).expanduser()
        self.src_path = self.workspace_path / "src" / "dolphin_slam"
        self.nodes_path = self.src_path / "nodes"
        
        print(f"🔍 工作空间: {self.workspace_path}")
        print(f"📁 源代码路径: {self.src_path}")
        
        if not self.src_path.exists():
            print(f"❌ 错误: 找不到源代码路径 {self.src_path}")
            sys.exit(1)
            
    def create_backup(self):
        """创建备份"""
        print("💾 创建备份...")
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        backup_dir = self.workspace_path / f"backup_comprehensive_{timestamp}"
        backup_dir.mkdir(exist_ok=True)
        
        files_to_backup = [
            self.nodes_path / "place_cell_node.py",
            self.nodes_path / "robot_state_node.py",
        ]
        
        for file_path in files_to_backup:
            if file_path.exists():
                backup_path = backup_dir / file_path.name
                shutil.copy2(file_path, backup_path)
                print(f"✅ 已备份: {file_path.name}")
                
        return backup_dir
        
    def check_place_cell_network_exists(self):
        """检查PlaceCellNetwork类是否存在"""
        pcn_file = self.src_path / "dolphin_slam" / "place_cell_network.py"
        if pcn_file.exists():
            print(f"✅ 找到PlaceCellNetwork: {pcn_file}")
            return True
        else:
            print(f"❌ 未找到PlaceCellNetwork: {pcn_file}")
            return False
            
    def fix_place_cell_node(self):
        """修复位置细胞节点 - 正确导入PlaceCellNetwork"""
        print("🔧 修复 place_cell_node.py - 正确集成PlaceCellNetwork...")
        
        fixed_code = '''#!/usr/bin/env python3
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
                ('activation_threshold', 0.1),
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
        
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        if self.last_timestamp is not None:
            dt = current_time - self.last_timestamp
            
            if 0 < dt < 1.0 and self.import_success and self.place_cell_network:
                try:
                    # 使用真正的PlaceCellNetwork
                    angular_velocity = 0.0
                    self.place_cell_network.path_integration_update(velocity, angular_velocity, dt)
                    
                except Exception as e:
                    self.get_logger().debug(f'路径积分更新失败: {e}')
            
            elif 0 < dt < 1.0 and not self.import_success:
                # 使用备用网络
                self.update_fallback_network(velocity, dt)
        
        self.last_position = current_position
        self.last_timestamp = current_time
        
        if self.debug_mode and self.update_count % 50 == 0:
            self.get_logger().debug(f'里程计: 位置={current_position}, 速度={velocity}')
        
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
        print("\\n🛑 用户中断")
    except Exception as e:
        print(f'❌ 节点错误: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
'''
        
        place_cell_file = self.nodes_path / "place_cell_node.py"
        with open(place_cell_file, 'w', encoding='utf-8') as f:
            f.write(fixed_code)
            
        print(f"✅ place_cell_node.py 已修复 - 正确集成PlaceCellNetwork")
        
    def fix_robot_state_node(self):
        """修复机器人状态节点 - 正确的时间同步"""
        print("🔧 修复 robot_state_node.py - 修复时间同步问题...")
        
        fixed_code = '''#!/usr/bin/env python3
"""
Dolphin SLAM - 机器人状态 ROS2 节点 (时间同步修复版)
修复导航数据播放过快的问题，实现真正的时间同步
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
    """机器人状态 ROS2 节点 (时间同步修复版)"""
    
    def __init__(self):
        super().__init__('robot_state_node')
        
        # 声明参数
        self.declare_parameters(
            namespace='',
            parameters=[
                ('navigation_csv', ''),
                ('publish_rate', 10.0),  # 降低发布频率
                ('odom_frame', 'odom'),
                ('base_frame', 'base_link'),
                ('use_ekf', False),
                ('process_noise_std', 0.1),
                ('measurement_noise_std', 0.05),
                ('playback_speed', 1.0),  # 播放速度倍数
                ('sync_tolerance', 0.1),  # 同步容差(秒)
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
        
        # 当前状态
        self.current_pose = {
            'x': 0.0, 'y': 0.0, 'z': 0.0,
            'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0
        }
        self.current_velocity = {
            'vx': 0.0, 'vy': 0.0, 'vz': 0.0,
            'wx': 0.0, 'wy': 0.0, 'wz': 0.0
        }
        
        # 加载导航数据
        if self.navigation_csv:
            self.load_navigation_data()
        
        # 发布者
        self.odometry_pub = self.create_publisher(
            Odometry,
            '/dolphin_slam/odometry',  # 修正话题名称
            10
        )
        
        # 定时器 - 重要：使用合理的频率
        self.publish_timer = self.create_timer(
            1.0 / self.publish_rate,
            self.publish_state
        )
        
        # 导航数据播放定时器 - 修复：使用更合适的频率
        if self.data_loaded:
            self.nav_timer = self.create_timer(
                0.02,  # 50Hz检查，但基于时间戳播放
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
            
            # 数据预处理
            required_columns = ['timestamp', 'x', 'y', 'z', 'roll', 'pitch', 'yaw']
            missing_columns = [col for col in required_columns if col not in self.navigation_data.columns]
            
            if missing_columns:
                self.get_logger().error(f'缺少必需列: {missing_columns}')
                return
            
            # 数据类型转换
            for col in required_columns:
                self.navigation_data[col] = pd.to_numeric(self.navigation_data[col], errors='coerce')
            
            # 删除无效行
            self.navigation_data = self.navigation_data.dropna()
            
            # 按时间戳排序
            self.navigation_data = self.navigation_data.sort_values('timestamp').reset_index(drop=True)
            
            self.data_loaded = True
            
            self.get_logger().info(f'成功加载导航数据: {len(self.navigation_data)} 条记录')
            self.get_logger().info(f'数据类型转换完成')
            
            # 如果有RobotState，也加载到那里
            if self.robot_state:
                self.robot_state.load_navigation_data(self.navigation_csv)
            
        except Exception as e:
            self.get_logger().error(f'加载导航数据失败: {e}')
            self.data_loaded = False
            
    def update_navigation_playback(self):
        """更新导航数据播放 - 基于真实时间戳同步"""
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
        max_updates_per_cycle = 10  # 限制每次最多处理的数据点
        
        while (self.nav_data_index < len(self.navigation_data) and 
               updates_this_cycle < max_updates_per_cycle):
            
            current_data_time = self.navigation_data['timestamp'].iloc[self.nav_data_index]
            
            # 检查是否到了播放时间
            if current_data_time <= target_data_time + self.sync_tolerance:
                # 更新状态
                row = self.navigation_data.iloc[self.nav_data_index]
                
                self.current_pose = {
                    'x': float(row['x']),
                    'y': float(row['y']), 
                    'z': float(row['z']),
                    'roll': float(row['roll']),
                    'pitch': float(row['pitch']),
                    'yaw': float(row['yaw'])
                }
                
                # 计算速度（简化版本）
                if self.nav_data_index > 0:
                    prev_row = self.navigation_data.iloc[self.nav_data_index - 1]
                    dt = current_data_time - prev_row['timestamp']
                    
                    if dt > 0:
                        self.current_velocity = {
                            'vx': (row['x'] - prev_row['x']) / dt,
                            'vy': (row['y'] - prev_row['y']) / dt,
                            'vz': (row['z'] - prev_row['z']) / dt,
                            'wx': 0.0, 'wy': 0.0, 'wz': 0.0
                        }
                
                self.nav_data_index += 1
                updates_this_cycle += 1
                self.processed_count += 1
                
                # 定期报告进度 - 降低频率
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
        """发布机器人状态"""
        try:
            # 创建里程计消息
            odom_msg = Odometry()
            odom_msg.header.stamp = self.get_clock().now().to_msg()
            odom_msg.header.frame_id = self.odom_frame
            odom_msg.child_frame_id = self.base_frame
            
            # 位置
            odom_msg.pose.pose.position.x = self.current_pose['x']
            odom_msg.pose.pose.position.y = self.current_pose['y']
            odom_msg.pose.pose.position.z = self.current_pose['z']
            
            # 姿态
            q = Rotation.from_euler('xyz', [
                self.current_pose['roll'],
                self.current_pose['pitch'], 
                self.current_pose['yaw']
            ]).as_quat()
            
            odom_msg.pose.pose.orientation.x = float(q[0])
            odom_msg.pose.pose.orientation.y = float(q[1])
            odom_msg.pose.pose.orientation.z = float(q[2])
            odom_msg.pose.pose.orientation.w = float(q[3])
            
            # 速度
            odom_msg.twist.twist.linear.x = self.current_velocity['vx']
            odom_msg.twist.twist.linear.y = self.current_velocity['vy']
            odom_msg.twist.twist.linear.z = self.current_velocity['vz']
            odom_msg.twist.twist.angular.x = self.current_velocity['wx']
            odom_msg.twist.twist.angular.y = self.current_velocity['wy']
            odom_msg.twist.twist.angular.z = self.current_velocity['wz']
            
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
        print("\\n🛑 用户中断")
    except Exception as e:
        print(f'❌ 节点错误: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
'''
        
        robot_state_file = self.nodes_path / "robot_state_node.py"
        with open(robot_state_file, 'w', encoding='utf-8') as f:
            f.write(fixed_code)
            
        print(f"✅ robot_state_node.py 已修复 - 修复时间同步")
        
    def rebuild_project(self):
        """重新构建项目"""
        print("🔨 重新构建项目...")
        
        os.chdir(self.workspace_path)
        
        # 清理构建缓存
        for cache_dir in ['build', 'install', 'log']:
            cache_path = self.workspace_path / cache_dir
            if cache_path.exists():
                shutil.rmtree(cache_path)
                print(f"🗑️  已清理: {cache_dir}")
        
        # 构建项目
        try:
            result = subprocess.run(
                ['colcon', 'build', '--packages-select', 'dolphin_slam', '--symlink-install'],
                capture_output=True,
                text=True,
                cwd=self.workspace_path
            )
            
            if result.returncode == 0:
                print("✅ 项目构建成功！")
                return True
            else:
                print(f"❌ 构建失败:")
                print(result.stderr)
                return False
                
        except FileNotFoundError:
            print("❌ 未找到 colcon 命令")
            return False
            
    def run_comprehensive_fix(self):
        """运行综合修复"""
        print("🚀 Dolphin SLAM 综合修复")
        print("=" * 50)
        
        # 检查PlaceCellNetwork
        pcn_exists = self.check_place_cell_network_exists()
        
        # 创建备份
        backup_dir = self.create_backup()
        
        # 修复位置细胞节点
        self.fix_place_cell_node()
        
        # 修复机器人状态节点
        self.fix_robot_state_node()
        
        # 重新构建
        if self.rebuild_project():
            print("\\n🎉 综合修复完成！")
            print("=" * 30)
            print("✅ 修复内容:")
            print("  1. place_cell_node.py - 正确导入PlaceCellNetwork + 备用实现")
            print("  2. robot_state_node.py - 修复时间同步问题")
            print("  3. 降低更新频率，减少CPU负载")
            print("  4. 添加详细的调试信息")
            print("\\n🚀 测试启动:")
            print("  source install/setup.bash")
            print("  ./start_dolphin.sh")
            print("\\n📋 预期改善:")
            print("  ✅ 神经元活跃度 > 0.1")
            print("  ✅ 导航数据播放速度匹配图像数据")
            print("  ✅ 位置细胞网络正常工作")
            print(f"\\n📦 备份位置: {backup_dir}")
        else:
            print("\\n❌ 修复失败，请检查构建错误")
            return False
            
        return True

def main():
    """主函数"""
    fixer = DolphinSLAMComprehensiveFixer()
    success = fixer.run_comprehensive_fix()
    
    if success:
        print("\\n✨ 现在重新启动系统测试！")
        print("   预期看到：神经元活跃度 > 0, 时间同步正常")
    else:
        print("\\n💡 如果仍有问题，请提供新的错误日志")
    
    return 0 if success else 1

if __name__ == "__main__":
    sys.exit(main())
