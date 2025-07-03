#!/usr/bin/env python3
"""
Dolphin SLAM 修复脚本
=====================
修复位置细胞网络和经验地图的关键问题

问题1: place_cell_node.py 使用简化模拟而非真正的PlaceCellNetwork
问题2: 经验地图轨迹长度限制导致重复日志

这个脚本会：
1. 修复place_cell_node.py 使用真正的PlaceCellNetwork类
2. 修复experience_map_node.py 的轨迹管理问题
3. 备份原文件并提供回滚选项
"""

import os
import sys
import shutil
from pathlib import Path
import subprocess
from datetime import datetime

class DolphinSLAMFixer:
    """Dolphin SLAM修复器"""
    
    def __init__(self, workspace_path: str = "~/dolphin_slam_ws"):
        self.workspace_path = Path(workspace_path).expanduser()
        self.src_path = self.workspace_path / "src" / "dolphin_slam"
        self.nodes_path = self.src_path / "nodes"
        self.dolphin_slam_path = self.src_path / "dolphin_slam"
        
        print(f"🔍 工作空间: {self.workspace_path}")
        print(f"📁 源代码路径: {self.src_path}")
        
        # 检查路径
        if not self.src_path.exists():
            print(f"❌ 错误: 找不到源代码路径 {self.src_path}")
            sys.exit(1)
            
    def create_backup(self):
        """创建备份"""
        print("💾 创建备份...")
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        backup_dir = self.workspace_path / f"backup_{timestamp}"
        backup_dir.mkdir(exist_ok=True)
        
        files_to_backup = [
            self.nodes_path / "place_cell_node.py",
            self.nodes_path / "experience_map_node.py"
        ]
        
        for file_path in files_to_backup:
            if file_path.exists():
                backup_path = backup_dir / file_path.name
                shutil.copy2(file_path, backup_path)
                print(f"✅ 已备份: {file_path.name} -> {backup_path}")
                
        print(f"📦 备份完成: {backup_dir}")
        return backup_dir
        
    def create_fixed_place_cell_node(self):
        """创建修复后的位置细胞节点 - 使用真正的PlaceCellNetwork"""
        print("🔧 修复 place_cell_node.py - 集成真正的PlaceCellNetwork...")
        
        fixed_code = '''#!/usr/bin/env python3
"""
Dolphin SLAM - 位置细胞网络 ROS2 节点 (修复版)
使用真正的PlaceCellNetwork类，而非简化模拟
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

# 添加dolphin_slam包到Python路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'dolphin_slam'))

# 导入真正的PlaceCellNetwork类
try:
    from place_cell_network import PlaceCellNetwork
    print("✅ 成功导入 PlaceCellNetwork 类")
except ImportError as e:
    print(f"❌ 无法导入 PlaceCellNetwork: {e}")
    sys.exit(1)

class PlaceCellNode(Node):
    """位置细胞网络 ROS2 节点 (修复版)"""
    
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
                ('update_rate', 20.0),
                ('activation_threshold', 0.1),
                ('initial_activity_strength', 0.5),
                ('neurons_step', 0.25),
                ('recurrent_connection_std', 2.0),
                ('input_learning_rate', 0.1),
                ('min_input_age', 10),
                ('weight_function', 'mexican_hat'),
            ]
        )
        
        # 获取参数
        self.update_rate = self.get_parameter('update_rate').value
        self.neurons_per_dimension = self.get_parameter('neurons_per_dimension').value
        self.activation_threshold = self.get_parameter('activation_threshold').value
        
        # 创建真正的位置细胞网络！
        try:
            self.place_cell_network = PlaceCellNetwork(
                neurons_per_dim=self.neurons_per_dimension,
                neurons_step=self.get_parameter('neurons_step').value,
                recurrent_conn_std=self.get_parameter('recurrent_connection_std').value,
                input_learning_rate=self.get_parameter('input_learning_rate').value,
                min_input_age=self.get_parameter('min_input_age').value,
                weight_function=self.get_parameter('weight_function').value
            )
            
            # 确保网络正确初始化
            self.place_cell_network.reset()
            
            print(f"✅ 位置细胞网络已创建: {self.neurons_per_dimension}³ 神经元")
            print(f"   初始最大活动: {np.max(self.place_cell_network.activity):.3f}")
            print(f"   活跃神经元数: {np.sum(self.place_cell_network.activity > self.activation_threshold)}")
            
        except Exception as e:
            self.get_logger().error(f"创建位置细胞网络失败: {e}")
            raise
        
        # 状态变量
        self.last_odometry: Optional[Odometry] = None
        self.last_position = np.zeros(3)
        self.last_timestamp = None
        self.update_count = 0
        
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
        
        self.stats_pub = self.create_publisher(
            MarkerArray,
            '/place_cells/statistics',
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
        
        self.stats_timer = self.create_timer(1.0, self.publish_statistics)
        
        self.get_logger().info(f'位置细胞网络节点已启动: {self.neurons_per_dimension}³ 神经元')
        
    def odometry_callback(self, msg: Odometry):
        """处理里程计数据 - 执行路径积分"""
        self.last_odometry = msg
        
        # 提取位置和速度
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
        
        # 计算时间间隔
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        if self.last_timestamp is not None:
            dt = current_time - self.last_timestamp
            
            # 执行路径积分更新
            if dt > 0 and dt < 1.0:  # 合理的时间间隔
                try:
                    # 计算角速度 (简化版本，假设为0)
                    angular_velocity = 0.0
                    
                    # 更新位置细胞网络
                    self.place_cell_network.path_integration_update(velocity, angular_velocity, dt)
                    
                    self.get_logger().debug(f'路径积分: 速度={velocity}, dt={dt:.3f}s')
                    
                except Exception as e:
                    self.get_logger().error(f'路径积分更新失败: {e}')
        
        self.last_position = current_position
        self.last_timestamp = current_time
        
    def visual_match_callback(self, msg: Float32MultiArray):
        """处理视觉匹配数据 - 添加视觉输入"""
        if len(msg.data) >= 2:
            template_id = int(msg.data[0])
            similarity = msg.data[1]
            
            # 应用视觉输入到网络
            try:
                self.place_cell_network.visual_input_update(template_id, similarity)
                self.get_logger().debug(f'视觉输入: 模板={template_id}, 相似度={similarity:.3f}')
            except Exception as e:
                self.get_logger().error(f'视觉输入更新失败: {e}')
        
    def update_network(self):
        """更新神经网络"""
        try:
            self.update_count += 1
            
            # 应用递归动力学
            self.place_cell_network.apply_recurrent_dynamics()
            
            # 发布活动数据
            self.publish_activity()
            
            # 定期报告状态
            if self.update_count % 250 == 0:  # 每5秒一次 (20Hz * 250)
                max_activity = np.max(self.place_cell_network.activity)
                active_neurons = np.sum(self.place_cell_network.activity > self.activation_threshold)
                center = self.place_cell_network.get_activity_center()
                
                self.get_logger().info(
                    f'网络状态: {active_neurons}/{self.neurons_per_dimension**3} 神经元活跃, '
                    f'最大活动: {max_activity:.3f}, 活动中心: [{center[0]:.1f}, {center[1]:.1f}, {center[2]:.1f}]'
                )
                
        except Exception as e:
            self.get_logger().error(f'网络更新错误: {e}')
            
    def publish_activity(self):
        """发布网络活动数据"""
        try:
            # 扁平化3D活动数据为1D
            activity_flat = self.place_cell_network.activity.flatten()
            
            msg = Float32MultiArray()
            msg.data = activity_flat.tolist()
            self.activity_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f'发布活动数据失败: {e}')
            
    def publish_statistics(self):
        """发布统计信息和可视化数据"""
        try:
            # 创建统计标记
            marker_array = MarkerArray()
            
            # 活动中心标记
            center_marker = Marker()
            center_marker.header.frame_id = "map"
            center_marker.header.stamp = self.get_clock().now().to_msg()
            center_marker.ns = "place_cell_center"
            center_marker.id = 0
            center_marker.type = Marker.SPHERE
            center_marker.action = Marker.ADD
            
            center = self.place_cell_network.get_activity_center()
            center_marker.pose.position.x = float(center[0] * 0.25)  # 缩放到实际尺寸
            center_marker.pose.position.y = float(center[1] * 0.25)
            center_marker.pose.position.z = float(center[2] * 0.25)
            
            center_marker.scale.x = 0.5
            center_marker.scale.y = 0.5
            center_marker.scale.z = 0.5
            
            center_marker.color.r = 1.0
            center_marker.color.g = 0.0
            center_marker.color.b = 0.0
            center_marker.color.a = 0.8
            
            marker_array.markers.append(center_marker)
            
            self.visualization_pub.publish(marker_array)
            
        except Exception as e:
            self.get_logger().debug(f'发布可视化数据失败: {e}')

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
        
        # 写入修复后的文件
        place_cell_file = self.nodes_path / "place_cell_node.py"
        with open(place_cell_file, 'w', encoding='utf-8') as f:
            f.write(fixed_code)
            
        print(f"✅ place_cell_node.py 已修复")
        
    def create_fixed_experience_map_node(self):
        """创建修复后的经验地图节点 - 解决轨迹重复问题"""
        print("🔧 修复 experience_map_node.py - 解决轨迹重复问题...")
        
        fixed_code = '''#!/usr/bin/env python3
"""
Dolphin SLAM - 经验地图 ROS2 节点 (修复版)
解决轨迹重复日志问题
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray, Marker
import numpy as np
from typing import Optional

class ExperienceMapNode(Node):
    """经验地图 ROS2 节点 (修复版)"""
    
    def __init__(self):
        super().__init__('experience_map_node')
        
        # 声明参数
        self.declare_parameters(
            namespace='',
            parameters=[
                ('odometry_topic', '/dolphin_slam/odometry'),
                ('trajectory_topic', '/dolphin_slam/trajectory'),
                ('max_trajectory_length', 2000),  # 增加到2000
                ('cleanup_threshold', 1500),      # 1500时开始清理
                ('log_interval', 100),             # 每100个点记录一次日志
            ]
        )
        
        # 状态变量
        self.current_odometry: Optional[Odometry] = None
        self.experience_count = 0
        self.trajectory_poses = []
        self.last_log_count = 0
        
        # 获取参数
        self.max_trajectory_length = self.get_parameter('max_trajectory_length').value
        self.cleanup_threshold = self.get_parameter('cleanup_threshold').value
        self.log_interval = self.get_parameter('log_interval').value
        
        # 订阅者
        self.pc_activity_sub = self.create_subscription(
            Float32MultiArray,
            '/place_cells/activity',
            self.place_cell_callback,
            10
        )
        
        self.visual_match_sub = self.create_subscription(
            Float32MultiArray,
            '/local_view/matches',
            self.visual_match_callback,
            10
        )
        
        self.odometry_sub = self.create_subscription(
            Odometry,
            self.get_parameter('odometry_topic').value,
            self.odometry_callback,
            10
        )
        
        # 发布者
        self.experience_pub = self.create_publisher(
            Float32MultiArray,
            '/experience_map/experiences',
            10
        )
        
        self.map_markers_pub = self.create_publisher(
            MarkerArray,
            '/experience_map/markers',
            10
        )
        
        self.trajectory_pub = self.create_publisher(
            Path,
            self.get_parameter('trajectory_topic').value,
            10
        )
        
        # 定时器
        self.update_timer = self.create_timer(0.2, self.update_map)
        self.viz_timer = self.create_timer(1.0, self.publish_visualizations)
        
        self.get_logger().info('经验地图节点已启动 - 修复版')
        
    def odometry_callback(self, msg: Odometry):
        """处理里程计数据 - 智能轨迹管理"""
        self.current_odometry = msg
        
        # 更新轨迹
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose = msg.pose.pose
        self.trajectory_poses.append(pose_stamped)
        
        # 智能轨迹管理 - 避免频繁重复日志
        trajectory_count = len(self.trajectory_poses)
        
        # 智能清理：当超过阈值时，保留最近的点
        if trajectory_count > self.max_trajectory_length:
            # 保留最近的cleanup_threshold个点
            self.trajectory_poses = self.trajectory_poses[-self.cleanup_threshold:]
            
            # 只在首次清理时记录日志
            if trajectory_count == self.max_trajectory_length + 1:
                self.get_logger().info(
                    f'轨迹点数达到上限({self.max_trajectory_length})，已清理至{self.cleanup_threshold}个点'
                )
        
        # 定期记录轨迹状态，但不要太频繁
        elif trajectory_count % self.log_interval == 0:
            self.get_logger().info(f'轨迹点数: {trajectory_count}')
        
    def place_cell_callback(self, msg: Float32MultiArray):
        """处理位置细胞活动"""
        if len(msg.data) > 0:
            max_activity = max(msg.data)
            active_count = sum(1 for x in msg.data if x > 0.1)
            self.get_logger().debug(f'位置细胞活动: 最大={max_activity:.3f}, 活跃={active_count}')
        
    def visual_match_callback(self, msg: Float32MultiArray):
        """处理视觉匹配结果"""
        if len(msg.data) > 0:
            similarity = msg.data[0]
            self.get_logger().debug(f'视觉匹配: {similarity:.3f}')
        
    def update_map(self):
        """更新经验地图"""
        if self.current_odometry is None:
            return
            
        # 简单的经验创建逻辑
        self.experience_count += 1
        
        # 只在特定间隔记录经验数
        if self.experience_count % (self.log_interval * 2) == 0:
            self.get_logger().info(f'经验计数: {self.experience_count}')
        
    def publish_visualizations(self):
        """发布可视化信息"""
        try:
            # 发布轨迹
            if len(self.trajectory_poses) > 1:
                trajectory = Path()
                trajectory.header.frame_id = "map"
                trajectory.header.stamp = self.get_clock().now().to_msg()
                trajectory.poses = self.trajectory_poses
                self.trajectory_pub.publish(trajectory)
                
        except Exception as e:
            self.get_logger().debug(f'可视化错误: {e}')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = ExperienceMapNode()
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
        
        # 写入修复后的文件
        experience_map_file = self.nodes_path / "experience_map_node.py"
        with open(experience_map_file, 'w', encoding='utf-8') as f:
            f.write(fixed_code)
            
        print(f"✅ experience_map_node.py 已修复")
        
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
            print("❌ 未找到 colcon 命令，请确保ROS2已正确安装")
            return False
            
    def run_fix(self):
        """运行完整修复流程"""
        print("🚀 Dolphin SLAM 修复脚本")
        print("=" * 50)
        
        # 1. 创建备份
        backup_dir = self.create_backup()
        
        # 2. 修复place_cell_node.py
        self.create_fixed_place_cell_node()
        
        # 3. 修复experience_map_node.py  
        self.create_fixed_experience_map_node()
        
        # 4. 重新构建
        if self.rebuild_project():
            print("\\n🎉 修复完成！")
            print("=" * 30)
            print("✅ 修复内容:")
            print("  1. place_cell_node.py - 集成真正的PlaceCellNetwork类")
            print("  2. experience_map_node.py - 修复轨迹重复日志问题")
            print("  3. 项目重新构建成功")
            print("\\n🚀 测试启动:")
            print("  source install/setup.bash")
            print("  ./start_dolphin.sh")
            print(f"\\n📦 备份位置: {backup_dir}")
        else:
            print("\\n❌ 修复失败，请检查构建错误")
            return False
            
        return True

def main():
    """主函数"""
    fixer = DolphinSLAMFixer()
    success = fixer.run_fix()
    
    if success:
        print("\\n✨ 现在应该能看到神经元正常活跃了！")
    else:
        print("\\n💡 如果仍有问题，请提供新的错误日志")
    
    return 0 if success else 1

if __name__ == "__main__":
    sys.exit(main())
