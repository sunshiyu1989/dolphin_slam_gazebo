#!/bin/bash
# Dolphin SLAM 立即部署脚本
# 基于项目资料重建的可工作节点

echo "🐬 部署基于项目资料的 Dolphin SLAM 节点"
echo "======================================="

WORKSPACE_DIR="$HOME/dolphin_slam_ws"
cd "$WORKSPACE_DIR"

# 备份现有文件
echo "📦 备份现有节点文件..."
BACKUP_DIR="backup_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$BACKUP_DIR"

if [ -d "src/dolphin_slam/dolphin_slam" ]; then
    cp -r src/dolphin_slam/dolphin_slam "$BACKUP_DIR/"
fi

if [ -f "src/dolphin_slam/config/dolphin_slam_params.yaml" ]; then
    cp src/dolphin_slam/config/dolphin_slam_params.yaml "$BACKUP_DIR/"
fi

echo "✅ 备份保存在: $BACKUP_DIR"

# 创建目录结构
mkdir -p src/dolphin_slam/dolphin_slam
mkdir -p src/dolphin_slam/config

echo "📝 1. 部署 robot_state_node.py..."
cat > src/dolphin_slam/dolphin_slam/robot_state_node.py << 'EOF'
#!/usr/bin/env python3
"""
Dolphin SLAM - 机器人状态节点 (基于项目资料重建)
发布里程计和轨迹数据
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistWithCovarianceStamped
from tf2_ros import TransformBroadcaster
import numpy as np
from scipy.spatial.transform import Rotation
import pandas as pd
import os

class RobotState:
    """机器人状态估计器"""
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.wx = 0.0
        self.wy = 0.0
        self.wz = 0.0
        self.navigation_data = None
        
    def load_navigation_data(self, csv_path):
        """加载导航数据"""
        if os.path.exists(csv_path):
            self.navigation_data = pd.read_csv(csv_path)
            return True
        return False
    
    def update_from_navigation(self, timestamp):
        """从导航数据更新状态"""
        if self.navigation_data is None:
            return False
            
        # 找最近的时间戳
        idx = (self.navigation_data['timestamp'] - timestamp).abs().idxmin()
        row = self.navigation_data.iloc[idx]
        
        self.x = float(row.get('x', 0.0))
        self.y = float(row.get('y', 0.0))
        self.z = float(row.get('z', 0.0))
        self.roll = float(row.get('roll', 0.0))
        self.pitch = float(row.get('pitch', 0.0))
        self.yaw = float(row.get('yaw', 0.0))
        self.vx = float(row.get('vx', 0.0))
        self.vy = float(row.get('vy', 0.0))
        self.vz = float(row.get('vz', 0.0))
        
        return True

class RobotStateNode(Node):
    """机器人状态ROS2节点"""
    
    def __init__(self):
        super().__init__('robot_state_node')
        
        # 声明参数
        self.declare_parameters(
            namespace='',
            parameters=[
                ('navigation_csv', ''),
                ('base_frame', 'base_link'),
                ('odom_frame', 'odom'),
                ('publish_rate', 20.0),
                ('publish_tf', True),
            ]
        )
        
        # 获取参数
        self.navigation_csv = self.get_parameter('navigation_csv').value
        self.base_frame = self.get_parameter('base_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.publish_tf = self.get_parameter('publish_tf').value
        
        # 初始化状态
        self.robot_state = RobotState()
        self.nav_data_index = 0
        
        # TF广播器
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # 发布者 - 使用dolphin_slam命名空间
        self.odometry_pub = self.create_publisher(
            Odometry,
            '/dolphin_slam/odometry',
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
        
        # 加载导航数据
        if self.navigation_csv:
            success = self.robot_state.load_navigation_data(self.navigation_csv)
            if success:
                self.get_logger().info(f'导航数据已加载: {self.navigation_csv}')
                # 创建播放定时器
                self.nav_playback_timer = self.create_timer(0.05, self.playback_navigation_data)
            else:
                self.get_logger().warn(f'无法加载导航数据: {self.navigation_csv}')
        
        self.get_logger().info('机器人状态节点已启动')
    
    def playback_navigation_data(self):
        """播放导航数据"""
        if (self.robot_state.navigation_data is None or 
            self.nav_data_index >= len(self.robot_state.navigation_data)):
            return
            
        timestamp = self.robot_state.navigation_data['timestamp'].iloc[self.nav_data_index]
        success = self.robot_state.update_from_navigation(timestamp)
        
        if success:
            self.get_logger().debug(f'播放导航数据: 索引 {self.nav_data_index}')
        
        self.nav_data_index += 1
        
        if self.nav_data_index >= len(self.robot_state.navigation_data):
            self.get_logger().info('导航数据播放完成')
            self.nav_playback_timer.cancel()
    
    def publish_state(self):
        """发布机器人状态"""
        stamp = self.get_clock().now().to_msg()
        
        # 创建里程计消息
        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame
        
        # 位置
        odom_msg.pose.pose.position.x = self.robot_state.x
        odom_msg.pose.pose.position.y = self.robot_state.y
        odom_msg.pose.pose.position.z = self.robot_state.z
        
        # 姿态
        q = Rotation.from_euler('xyz', [
            self.robot_state.roll, 
            self.robot_state.pitch, 
            self.robot_state.yaw
        ]).as_quat()
        
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]
        
        # 速度
        odom_msg.twist.twist.linear.x = self.robot_state.vx
        odom_msg.twist.twist.linear.y = self.robot_state.vy
        odom_msg.twist.twist.linear.z = self.robot_state.vz
        odom_msg.twist.twist.angular.x = self.robot_state.wx
        odom_msg.twist.twist.angular.y = self.robot_state.wy
        odom_msg.twist.twist.angular.z = self.robot_state.wz
        
        # 协方差
        odom_msg.pose.covariance = [0.1] * 36
        odom_msg.twist.covariance = [0.05] * 36
        
        # 发布里程计
        self.odometry_pub.publish(odom_msg)
        
        # 发布位姿
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header = odom_msg.header
        pose_msg.pose.pose = odom_msg.pose.pose
        pose_msg.pose.covariance = odom_msg.pose.covariance
        self.pose_pub.publish(pose_msg)
        
        # 发布TF
        if self.publish_tf:
            self.publish_transforms(stamp)
    
    def publish_transforms(self, stamp):
        """发布TF变换"""
        # odom -> base_link
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.base_frame
        
        transform.transform.translation.x = self.robot_state.x
        transform.transform.translation.y = self.robot_state.y
        transform.transform.translation.z = self.robot_state.z
        
        q = Rotation.from_euler('xyz', [
            self.robot_state.roll,
            self.robot_state.pitch, 
            self.robot_state.yaw
        ]).as_quat()
        
        transform.transform.rotation.x = q[0]
        transform.transform.rotation.y = q[1]
        transform.transform.rotation.z = q[2]
        transform.transform.rotation.w = q[3]
        
        self.tf_broadcaster.sendTransform(transform)

def main(args=None):
    rclpy.init(args=args)
    try:
        node = RobotStateNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
EOF

echo "📝 2. 部署 place_cell_node.py..."
cat > src/dolphin_slam/dolphin_slam/place_cell_node.py << 'EOF'
#!/usr/bin/env python3
"""
Dolphin SLAM - 位置细胞网络节点 (基于项目资料重建)
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import MarkerArray, Marker
import numpy as np
import time

class PlaceCellNode(Node):
    """位置细胞网络ROS2节点"""
    
    def __init__(self):
        super().__init__('place_cell_node')
        
        # 声明参数
        self.declare_parameters(
            namespace='',
            parameters=[
                ('neurons_per_dimension', 16),
                ('update_rate', 20.0),
                ('odometry_topic', '/dolphin_slam/odometry'),  # 修正后的话题
                ('visual_match_topic', '/local_view/matches'),
                ('activity_topic', '/place_cells/activity'),
            ]
        )
        
        # 获取参数
        self.neurons_per_dimension = self.get_parameter('neurons_per_dimension').value
        self.update_rate = self.get_parameter('update_rate').value
        self.odometry_topic = self.get_parameter('odometry_topic').value
        self.visual_match_topic = self.get_parameter('visual_match_topic').value
        self.activity_topic = self.get_parameter('activity_topic').value
        
        # 初始化网络
        total_neurons = self.neurons_per_dimension ** 3
        self.activity_data = np.random.random(total_neurons) * 0.1
        self.last_position = np.array([0.0, 0.0, 0.0])
        self.last_odometry = None
        self.update_count = 0
        
        # 订阅者
        self.odometry_sub = self.create_subscription(
            Odometry,
            self.odometry_topic,
            self.odometry_callback,
            10
        )
        
        self.visual_match_sub = self.create_subscription(
            Float32MultiArray,
            self.visual_match_topic,
            self.visual_match_callback,
            10
        )
        
        # 发布者
        self.activity_pub = self.create_publisher(
            Float32MultiArray,
            self.activity_topic,
            10
        )
        
        self.stats_pub = self.create_publisher(
            MarkerArray,
            '/place_cells/statistics',
            10
        )
        
        # 定时器
        self.update_timer = self.create_timer(
            1.0 / self.update_rate,
            self.update_network
        )
        
        self.viz_timer = self.create_timer(2.0, self.publish_statistics)
        
        self.get_logger().info(f'位置细胞网络节点已启动: {self.neurons_per_dimension}³ 神经元')
        self.get_logger().info(f'订阅里程计: {self.odometry_topic}')
    
    def odometry_callback(self, msg: Odometry):
        """处理里程计数据"""
        self.last_odometry = msg
        
        # 提取位置
        position = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])
        
        # 计算位置变化
        position_change = np.linalg.norm(position - self.last_position)
        
        # 更新位置细胞活动
        self.update_place_cells(position, position_change)
        self.last_position = position
        
        self.get_logger().debug(f'位置更新: {position}, 变化: {position_change:.3f}')
    
    def visual_match_callback(self, msg: Float32MultiArray):
        """处理视觉匹配数据"""
        if len(msg.data) > 0:
            similarity = msg.data[0] if msg.data[0] > 0 else 0.1
            
            # 视觉输入增强神经元活动
            enhancement = similarity * 0.2
            self.activity_data = np.clip(self.activity_data + enhancement, 0, 1)
            
            self.get_logger().debug(f'视觉匹配增强: {similarity:.3f}')
    
    def update_place_cells(self, position, movement):
        """更新位置细胞活动"""
        # 基于位置的空间编码
        for i in range(self.neurons_per_dimension):
            for j in range(self.neurons_per_dimension):
                for k in range(self.neurons_per_dimension):
                    # 计算神经元的空间中心
                    neuron_center = np.array([i, j, k]) * 2.0  # 2米间隔
                    
                    # 计算距离
                    distance = np.linalg.norm(position - neuron_center)
                    
                    # 高斯激活函数
                    sigma = 1.5  # 感受野大小
                    activation = np.exp(-distance**2 / (2 * sigma**2))
                    
                    # 运动调制
                    movement_modulation = 1.0 + movement * 0.5
                    
                    # 更新神经元索引
                    neuron_idx = i * self.neurons_per_dimension**2 + j * self.neurons_per_dimension + k
                    self.activity_data[neuron_idx] = activation * movement_modulation
    
    def update_network(self):
        """更新神经网络"""
        try:
            self.update_count += 1
            
            # 如果没有里程计数据，生成基础活动
            if self.last_odometry is None:
                t = time.time()
                for i in range(len(self.activity_data)):
                    self.activity_data[i] = 0.1 + 0.05 * np.sin(t * 0.5 + i * 0.1)
            
            # 应用衰减
            self.activity_data *= 0.98
            
            # 添加噪声
            self.activity_data += np.random.random(len(self.activity_data)) * 0.005
            
            # 归一化
            max_activity = np.max(self.activity_data)
            if max_activity > 1.0:
                self.activity_data /= max_activity
            
            # 发布活动数据
            activity_msg = Float32MultiArray()
            activity_msg.data = self.activity_data.tolist()
            self.activity_pub.publish(activity_msg)
            
            # 定期报告
            if self.update_count % 100 == 0:
                active_neurons = np.sum(self.activity_data > 0.1)
                max_activity = np.max(self.activity_data)
                self.get_logger().info(
                    f'网络状态: {active_neurons}/{len(self.activity_data)} 神经元活跃, '
                    f'最大活动: {max_activity:.3f}'
                )
                
        except Exception as e:
            self.get_logger().error(f'网络更新错误: {e}')
    
    def publish_statistics(self):
        """发布统计信息"""
        if len(self.activity_data) == 0:
            return
            
        marker_array = MarkerArray()
        
        # 创建统计文本
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "place_cell_stats"
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        
        marker.pose.position.x = 5.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 5.0
        marker.pose.orientation.w = 1.0
        
        marker.scale.z = 0.5
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        
        # 计算统计数据
        peak_activity = np.max(self.activity_data)
        mean_activity = np.mean(self.activity_data)
        active_neurons = np.sum(self.activity_data > 0.1)
        total_neurons = len(self.activity_data)
        
        marker.text = (
            f"位置细胞网络统计\\n"
            f"================\\n"
            f"网络规模: {self.neurons_per_dimension}³\\n"
            f"总神经元: {total_neurons}\\n"
            f"活跃神经元: {active_neurons}\\n"
            f"峰值活动: {peak_activity:.3f}\\n"
            f"平均活动: {mean_activity:.3f}\\n"
            f"更新次数: {self.update_count}"
        )
        
        marker_array.markers.append(marker)
        self.stats_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    try:
        node = PlaceCellNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
EOF

echo "📝 3. 部署 experience_map_node.py..."
cat > src/dolphin_slam/dolphin_slam/experience_map_node.py << 'EOF'
#!/usr/bin/env python3
"""
Dolphin SLAM - 经验地图节点 (基于项目资料重建)
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray, Marker
import numpy as np
import time

class ExperienceMapNode(Node):
    """经验地图ROS2节点"""
    
    def __init__(self):
        super().__init__('experience_map_node')
        
        # 声明参数
        self.declare_parameters(
            namespace='',
            parameters=[
                ('odometry_topic', '/dolphin_slam/odometry'),  # 修正后的话题
                ('place_cell_topic', '/place_cells/activity'),
                ('visual_match_topic', '/local_view/matches'),
            ]
        )
        
        # 获取参数
        self.odometry_topic = self.get_parameter('odometry_topic').value
        self.place_cell_topic = self.get_parameter('place_cell_topic').value
        self.visual_match_topic = self.get_parameter('visual_match_topic').value
        
        # 状态变量
        self.current_odometry = None
        self.trajectory_poses = []
        self.place_cell_activity = None
        self.visual_matches = None
        self.experience_count = 0
        self.message_count = 0
        
        # 订阅者
        self.odometry_sub = self.create_subscription(
            Odometry,
            self.odometry_topic,
            self.odometry_callback,
            10
        )
        
        self.place_cell_sub = self.create_subscription(
            Float32MultiArray,
            self.place_cell_topic,
            self.place_cell_callback,
            10
        )
        
        self.visual_match_sub = self.create_subscription(
            Float32MultiArray,
            self.visual_match_topic,
            self.visual_match_callback,
            10
        )
        
        # 发布者
        self.trajectory_pub = self.create_publisher(
            Path,
            '/dolphin_slam/trajectory',
            10
        )
        
        self.experience_pub = self.create_publisher(
            Float32MultiArray,
            '/experience_map/experiences',
            10
        )
        
        self.markers_pub = self.create_publisher(
            MarkerArray,
            '/experience_map/markers',
            10
        )
        
        # 定时器
        self.update_timer = self.create_timer(0.1, self.update_and_publish)
        self.viz_timer = self.create_timer(1.0, self.publish_visualizations)
        
        self.get_logger().info('经验地图节点已启动')
        self.get_logger().info(f'订阅里程计: {self.odometry_topic}')
    
    def odometry_callback(self, msg: Odometry):
        """处理里程计数据并构建轨迹"""
        self.current_odometry = msg
        
        # 创建轨迹点
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose = msg.pose.pose
        
        self.trajectory_poses.append(pose_stamped)
        
        # 限制轨迹长度
        if len(self.trajectory_poses) > 2000:
            self.trajectory_poses = self.trajectory_poses[-1500:]
        
        self.get_logger().debug(f'轨迹点数: {len(self.trajectory_poses)}')
    
    def place_cell_callback(self, msg: Float32MultiArray):
        """处理位置细胞活动"""
        self.place_cell_activity = msg.data
        self.get_logger().debug(f'位置细胞数据: {len(msg.data)} 个神经元')
    
    def visual_match_callback(self, msg: Float32MultiArray):
        """处理视觉匹配数据"""
        self.visual_matches = msg.data
        self.get_logger().debug(f'视觉匹配数据: {len(msg.data)} 个匹配')
    
    def update_and_publish(self):
        """更新经验地图并发布轨迹"""
        # 发布轨迹
        if self.trajectory_poses:
            path_msg = Path()
            path_msg.header.stamp = self.get_clock().now().to_msg()
            path_msg.header.frame_id = "map"
            path_msg.poses = self.trajectory_poses.copy()
            
            self.trajectory_pub.publish(path_msg)
            
            self.message_count += 1
            if self.message_count % 100 == 0:
                self.get_logger().info(f'轨迹发布: {len(self.trajectory_poses)} 个点')
        
        # 发布经验数据
        if self.current_odometry:
            self.publish_experience_data()
    
    def publish_experience_data(self):
        """发布经验数据"""
        experience_msg = Float32MultiArray()
        
        # 构建经验数据
        data = [
            self.current_odometry.pose.pose.position.x,
            self.current_odometry.pose.pose.position.y,
            self.current_odometry.pose.pose.position.z,
            len(self.trajectory_poses),  # 轨迹长度
        ]
        
        # 添加位置细胞信息
        if self.place_cell_activity:
            max_activity = max(self.place_cell_activity) if self.place_cell_activity else 0.0
            active_count = sum(1 for a in self.place_cell_activity if a > 0.1)
            data.extend([max_activity, active_count])
        else:
            data.extend([0.0, 0.0])
        
        # 添加视觉匹配信息
        if self.visual_matches:
            match_strength = sum(self.visual_matches) if self.visual_matches else 0.0
            data.append(match_strength)
        else:
            data.append(0.0)
        
        experience_msg.data = data
        self.experience_pub.publish(experience_msg)
        
        self.experience_count += 1
    
    def publish_visualizations(self):
        """发布可视化标记"""
        if not self.trajectory_poses:
            return
            
        marker_array = MarkerArray()
        
        # 轨迹线标记
        trajectory_marker = Marker()
        trajectory_marker.header.frame_id = "map"
        trajectory_marker.header.stamp = self.get_clock().now().to_msg()
        trajectory_marker.ns = "trajectory"
        trajectory_marker.id = 0
        trajectory_marker.type = Marker.LINE_STRIP
        trajectory_marker.action = Marker.ADD
        
        trajectory_marker.scale.x = 0.1  # 线宽
        trajectory_marker.color.r = 0.0
        trajectory_marker.color.g = 1.0
        trajectory_marker.color.b = 0.0
        trajectory_marker.color.a = 1.0
        
        # 添加轨迹点
        for pose in self.trajectory_poses[::10]:  # 每10个点取一个，减少计算量
            trajectory_marker.points.append(pose.pose.position)
        
        marker_array.markers.append(trajectory_marker)
        
        # 当前位置标记
        if self.current_odometry:
            current_pos_marker = Marker()
            current_pos_marker.header.frame_id = "map"
            current_pos_marker.header.stamp = self.get_clock().now().to_msg()
            current_pos_marker.ns = "current_position"
            current_pos_marker.id = 1
            current_pos_marker.type = Marker.SPHERE
            current_pos_marker.action = Marker.ADD
            
            current_pos_marker.pose = self.current_odometry.pose.pose
            current_pos_marker.scale.x = 0.5
            current_pos_marker.scale.y = 0.5
            current_pos_marker.scale.z = 0.5
            current_pos_marker.color.r = 1.0
            current_pos_marker.color.g = 0.0
            current_pos_marker.color.b = 0.0
            current_pos_marker.color.a = 1.0
            
            marker_array.markers.append(current_pos_marker)
        
        # 统计信息文本
        stats_marker = Marker()
        stats_marker.header.frame_id = "map"
        stats_marker.header.stamp = self.get_clock().now().to_msg()
        stats_marker.ns = "statistics"
        stats_marker.id = 2
        stats_marker.type = Marker.TEXT_VIEW_FACING
        stats_marker.action = Marker.ADD
        
        stats_marker.pose.position.x = -5.0
        stats_marker.pose.position.y = 0.0
        stats_marker.pose.position.z = 5.0
        stats_marker.pose.orientation.w = 1.0
        
        stats_marker.scale.z = 0.5
        stats_marker.color.r = 1.0
        stats_marker.color.g = 1.0
        stats_marker.color.b = 1.0
        stats_marker.color.a = 1.0
        
        # 统计信息
        active_pc = 0
        max_pc_activity = 0.0
        if self.place_cell_activity:
            active_pc = sum(1 for a in self.place_cell_activity if a > 0.1)
            max_pc_activity = max(self.place_cell_activity)
        
        visual_strength = 0.0
        if self.visual_matches:
            visual_strength = sum(self.visual_matches)
        
        stats_marker.text = (
            f"经验地图统计\\n"
            f"==============\\n"
            f"轨迹点数: {len(self.trajectory_poses)}\\n"
            f"经验总数: {self.experience_count}\\n"
            f"活跃位置细胞: {active_pc}\\n"
            f"最大PC活动: {max_pc_activity:.3f}\\n"
            f"视觉匹配强度: {visual_strength:.3f}"
        )
        
        marker_array.markers.append(stats_marker)
        
        self.markers_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    try:
        node = ExperienceMapNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
EOF

echo "📝 4. 部署修复版配置文件..."
cat > src/dolphin_slam/config/dolphin_slam_params.yaml << 'EOF'
# 修复版 Dolphin SLAM 配置文件
# 统一话题命名，确保节点间正确通信

robot_state_node:
  ros__parameters:
    navigation_csv: "/media/psf/Samsung T7/SLAM Data/Sunboat_03-09-2023/2023-09-03-07-58-37/navigation/navigation.csv"
    base_frame: "base_link"
    odom_frame: "odom"
    map_frame: "map"
    publish_rate: 20.0
    publish_tf: true

image_processing_node:
  ros__parameters:
    camera_path: "/media/psf/Samsung T7/SLAM Data/Sunboat_03-09-2023/2023-09-03-07-58-37/camera"
    sonar_path: "/media/psf/Samsung T7/SLAM Data/Sunboat_03-09-2023/2023-09-03-07-58-37/sonar"
    feature_type: "SIFT"
    max_features: 500
    enable_visualization: true
    camera_topic: "/camera/image_raw"
    sonar_topic: "/sonar/image_raw"
    descriptors_topic: "/features/descriptors"
    keypoints_topic: "/features/keypoints"

local_view_node:
  ros__parameters:
    matching_algorithm: "fabmap"
    similarity_threshold: 0.65
    vocabulary_size: 500
    descriptors_topic: "/features/descriptors"
    matches_topic: "/local_view/matches"

place_cell_node:
  ros__parameters:
    neurons_per_dimension: 16
    update_rate: 20.0
    odometry_topic: "/dolphin_slam/odometry"    # 修正：统一使用dolphin_slam命名空间
    visual_match_topic: "/local_view/matches"
    activity_topic: "/place_cells/activity"

experience_map_node:
  ros__parameters:
    odometry_topic: "/dolphin_slam/odometry"    # 修正：统一使用dolphin_slam命名空间
    place_cell_topic: "/place_cells/activity"
    visual_match_topic: "/local_view/matches"
    match_threshold: 0.75
EOF

# 设置执行权限
chmod +x src/dolphin_slam/dolphin_slam/*.py

echo "🔨 5. 重新构建项目..."
colcon build --packages-select dolphin_slam --symlink-install

if [ $? -eq 0 ]; then
    echo "✅ 构建成功"
    
    echo "🔄 6. 重新加载环境..."
    source install/setup.bash
    export ROS_DOMAIN_ID=42
    
    echo ""
    echo "🎉 部署完成！基于项目资料的可工作节点已部署"
    echo "============================================="
    echo ""
    echo "🔧 关键修复内容:"
    echo "   ✅ robot_state_node: 发布到 /dolphin_slam/odometry"
    echo "   ✅ place_cell_node: 订阅 /dolphin_slam/odometry"
    echo "   ✅ experience_map_node: 发布轨迹到 /dolphin_slam/trajectory"
    echo "   ✅ 配置文件: 统一话题命名规范"
    echo ""
    echo "🚀 启动系统:"
    echo "ros2 launch dolphin_slam dolphin_slam_enhanced_launch.py \\"
    echo "    dataset_path:=\"/media/psf/Samsung T7/SLAM Data/Sunboat_03-09-2023/2023-09-03-07-58-37\" \\"
    echo "    enable_rviz:=false"
    echo ""
    echo "🎯 预期结果:"
    echo "   ✅ /dolphin_slam/odometry: ~20 Hz"
    echo "   ✅ /dolphin_slam/trajectory: ~10 Hz" 
    echo "   ✅ /place_cells/activity: ~20 Hz"
    echo "   ✅ /local_view/matches: ~1.7 Hz (保持现有水平)"
    echo ""
    echo "📊 监控系统状态:"
    echo "# 在另一个终端运行:"
    echo "export ROS_DOMAIN_ID=42"
    echo "cd ~/dolphin_slam_ws && source install/setup.bash"
    echo "timeout 5 ros2 topic hz /dolphin_slam/odometry"
    echo "timeout 5 ros2 topic hz /dolphin_slam/trajectory"
    echo "timeout 5 ros2 topic hz /place_cells/activity"
    echo "timeout 5 ros2 topic hz /local_view/matches"
    
else
    echo "❌ 构建失败，请检查错误信息"
    exit 1
fi
