#!/usr/bin/env python3
"""
Dolphin SLAM - 经验地图 ROS2 节点
构建和维护拓扑-度量混合地图
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import Header, Float32MultiArray, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
from typing import Optional, List, Tuple
import os

# 导入核心模块
from dolphin_slam.experience_map import ExperienceMap, Experience

# 导入自定义消息（需要先生成）
# from dolphin_slam.msg import ExperienceEvent, PlaceCellActivity, LocalViewMatch

class ExperienceMapNode(Node):
    """经验地图 ROS2 节点"""
    
    def __init__(self):
        super().__init__('experience_map_node')
        
        # 声明参数
        self.declare_parameters(
            namespace='',
            parameters=[
                ('match_threshold', 0.75),
                ('lv_factor', 0.5),
                ('pc_factor', 0.5),
                ('min_experience_age', 5),
                ('loop_closure_threshold', 0.8),
                ('experience_topic', '/experience_map/experiences'),
                ('loop_closure_topic', '/experience_map/loop_closures'),
                ('map_save_path', ''),
                ('auto_save_interval', 50),  # 每N个经验自动保存
            ]
        )
        
        # 获取参数
        self.match_threshold = self.get_parameter('match_threshold').value
        self.lv_factor = self.get_parameter('lv_factor').value
        self.pc_factor = self.get_parameter('pc_factor').value
        self.min_experience_age = self.get_parameter('min_experience_age').value
        self.loop_closure_threshold = self.get_parameter('loop_closure_threshold').value
        self.map_save_path = self.get_parameter('map_save_path').value
        self.auto_save_interval = self.get_parameter('auto_save_interval').value
        
        # 初始化经验地图
        self.experience_map = ExperienceMap(
            match_threshold=self.match_threshold,
            lv_factor=self.lv_factor,
            pc_factor=self.pc_factor,
            min_experience_age=self.min_experience_age,
            loop_closure_threshold=self.loop_closure_threshold
        )
        
        # 状态变量
        self.current_pose = None
        self.current_visual_template = None
        self.current_visual_similarity = 0.0
        self.current_pc_center = None
        self.last_experience_creation_time = None
        
        # 订阅者
        # 位置细胞活动
        self.pc_activity_sub = self.create_subscription(
            Float32MultiArray,  # 临时使用，应该是 PlaceCellActivity
            '/place_cells/activity',
            self.place_cell_callback,
            10
        )
        
        # 视觉匹配
        self.visual_match_sub = self.create_subscription(
            Float32MultiArray,  # 临时使用，应该是 LocalViewMatch
            '/local_view/matches',
            self.visual_match_callback,
            10
        )
        
        # 机器人位姿
        self.odometry_sub = self.create_subscription(
            Odometry,
            '/robot/odometry',
            self.odometry_callback,
            10
        )
        
        # 也支持 PoseWithCovarianceStamped
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/robot/odometry',
            self.pose_callback,
            10
        )
        
        # 发布者
        # 经验事件
        self.experience_pub = self.create_publisher(
            Float32MultiArray,  # 临时使用，应该是 ExperienceEvent
            self.get_parameter('experience_topic').value,
            10
        )
        
        # 闭环检测
        self.loop_closure_pub = self.create_publisher(
            Float32MultiArray,
            self.get_parameter('loop_closure_topic').value,
            10
        )
        
        # 地图可视化
        self.map_markers_pub = self.create_publisher(
            MarkerArray,
            '/experience_map/markers',
            10
        )
        
        # 机器人轨迹
        self.trajectory_pub = self.create_publisher(
            Path,
            '/dolphin_slam/trajectory',
            10
        )
        
        # 统计信息
        self.stats_pub = self.create_publisher(
            MarkerArray,
            '/experience_map/statistics',
            10
        )
        
        # 服务
        self.save_map_srv = self.create_service(
            'std_srvs/srv/Trigger',
            '/dolphin_slam/save_map',
            self.save_map_callback
        )
        
        self.load_map_srv = self.create_service(
            'std_srvs/srv/Trigger',
            '/dolphin_slam/load_map',
            self.load_map_callback
        )
        
        # 定时器
        self.update_timer = self.create_timer(0.1, self.update_map)  # 10 Hz
        self.viz_timer = self.create_timer(0.5, self.publish_visualizations)
        self.stats_timer = self.create_timer(2.0, self.publish_statistics)
        
        # 轨迹路径
        self.trajectory_path = Path()
        self.trajectory_path.header.frame_id = "map"
        
        self.get_logger().info('经验地图节点已启动')
        
    def place_cell_callback(self, msg: Float32MultiArray):
        """处理位置细胞活动"""
        if len(msg.data) >= 10:
            # 提取质心位置
            self.current_pc_center = (
                msg.data[7],  # center_x
                msg.data[8],  # center_y
                msg.data[9]   # center_z
            )
            
    def visual_match_callback(self, msg: Float32MultiArray):
        """处理视觉匹配结果"""
        if len(msg.data) >= 2:
            self.current_visual_template = int(msg.data[0])
            self.current_visual_similarity = msg.data[1]
            
    def odometry_callback(self, msg: Odometry):
        """处理里程计数据"""
        self.current_pose = msg.pose.pose
        
        # 更新轨迹
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose = msg.pose.pose
        self.trajectory_path.poses.append(pose_stamped)
        
    def pose_callback(self, msg: PoseWithCovarianceStamped):
        """处理位姿数据"""
        self.current_pose = msg.pose.pose
        
        # 更新轨迹
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose = msg.pose.pose
        self.trajectory_path.poses.append(pose_stamped)
        
    def update_map(self):
        """更新地图（主循环）"""
        # 检查是否有足够的信息
        if (self.current_pose is None or 
            self.current_visual_template is None or
            self.current_pc_center is None):
            return
            
        # 获取当前位置
        x = self.current_pose.position.x
        y = self.current_pose.position.y
        z = self.current_pose.position.z
        
        # 从四元数计算偏航角
        q = self.current_pose.orientation
        theta = np.arctan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        # 查找匹配的经验
        matched_exp_id = self.experience_map.find_match(
            self.current_visual_template,
            self.current_visual_similarity,
            self.current_pc_center,
            (x, y, z)
        )
        
        if matched_exp_id is not None:
            # 更新现有经验
            self.experience_map.update_experience(
                matched_exp_id, x, y, z, theta, current_time
            )
            
            # 切换到该经验
            self.experience_map.current_experience_id = matched_exp_id
            
            # 发布经验更新事件
            self.publish_experience_event(matched_exp_id, 'update')
            
            self.get_logger().debug(f'更新经验 #{matched_exp_id}')
            
            # 检测闭环
            loop_closures = self.experience_map.detect_loop_closure(matched_exp_id)
            if loop_closures:
                for loop_exp_id, similarity in loop_closures:
                    self.publish_loop_closure(matched_exp_id, loop_exp_id, similarity)
                    
        else:
            # 创建新经验
            new_exp_id = self.experience_map.create_experience(
                x, y, z, theta,
                self.current_visual_template,
                self.current_pc_center,
                current_time
            )
            
            # 发布经验创建事件
            self.publish_experience_event(new_exp_id, 'create')
            
            self.get_logger().info(f'创建新经验 #{new_exp_id} at ({x:.2f}, {y:.2f}, {z:.2f})')
            
            # 自动保存
            if new_exp_id % self.auto_save_interval == 0 and self.map_save_path:
                self.save_map()
                
    def publish_experience_event(self, exp_id: int, action: str):
        """发布经验事件"""
        msg = Float32MultiArray()
        
        if exp_id in self.experience_map.experiences:
            exp = self.experience_map.experiences[exp_id]
            
            action_code = {'create': 0, 'update': 1, 'loop': 2}.get(action, 0)
            
            msg.data = [
                float(exp_id),
                float(action_code),
                exp.x,
                exp.y,
                exp.z,
                exp.theta,
                float(exp.visual_template_id),
                1.0  # confidence
            ]
            
            self.experience_pub.publish(msg)
            
    def publish_loop_closure(self, exp1_id: int, exp2_id: int, similarity: float):
        """发布闭环检测结果"""
        msg = Float32MultiArray()
        msg.data = [
            float(exp1_id),
            float(exp2_id),
            similarity
        ]
        
        self.loop_closure_pub.publish(msg)
        
        self.get_logger().info(
            f'闭环检测: 经验 #{exp1_id} <-> #{exp2_id}, 相似度: {similarity:.3f}'
        )
        
    def publish_visualizations(self):
        """发布可视化数据"""
        marker_array = MarkerArray()
        
        # 清除旧标记
        clear_marker = Marker()
        clear_marker.header.frame_id = "map"
        clear_marker.header.stamp = self.get_clock().now().to_msg()
        clear_marker.ns = "experiences"
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)
        
        # 经验节点
        for exp_id, exp in self.experience_map.experiences.items():
            # 节点标记
            node_marker = Marker()
            node_marker.header.frame_id = "map"
            node_marker.header.stamp = self.get_clock().now().to_msg()
            node_marker.ns = "experiences"
            node_marker.id = exp_id
            node_marker.type = Marker.SPHERE
            node_marker.action = Marker.ADD
            
            node_marker.pose.position.x = exp.x
            node_marker.pose.position.y = exp.y
            node_marker.pose.position.z = exp.z
            node_marker.pose.orientation.w = 1.0
            
            # 当前经验用不同颜色
            if exp_id == self.experience_map.current_experience_id:
                node_marker.scale.x = 0.4
                node_marker.scale.y = 0.4
                node_marker.scale.z = 0.4
                node_marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
            else:
                node_marker.scale.x = 0.2
                node_marker.scale.y = 0.2
                node_marker.scale.z = 0.2
                node_marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8)
                
            marker_array.markers.append(node_marker)
            
            # 文本标签
            text_marker = Marker()
            text_marker.header = node_marker.header
            text_marker.ns = "experience_labels"
            text_marker.id = exp_id
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            
            text_marker.pose.position.x = exp.x
            text_marker.pose.position.y = exp.y
            text_marker.pose.position.z = exp.z + 0.5
            text_marker.pose.orientation.w = 1.0
            
            text_marker.scale.z = 0.2
            text_marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            text_marker.text = f"E{exp_id}"
            
            marker_array.markers.append(text_marker)
            
        # 连接
        connection_id = 0
        for exp1_id, neighbors in self.experience_map.connections.items():
            exp1 = self.experience_map.experiences[exp1_id]
            
            for exp2_id in neighbors:
                if exp2_id > exp1_id:  # 避免重复
                    exp2 = self.experience_map.experiences[exp2_id]
                    
                    line_marker = Marker()
                    line_marker.header.frame_id = "map"
                    line_marker.header.stamp = self.get_clock().now().to_msg()
                    line_marker.ns = "connections"
                    line_marker.id = connection_id
                    line_marker.type = Marker.LINE_STRIP
                    line_marker.action = Marker.ADD
                    
                    # 起点
                    p1 = PoseStamped()
                    p1.pose.position.x = exp1.x
                    p1.pose.position.y = exp1.y
                    p1.pose.position.z = exp1.z
                    
                    # 终点
                    p2 = PoseStamped()
                    p2.pose.position.x = exp2.x
                    p2.pose.position.y = exp2.y
                    p2.pose.position.z = exp2.z
                    
                    line_marker.points = [p1.pose.position, p2.pose.position]
                    
                    line_marker.scale.x = 0.05  # 线宽
                    line_marker.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.5)
                    
                    marker_array.markers.append(line_marker)
                    connection_id += 1
                    
        self.map_markers_pub.publish(marker_array)
        
        # 发布轨迹
        self.trajectory_path.header.stamp = self.get_clock().now().to_msg()
        self.trajectory_pub.publish(self.trajectory_path)
        
    def publish_statistics(self):
        """发布统计信息"""
        num_experiences = len(self.experience_map.experiences)
        num_connections = sum(len(neighbors) for neighbors in self.experience_map.connections.values()) // 2
        
        marker_array = MarkerArray()
        
        # 创建文本标记
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "statistics"
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        
        marker.pose.position.x = -5.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 5.0
        marker.pose.orientation.w = 1.0
        
        marker.scale.z = 0.5
        
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        
        # 统计文本
        marker.text = (
            f"经验地图统计\n"
            f"==============\n"
            f"经验总数: {num_experiences}\n"
            f"连接总数: {num_connections}\n"
            f"闭环检测: {self.experience_map.loop_closures_detected}\n"
            f"总距离: {self.experience_map.total_distance_traveled:.2f}m\n"
            f"当前经验: #{self.experience_map.current_experience_id}"
        )
        
        marker_array.markers.append(marker)
        self.stats_pub.publish(marker_array)
        
        # 输出到日志
        self.get_logger().info(
            f"EM 统计 - 经验: {num_experiences}, 闭环: {self.experience_map.loop_closures_detected}"
        )
        
    def save_map_callback(self, request, response):
        """保存地图服务回调"""
        try:
            if self.map_save_path:
                filename = os.path.join(
                    self.map_save_path,
                    f'experience_map_{self.get_clock().now().nanoseconds}.pkl'
                )
                self.experience_map.save_map(filename)
                response.success = True
                response.message = f"地图已保存到 {filename}"
            else:
                response.success = False
                response.message = "地图保存路径未设置"
        except Exception as e:
            response.success = False
            response.message = f"保存失败: {e}"
            
        return response
        
    def load_map_callback(self, request, response):
        """加载地图服务回调"""
        try:
            # 简化实现：加载最新的地图文件
            if self.map_save_path and os.path.exists(self.map_save_path):
                map_files = [f for f in os.listdir(self.map_save_path) 
                           if f.startswith('experience_map_') and f.endswith('.pkl')]
                if map_files:
                    latest_file = sorted(map_files)[-1]
                    filename = os.path.join(self.map_save_path, latest_file)
                    self.experience_map.load_map(filename)
                    response.success = True
                    response.message = f"地图已从 {filename} 加载"
                else:
                    response.success = False
                    response.message = "未找到地图文件"
            else:
                response.success = False
                response.message = "地图路径不存在"
        except Exception as e:
            response.success = False
            response.message = f"加载失败: {e}"
            
        return response
        
    def save_map(self):
        """自动保存地图"""
        try:
            if self.map_save_path:
                os.makedirs(self.map_save_path, exist_ok=True)
                filename = os.path.join(
                    self.map_save_path,
                    f'experience_map_auto_{len(self.experience_map.experiences)}.pkl'
                )
                self.experience_map.save_map(filename)
                self.get_logger().info(f"地图自动保存到 {filename}")
        except Exception as e:
            self.get_logger().error(f"自动保存失败: {e}")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = ExperienceMapNode()
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
