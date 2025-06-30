#!/usr/bin/env python3
"""
Dolphin SLAM - 经验地图 ROS2 节点（完全重写版本）
只使用 Odometry 消息类型，避免任何冲突
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
    """经验地图 ROS2 节点"""
    
    def __init__(self):
        super().__init__('experience_map_node')
        
        # 声明参数
        self.declare_parameters(
            namespace='',
            parameters=[
                ('place_cell_topic', '/place_cells/activity'),
                ('local_view_topic', '/local_view/matches'),
                ('experience_topic', '/experience_map/experiences'),
                ('odometry_topic', '/robot/odometry'),
            ]
        )
        
        # 状态变量
        self.current_odometry: Optional[Odometry] = None
        self.current_visual_similarity = 0.0
        self.experience_count = 0
        
        # 订阅者 - 只使用 Odometry，绝不使用 PoseWithCovarianceStamped
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
            '/robot/odometry',
            self.odometry_callback,
            10
        )
        
        # 发布者
        self.experience_pub = self.create_publisher(
            Float32MultiArray,
            self.get_parameter('experience_topic').value,
            10
        )
        
        self.map_markers_pub = self.create_publisher(
            MarkerArray,
            '/experience_map/markers',
            10
        )
        
        self.trajectory_pub = self.create_publisher(
            Path,
            '/dolphin_slam/trajectory',
            10
        )
        
        # 定时器
        self.update_timer = self.create_timer(0.1, self.update_map)
        self.viz_timer = self.create_timer(1.0, self.publish_visualizations)
        
        self.get_logger().info('经验地图节点已启动')
        
    def odometry_callback(self, msg: Odometry):
        """处理里程计数据 - 只接受 Odometry 类型"""
        self.current_odometry = msg
        
    def place_cell_callback(self, msg: Float32MultiArray):
        """处理位置细胞活动"""
        if len(msg.data) > 0:
            activity = np.array(msg.data)
            peak_idx = np.argmax(activity)
            
    def visual_match_callback(self, msg: Float32MultiArray):
        """处理视觉匹配"""
        if len(msg.data) > 0:
            self.current_visual_similarity = msg.data[0]
            
    def update_map(self):
        """更新经验地图"""
        if self.current_odometry is None:
            return
            
        try:
            # 获取当前位置
            position = np.array([
                self.current_odometry.pose.pose.position.x,
                self.current_odometry.pose.pose.position.y,
                self.current_odometry.pose.pose.position.z
            ])
            
            # 简单的经验创建逻辑
            self.experience_count += 1
            
        except Exception as e:
            self.get_logger().error(f'地图更新失败: {e}')
    
    def publish_visualizations(self):
        """发布可视化"""
        try:
            markers = MarkerArray()
            
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "experience_map"
            marker.id = 0
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            
            marker.pose.position.x = 0.0
            marker.pose.position.y = 0.0
            marker.pose.position.z = 3.0
            marker.pose.orientation.w = 1.0
            
            marker.scale.z = 0.5
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            
            marker.text = f"Experiences: {self.experience_count}"
            
            markers.markers.append(marker)
            self.map_markers_pub.publish(markers)
            
        except Exception as e:
            self.get_logger().debug(f'可视化错误: {e}')

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
