#!/usr/bin/env python3
"""
Dolphin SLAM 监控脚本 - 解决话题冲突版本
处理多种消息类型和话题冲突
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Float32MultiArray
import numpy as np
import time
import os
import matplotlib.pyplot as plt
from collections import deque
import threading

class ConflictFreeMonitor(Node):
    def __init__(self):
        super().__init__('conflict_free_monitor')
        
        # 数据存储
        self.trajectory_points = deque(maxlen=2000)
        self.current_position = None
        self.place_cell_activity = None
        self.visual_matches = None
        
        # 统计信息
        self.start_time = time.time()
        self.message_counts = {
            'trajectory': 0,
            'odometry_nav': 0,
            'odometry_pose': 0, 
            'place_cells': 0,
            'visual_matches': 0
        }
        
        # 订阅者 - 处理多种消息类型
        
        # 轨迹数据
        self.trajectory_sub = self.create_subscription(
            Path,
            '/dolphin_slam/trajectory',
            self.trajectory_callback,
            10
        )
        
        # 里程计数据 - nav_msgs/Odometry 类型
        self.odometry_nav_sub = self.create_subscription(
            Odometry,
            '/robot/odometry',
            self.odometry_nav_callback,
            10
        )
        
        # 位姿数据 - geometry_msgs/PoseWithCovarianceStamped 类型
        self.odometry_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/robot/odometry',  # 同一个话题，不同类型
            self.odometry_pose_callback,
            10
        )
        
        # 尝试其他可能的里程计话题
        self.robot_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/robot/pose',
            self.robot_pose_callback,
            10
        )
        
        # 位置细胞活动
        self.place_cell_sub = self.create_subscription(
            Float32MultiArray,
            '/place_cells/activity',
            self.place_cell_callback,
            10
        )
        
        # 视觉匹配
        self.visual_match_sub = self.create_subscription(
            Float32MultiArray,
            '/local_view/matches',
            self.visual_match_callback,
            10
        )
        
        # 定时器
        self.status_timer = self.create_timer(3.0, self.print_status)
        
        # matplotlib设置
        self.setup_plot()
        
        self.get_logger().info('🚀 冲突解决版监控器启动!')
        self.get_logger().info('📡 监听所有可能的话题类型')
        
    def setup_plot(self):
        """设置matplotlib绘图"""
        plt.ion()
        self.fig, (self.ax1, self.ax2) = plt.subplots(1, 2, figsize=(15, 6))
        
        self.ax1.set_title('🐬 Dolphin SLAM 轨迹追踪', fontsize=14)
        self.ax1.set_xlabel('X (m)')
        self.ax1.set_ylabel('Y (m)')
        self.ax1.grid(True, alpha=0.3)
        self.ax1.axis('equal')
        
        self.ax2.set_title('📊 系统活动监控', fontsize=14)
        self.ax2.set_xlabel('组件')
        self.ax2.set_ylabel('消息数量')
        
        # 轨迹绘图
        self.trajectory_line, = self.ax1.plot([], [], 'g-', linewidth=3, label='AUV轨迹', alpha=0.8)
        self.current_pos, = self.ax1.plot([], [], 'ro', markersize=12, label='当前位置', zorder=5)
        self.start_pos, = self.ax1.plot([], [], 'g^', markersize=12, label='起始点', zorder=5)
        
        self.ax1.legend()
        plt.tight_layout()
        plt.show()
        
    def trajectory_callback(self, msg: Path):
        """处理轨迹数据"""
        self.message_counts['trajectory'] += 1
        
        if msg.poses:
            self.trajectory_points.clear()
            for pose in msg.poses:
                x = pose.pose.position.x
                y = pose.pose.position.y
                self.trajectory_points.append((x, y))
            self.update_trajectory_plot()
            
    def odometry_nav_callback(self, msg: Odometry):
        """处理 nav_msgs/Odometry 类型的里程计数据"""
        self.message_counts['odometry_nav'] += 1
        self.current_position = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'z': msg.pose.pose.position.z,
            'source': 'nav_msgs/Odometry'
        }
        self.update_current_position()
        
    def odometry_pose_callback(self, msg: PoseWithCovarianceStamped):
        """处理 PoseWithCovarianceStamped 类型的里程计数据"""
        self.message_counts['odometry_pose'] += 1
        if self.current_position is None:  # 优先使用nav_msgs/Odometry
            self.current_position = {
                'x': msg.pose.pose.position.x,
                'y': msg.pose.pose.position.y, 
                'z': msg.pose.pose.position.z,
                'source': 'PoseWithCovarianceStamped'
            }
            self.update_current_position()
            
    def robot_pose_callback(self, msg: PoseWithCovarianceStamped):
        """处理 /robot/pose 话题"""
        if self.current_position is None:  # 作为备用
            self.current_position = {
                'x': msg.pose.pose.position.x,
                'y': msg.pose.pose.position.y,
                'z': msg.pose.pose.position.z, 
                'source': '/robot/pose'
            }
            self.update_current_position()
        
    def place_cell_callback(self, msg: Float32MultiArray):
        """处理位置细胞活动"""
        self.message_counts['place_cells'] += 1
        self.place_cell_activity = msg.data
        
    def visual_match_callback(self, msg: Float32MultiArray):
        """处理视觉匹配"""
        self.message_counts['visual_matches'] += 1
        self.visual_matches = msg.data
        
    def update_current_position(self):
        """更新当前位置显示"""
        if self.current_position:
            x, y = self.current_position['x'], self.current_position['y']
            self.current_pos.set_data([x], [y])
            
            # 如果没有轨迹，从当前位置开始
            if len(self.trajectory_points) == 0:
                self.trajectory_points.append((x, y))
                self.start_pos.set_data([x], [y])
                self.update_trajectory_plot()
        
    def update_trajectory_plot(self):
        """更新轨迹绘图"""
        if self.trajectory_points:
            x_data = [point[0] for point in self.trajectory_points]
            y_data = [point[1] for point in self.trajectory_points]
            
            self.trajectory_line.set_data(x_data, y_data)
            
            if len(x_data) > 1:
                x_range = max(x_data) - min(x_data)
                y_range = max(y_data) - min(y_data)
                margin = max(2.0, max(x_range, y_range) * 0.15)
                
                self.ax1.set_xlim(min(x_data) - margin, max(x_data) + margin)
                self.ax1.set_ylim(min(y_data) - margin, max(y_data) + margin)
            
            # 更新柱状图
            self.update_bar_chart()
            plt.draw()
            plt.pause(0.01)
    
    def update_bar_chart(self):
        """更新柱状图"""
        self.ax2.clear()
        self.ax2.set_title('📊 系统活动监控', fontsize=14)
        self.ax2.set_xlabel('组件')
        self.ax2.set_ylabel('消息数量')
        
        # 准备数据
        labels = []
        values = []
        colors = []
        
        for key, count in self.message_counts.items():
            if count > 0 or key in ['trajectory', 'visual_matches']:  # 总是显示这些重要组件
                labels.append(key.replace('_', '\n'))
                values.append(count)
                if count > 0:
                    colors.append('green' if count > 10 else 'orange')
                else:
                    colors.append('red')
        
        if labels:
            bars = self.ax2.bar(labels, values, color=colors, alpha=0.7)
            
            # 添加数值标签
            for bar, value in zip(bars, values):
                if value > 0:
                    self.ax2.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.5,
                                 str(value), ha='center', va='bottom', fontsize=10)
        
        self.ax2.tick_params(axis='x', rotation=45)
    
    def print_status(self):
        """打印系统状态"""
        current_time = time.time()
        elapsed = current_time - self.start_time
        
        os.system('clear' if os.name == 'posix' else 'cls')
        
        print("🐬 Dolphin SLAM 系统状态监控 (冲突解决版)")
        print("=" * 65)
        print(f"⏱️  运行时间: {elapsed:.1f}秒")
        print(f"📊 数据流统计 (处理多种消息类型):")
        
        total_odometry = self.message_counts['odometry_nav'] + self.message_counts['odometry_pose']
        
        # 显示合并的里程计统计
        odom_rate = total_odometry / elapsed if elapsed > 0 else 0
        odom_status = "✅" if total_odometry > 0 else "❌"
        print(f"   {odom_status} 里程计 (总计): {total_odometry} 条 ({odom_rate:.1f} Hz)")
        
        if self.message_counts['odometry_nav'] > 0:
            nav_rate = self.message_counts['odometry_nav'] / elapsed
            print(f"      📍 nav_msgs/Odometry: {self.message_counts['odometry_nav']} 条 ({nav_rate:.1f} Hz)")
        
        if self.message_counts['odometry_pose'] > 0:
            pose_rate = self.message_counts['odometry_pose'] / elapsed
            print(f"      📍 PoseWithCovariance: {self.message_counts['odometry_pose']} 条 ({pose_rate:.1f} Hz)")
        
        # 其他组件
        for key in ['trajectory', 'place_cells', 'visual_matches']:
            count = self.message_counts[key]
            rate = count / elapsed if elapsed > 0 else 0
            status = "✅" if count > 0 else "❌"
            print(f"   {status} {key}: {count} 条 ({rate:.1f} Hz)")
        
        # 详细状态
        print(f"\n🎯 系统详细状态:")
        
        if self.current_position:
            pos = self.current_position
            print(f"   📍 当前位置: ({pos['x']:.2f}, {pos['y']:.2f}, {pos['z']:.2f}) m")
            print(f"   📡 位置数据源: {pos['source']}")
        else:
            print("   ❌ 无位置数据")
        
        if self.trajectory_points:
            print(f"   🗺️  轨迹点数: {len(self.trajectory_points)}")
            if len(self.trajectory_points) > 1:
                total_distance = 0
                for i in range(1, len(self.trajectory_points)):
                    p1 = self.trajectory_points[i-1]
                    p2 = self.trajectory_points[i]
                    distance = np.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)
                    total_distance += distance
                print(f"   📏 总航行距离: {total_distance:.2f} m")
        else:
            print("   ❌ 无轨迹数据")
        
        if self.place_cell_activity:
            max_activity = max(self.place_cell_activity) if self.place_cell_activity else 0
            active_cells = sum(1 for a in self.place_cell_activity if a > 0.1)
            total_cells = len(self.place_cell_activity)
            print(f"   🧠 位置细胞: {active_cells}/{total_cells} 活跃 (max: {max_activity:.3f})")
        else:
            print("   ❌ 无位置细胞数据")
        
        if self.visual_matches:
            matches = sum(self.visual_matches) if len(self.visual_matches) > 0 else 0
            print(f"   👁️  视觉匹配强度: {matches:.1f}")
        else:
            print("   ❌ 无视觉匹配数据")
        
        # 系统健康度
        healthy_components = sum([
            1 if total_odometry > 0 else 0,
            1 if self.message_counts['trajectory'] > 0 else 0,
            1 if self.message_counts['place_cells'] > 0 else 0,
            1 if self.message_counts['visual_matches'] > 0 else 0,
        ])
        
        health_percentage = (healthy_components / 4) * 100
        
        if health_percentage >= 75:
            health_icon = "🟢"
            health_status = "优秀"
        elif health_percentage >= 50:
            health_icon = "🟡" 
            health_status = "良好"
        else:
            health_icon = "🔴"
            health_status = "需要修复"
        
        print(f"\n{health_icon} 系统健康度: {health_percentage:.0f}% - {health_status} ({healthy_components}/4 组件正常)")
        
        if health_percentage == 100:
            print(f"🎉 所有系统组件正常运行！海豚SLAM导航系统完全激活！")
        elif health_percentage >= 25:
            print(f"⚡ 系统部分工作，数据流正在建立...")
        
        print(f"\n💡 实时轨迹图在单独窗口 | 话题冲突已处理 | Ctrl+C 退出")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        monitor = ConflictFreeMonitor()
        
        plot_thread = threading.Thread(target=plt.show, daemon=True)
        plot_thread.start()
        
        print("🎯 启动冲突解决版监控器...")
        print("🔧 自动处理话题类型冲突")
        print("📈 实时轨迹图将在新窗口显示")
        print("⌨️  按 Ctrl+C 停止监控")
        
        rclpy.spin(monitor)
        
    except KeyboardInterrupt:
        print("\n🛑 监控停止")
        
    except Exception as e:
        print(f"❌ 监控出错: {e}")
        
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        plt.close('all')

if __name__ == '__main__':
    main()