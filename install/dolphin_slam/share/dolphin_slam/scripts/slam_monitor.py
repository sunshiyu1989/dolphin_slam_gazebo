#!/usr/bin/env python3
"""
Dolphin SLAM 监控脚本 - 使用实际存在的话题
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped
import numpy as np
import time
import os
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import threading

class SLAMMonitor(Node):
    def __init__(self):
        super().__init__('slam_monitor')
        
        # 数据存储
        self.trajectory_points = deque(maxlen=2000)
        self.odometry_data = None
        self.place_cell_activity = None
        self.visual_matches = None
        
        # 统计信息
        self.start_time = time.time()
        self.message_counts = {
            'trajectory': 0,
            'odometry': 0,
            'place_cells': 0,
            'visual_matches': 0
        }
        
        # 订阅者 - 使用实际存在的话题名称
        
        # 里程计数据 - 实际的话题名称
        self.odometry_sub = self.create_subscription(
            Odometry,
            '/robot/odometry',  # 实际发布的话题！
            self.odometry_callback,
            10
        )
        
        # 轨迹数据 - 已确认存在
        self.trajectory_sub = self.create_subscription(
            Path,
            '/dolphin_slam/trajectory',
            self.trajectory_callback,
            10
        )
        
        # 位置细胞活动 - 已确认存在
        self.place_cell_sub = self.create_subscription(
            Float32MultiArray,
            '/place_cells/activity',
            self.place_cell_callback,
            10
        )
        
        # 视觉匹配 - 已确认存在
        self.visual_match_sub = self.create_subscription(
            Float32MultiArray,
            '/local_view/matches',
            self.visual_match_callback,
            10
        )
        
        # 定时器
        self.status_timer = self.create_timer(2.0, self.print_status)
        
        # matplotlib设置
        self.setup_plot()
        
        self.get_logger().info('🚀 实际话题监控器启动!')
        self.get_logger().info('📡 监听话题:')
        self.get_logger().info('   - /robot/odometry (里程计)')
        self.get_logger().info('   - /dolphin_slam/trajectory (轨迹)')
        self.get_logger().info('   - /place_cells/activity (位置细胞)')
        self.get_logger().info('   - /local_view/matches (视觉匹配)')
        
    def setup_plot(self):
        """设置matplotlib绘图"""
        plt.ion()  # 交互模式
        self.fig, (self.ax1, self.ax2) = plt.subplots(1, 2, figsize=(14, 6))
        
        # 轨迹图
        self.ax1.set_title('🐬 Dolphin SLAM 实时轨迹', fontsize=14)
        self.ax1.set_xlabel('X (m)')
        self.ax1.set_ylabel('Y (m)')
        self.ax1.grid(True, alpha=0.3)
        self.ax1.axis('equal')
        
        # 数据统计图
        self.ax2.set_title('📊 数据流状态', fontsize=14)
        self.ax2.set_xlabel('时间 (s)')
        self.ax2.set_ylabel('累计消息数')
        
        # 绘图线条
        self.trajectory_line, = self.ax1.plot([], [], 'g-', linewidth=3, label='AUV轨迹', alpha=0.8)
        self.current_pos, = self.ax1.plot([], [], 'ro', markersize=12, label='当前位置', zorder=5)
        self.start_pos, = self.ax1.plot([], [], 'g^', markersize=12, label='起始点', zorder=5)
        
        # 统计线条
        self.time_data = []
        self.odom_data = []
        self.traj_data = []
        self.place_data = []
        self.visual_data = []
        
        self.odom_line, = self.ax2.plot([], [], 'b-', label='里程计', linewidth=2)
        self.traj_line, = self.ax2.plot([], [], 'g-', label='轨迹', linewidth=2)
        self.place_line, = self.ax2.plot([], [], 'r-', label='位置细胞', linewidth=2)
        self.visual_line, = self.ax2.plot([], [], 'm-', label='视觉匹配', linewidth=2)
        
        self.ax1.legend()
        self.ax2.legend()
        
        plt.tight_layout()
        plt.show()
        
    def trajectory_callback(self, msg: Path):
        """处理轨迹数据"""
        self.message_counts['trajectory'] += 1
        
        if msg.poses:
            # 提取所有轨迹点
            self.trajectory_points.clear()
            for pose in msg.poses:
                x = pose.pose.position.x
                y = pose.pose.position.y
                self.trajectory_points.append((x, y))
                
            # 更新绘图
            self.update_trajectory_plot()
            
    def odometry_callback(self, msg: Odometry):
        """处理里程计数据"""
        self.message_counts['odometry'] += 1
        self.odometry_data = msg
        
        # 更新当前位置
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.current_pos.set_data([x], [y])
        
        # 如果轨迹为空，从里程计创建轨迹点
        if len(self.trajectory_points) == 0:
            self.trajectory_points.append((x, y))
            # 设置起始点
            self.start_pos.set_data([x], [y])
            self.update_trajectory_plot()
        
    def place_cell_callback(self, msg: Float32MultiArray):
        """处理位置细胞活动"""
        self.message_counts['place_cells'] += 1
        self.place_cell_activity = msg.data
        
    def visual_match_callback(self, msg: Float32MultiArray):
        """处理视觉匹配"""
        self.message_counts['visual_matches'] += 1
        self.visual_matches = msg.data
        
    def update_trajectory_plot(self):
        """更新轨迹绘图"""
        if self.trajectory_points:
            x_data = [point[0] for point in self.trajectory_points]
            y_data = [point[1] for point in self.trajectory_points]
            
            self.trajectory_line.set_data(x_data, y_data)
            
            # 自动调整坐标轴
            if len(x_data) > 1:
                x_range = max(x_data) - min(x_data)
                y_range = max(y_data) - min(y_data)
                margin = max(2.0, max(x_range, y_range) * 0.1)
                
                self.ax1.set_xlim(min(x_data) - margin, max(x_data) + margin)
                self.ax1.set_ylim(min(y_data) - margin, max(y_data) + margin)
            
            # 更新统计图
            self.update_stats_plot()
            
            plt.draw()
            plt.pause(0.01)
    
    def update_stats_plot(self):
        """更新统计图"""
        current_time = time.time() - self.start_time
        self.time_data.append(current_time)
        self.odom_data.append(self.message_counts['odometry'])
        self.traj_data.append(self.message_counts['trajectory'])
        self.place_data.append(self.message_counts['place_cells'])
        self.visual_data.append(self.message_counts['visual_matches'])
        
        # 保持最近100个点
        if len(self.time_data) > 100:
            self.time_data = self.time_data[-100:]
            self.odom_data = self.odom_data[-100:]
            self.traj_data = self.traj_data[-100:]
            self.place_data = self.place_data[-100:]
            self.visual_data = self.visual_data[-100:]
        
        self.odom_line.set_data(self.time_data, self.odom_data)
        self.traj_line.set_data(self.time_data, self.traj_data)
        self.place_line.set_data(self.time_data, self.place_data)
        self.visual_line.set_data(self.time_data, self.visual_data)
        
        if self.time_data:
            self.ax2.set_xlim(0, max(self.time_data) + 1)
            max_y = max(max(self.odom_data), max(self.traj_data), 
                       max(self.place_data), max(self.visual_data)) + 1
            self.ax2.set_ylim(0, max_y)
    
    def print_status(self):
        """打印系统状态"""
        current_time = time.time()
        elapsed = current_time - self.start_time
        
        # 清屏
        os.system('clear' if os.name == 'posix' else 'cls')
        
        print("🐬 Dolphin SLAM 系统状态监控 (实际话题版)")
        print("=" * 60)
        print(f"⏱️  运行时间: {elapsed:.1f}秒")
        print(f"📊 数据流统计 (使用实际话题):")
        
        for topic, count in self.message_counts.items():
            rate = count / elapsed if elapsed > 0 else 0
            
            # 根据数据量显示状态
            if count > 0:
                if rate > 0.5:
                    status = "✅"
                elif rate > 0.1:
                    status = "🟡"  # 有数据但频率低
                else:
                    status = "🟠"  # 很少数据
            else:
                status = "❌"
                
            print(f"   {status} {topic}: {count} 条 ({rate:.1f} Hz)")
        
        # 系统状态详情
        print(f"\n🎯 系统详细状态:")
        
        # 里程计状态
        if self.odometry_data:
            pos = self.odometry_data.pose.pose.position
            print(f"   📍 当前位置: ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f}) m")
            
            vel = self.odometry_data.twist.twist.linear
            speed = np.sqrt(vel.x**2 + vel.y**2 + vel.z**2)
            print(f"   🚤 当前速度: {speed:.2f} m/s")
            
            # 姿态
            orient = self.odometry_data.pose.pose.orientation
            print(f"   🧭 姿态: w={orient.w:.3f}")
        else:
            print("   ❌ 无里程计数据")
        
        # 轨迹状态
        if self.trajectory_points:
            print(f"   🗺️  轨迹点数: {len(self.trajectory_points)}")
            
            if len(self.trajectory_points) > 1:
                # 计算总距离
                total_distance = 0
                for i in range(1, len(self.trajectory_points)):
                    p1 = self.trajectory_points[i-1]
                    p2 = self.trajectory_points[i]
                    distance = np.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)
                    total_distance += distance
                print(f"   📏 总航行距离: {total_distance:.2f} m")
                
                # 当前区域
                x_data = [p[0] for p in self.trajectory_points]
                y_data = [p[1] for p in self.trajectory_points]
                print(f"   📐 探索区域: {max(x_data)-min(x_data):.1f}m × {max(y_data)-min(y_data):.1f}m")
        else:
            print("   ❌ 无轨迹数据")
        
        # 位置细胞状态
        if self.place_cell_activity:
            max_activity = max(self.place_cell_activity) if self.place_cell_activity else 0
            active_cells = sum(1 for a in self.place_cell_activity if a > 0.1)
            total_cells = len(self.place_cell_activity)
            activity_percent = (active_cells / total_cells * 100) if total_cells > 0 else 0
            print(f"   🧠 位置细胞: {active_cells}/{total_cells} 活跃 ({activity_percent:.1f}%, max: {max_activity:.3f})")
        else:
            print("   ❌ 无位置细胞数据")
        
        # 视觉匹配状态  
        if self.visual_matches:
            matches = sum(self.visual_matches) if len(self.visual_matches) > 0 else 0
            print(f"   👁️  视觉匹配强度: {matches:.1f}")
        else:
            print("   ❌ 无视觉匹配数据")
        
        # 系统健康度
        healthy_components = sum([
            1 if self.message_counts['odometry'] > 0 else 0,
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
        elif health_percentage >= 50:
            print(f"⚡ 系统部分工作，继续监控...")
        else:
            print(f"🔧 需要检查节点状态")
                
        print(f"\n💡 实时轨迹图在单独窗口中显示 | 按 Ctrl+C 退出并保存数据")
        
    def save_trajectory(self, filename="dolphin_slam_trajectory.txt"):
        """保存轨迹数据"""
        if self.trajectory_points:
            with open(filename, 'w') as f:
                f.write("# Dolphin SLAM Trajectory Data\n")
                f.write("# Generated on: " + time.strftime("%Y-%m-%d %H:%M:%S\n"))
                f.write("# Total points: " + str(len(self.trajectory_points)) + "\n")
                f.write("# X(m) Y(m)\n")
                for x, y in self.trajectory_points:
                    f.write(f"{x:.6f} {y:.6f}\n")
            print(f"💾 轨迹数据已保存到 {filename} ({len(self.trajectory_points)} 个点)")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        monitor = SLAMMonitor()
        
        # 在单独线程中运行matplotlib
        plot_thread = threading.Thread(target=plt.show, daemon=True)
        plot_thread.start()
        
        print("🎯 开始监控Dolphin SLAM系统...")
        print("🐬 使用实际存在的话题进行监控")
        print("📈 实时轨迹图和统计图将在新窗口中显示")
        print("⌨️  按 Ctrl+C 停止监控并保存轨迹数据")
        
        rclpy.spin(monitor)
        
    except KeyboardInterrupt:
        print("\n🛑 监控停止")
        # 保存轨迹数据
        monitor.save_trajectory()
        
    except Exception as e:
        print(f"❌ 监控出错: {e}")
        
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        plt.close('all')

if __name__ == '__main__':
    main()