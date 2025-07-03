#!/usr/bin/env python3
"""
简单工作版 Dolphin SLAM 监控脚本
避免话题冲突，使用明确的话题名称
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float32MultiArray
import time
import os

class SimpleWorkingMonitor(Node):
    def __init__(self):
        super().__init__('simple_working_monitor')
        
        # 统计信息
        self.start_time = time.time()
        self.message_counts = {
            'trajectory': 0,
            'place_cells': 0,
            'visual_matches': 0
        }
        
        # 数据存储
        self.latest_position = None
        self.trajectory_length = 0
        self.place_cell_activity = None
        self.visual_matches = None
        
        # 订阅者 - 只订阅不冲突的话题
        
        # 轨迹数据 - 安全的话题
        self.trajectory_sub = self.create_subscription(
            Path,
            '/dolphin_slam/trajectory',
            self.trajectory_callback,
            10
        )
        
        # 位置细胞活动 - 安全的话题
        self.place_cell_sub = self.create_subscription(
            Float32MultiArray,
            '/place_cells/activity',
            self.place_cell_callback,
            10
        )
        
        # 视觉匹配 - 已确认工作的话题
        self.visual_match_sub = self.create_subscription(
            Float32MultiArray,
            '/local_view/matches',
            self.visual_match_callback,
            10
        )
        
        # 定时器
        self.status_timer = self.create_timer(2.0, self.print_status)
        
        self.get_logger().info('🚀 简单工作版监控器启动!')
        self.get_logger().info('📡 监听安全话题:')
        self.get_logger().info('   - /dolphin_slam/trajectory')
        self.get_logger().info('   - /place_cells/activity')
        self.get_logger().info('   - /local_view/matches')
        
    def trajectory_callback(self, msg: Path):
        """处理轨迹数据"""
        self.message_counts['trajectory'] += 1
        
        if msg.poses:
            self.trajectory_length = len(msg.poses)
            # 获取最新位置
            latest_pose = msg.poses[-1]
            self.latest_position = {
                'x': latest_pose.pose.position.x,
                'y': latest_pose.pose.position.y,
                'z': latest_pose.pose.position.z
            }
        
    def place_cell_callback(self, msg: Float32MultiArray):
        """处理位置细胞活动"""
        self.message_counts['place_cells'] += 1
        self.place_cell_activity = msg.data
        
    def visual_match_callback(self, msg: Float32MultiArray):
        """处理视觉匹配"""
        self.message_counts['visual_matches'] += 1
        self.visual_matches = msg.data
        
    def print_status(self):
        """打印系统状态"""
        current_time = time.time()
        elapsed = current_time - self.start_time
        
        # 清屏
        os.system('clear' if os.name == 'posix' else 'cls')
        
        print("🐬 Dolphin SLAM 系统状态监控 (简单工作版)")
        print("=" * 60)
        print(f"⏱️  运行时间: {elapsed:.1f}秒")
        print(f"📊 数据流统计 (避免话题冲突):")
        
        for topic, count in self.message_counts.items():
            rate = count / elapsed if elapsed > 0 else 0
            
            if count > 0:
                if rate > 0.5:
                    status = "✅"
                elif rate > 0.1:
                    status = "🟡"
                else:
                    status = "🟠"
            else:
                status = "❌"
                
            print(f"   {status} {topic}: {count} 条 ({rate:.1f} Hz)")
        
        # 系统详细状态
        print(f"\n🎯 系统详细状态:")
        
        # 轨迹信息
        if self.trajectory_length > 0:
            print(f"   🗺️  轨迹点数: {self.trajectory_length}")
            
            if self.latest_position:
                pos = self.latest_position
                print(f"   📍 最新位置: ({pos['x']:.2f}, {pos['y']:.2f}, {pos['z']:.2f}) m")
        else:
            print("   ❌ 无轨迹数据")
        
        # 位置细胞状态
        if self.place_cell_activity:
            max_activity = max(self.place_cell_activity) if self.place_cell_activity else 0
            active_cells = sum(1 for a in self.place_cell_activity if a > 0.1)
            total_cells = len(self.place_cell_activity)
            
            if total_cells > 0:
                activity_percent = (active_cells / total_cells * 100)
                print(f"   🧠 位置细胞: {active_cells}/{total_cells} 活跃 ({activity_percent:.1f}%)")
                print(f"      最大活动强度: {max_activity:.3f}")
        else:
            print("   ❌ 无位置细胞数据")
        
        # 视觉匹配状态  
        if self.visual_matches:
            if len(self.visual_matches) > 0:
                matches = sum(self.visual_matches)
                avg_match = matches / len(self.visual_matches)
                print(f"   👁️  视觉匹配: 总强度 {matches:.1f}, 平均 {avg_match:.2f}")
            else:
                print(f"   👁️  视觉匹配: 空数据")
        else:
            print("   ❌ 无视觉匹配数据")
        
        # 系统健康度评估
        healthy_components = sum([
            1 if self.message_counts['trajectory'] > 0 else 0,
            1 if self.message_counts['place_cells'] > 0 else 0,
            1 if self.message_counts['visual_matches'] > 0 else 0,
        ])
        
        health_percentage = (healthy_components / 3) * 100
        
        if health_percentage >= 67:
            health_icon = "🟢"
            health_status = "优秀"
        elif health_percentage >= 34:
            health_icon = "🟡" 
            health_status = "良好"
        else:
            health_icon = "🔴"
            health_status = "需要修复"
        
        print(f"\n{health_icon} 系统健康度: {health_percentage:.0f}% - {health_status} ({healthy_components}/3 组件正常)")
        
        # 状态说明
        if health_percentage == 100:
            print(f"🎉 所有监控组件正常工作！")
            print(f"💡 注意：里程计数据因话题冲突未直接监控")
        elif health_percentage >= 34:
            print(f"⚡ 系统部分组件工作正常")
        else:
            print(f"🔧 需要检查系统状态")
        
        print(f"\n📝 话题状态说明:")
        print(f"   ✅ /dolphin_slam/trajectory - 轨迹数据")
        print(f"   ✅ /place_cells/activity - 位置细胞活动") 
        print(f"   ✅ /local_view/matches - 视觉特征匹配")
        print(f"   ⚠️  /robot/odometry - 话题冲突，未监控")
        
        print(f"\n💡 按 Ctrl+C 退出监控")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        monitor = SimpleWorkingMonitor()
        
        print("🎯 启动简单工作版监控器...")
        print("🔧 避免话题冲突，专注核心功能")
        print("⌨️  按 Ctrl+C 停止监控")
        
        rclpy.spin(monitor)
        
    except KeyboardInterrupt:
        print("\n🛑 监控停止")
        
    except Exception as e:
        print(f"❌ 监控出错: {e}")
        
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()