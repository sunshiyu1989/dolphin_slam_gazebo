#!/usr/bin/env python3
"""
系统停止监听器 - 监听数据播放完成信号并优雅停止系统
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import subprocess
import time
import signal
import os

class SystemStopMonitor(Node):
    """
    系统停止监听器
    
    功能:
    1. 监听数据播放完成信号
    2. 等待所有节点处理完最后的数据
    3. 优雅地停止整个系统
    4. 生成运行报告
    """
    
    def __init__(self):
        super().__init__('system_stop_monitor')
        
        # 参数
        self.declare_parameter('grace_period', 5.0)  # 优雅停止等待时间
        self.declare_parameter('auto_shutdown', True)  # 是否自动关闭系统
        
        self.grace_period = self.get_parameter('grace_period').value
        self.auto_shutdown = self.get_parameter('auto_shutdown').value
        
        # 状态跟踪
        self.data_finished = False
        self.system_stopping = False
        self.start_time = time.time()
        
        # 订阅播放状态
        self.create_subscription(
            Bool,
            '/data_controller/finished',
            self.data_finished_callback,
            10
        )
        
        # 发布系统状态
        self.system_status_pub = self.create_publisher(
            Bool,
            '/system/stopping',
            10
        )
        
        self.get_logger().info('🔍 系统停止监听器已启动')
        self.get_logger().info(f'⏱️ 优雅停止等待时间: {self.grace_period} 秒')
        
    def data_finished_callback(self, msg):
        """数据播放完成回调"""
        if msg.data and not self.data_finished:
            self.data_finished = True
            runtime = time.time() - self.start_time
            
            self.get_logger().info('🏁 数据播放完成信号已接收')
            self.get_logger().info(f'📊 总运行时间: {runtime:.1f} 秒')
            
            if self.auto_shutdown:
                self.get_logger().info(f'⏳ 等待 {self.grace_period} 秒后停止系统...')
                self.create_timer(self.grace_period, self.initiate_system_stop)
            else:
                self.get_logger().info('ℹ️ 自动停止已禁用，系统将继续运行')
    
    def initiate_system_stop(self):
        """启动系统停止流程"""
        if self.system_stopping:
            return
            
        self.system_stopping = True
        
        # 发布系统停止信号
        stop_msg = Bool()
        stop_msg.data = True
        self.system_status_pub.publish(stop_msg)
        
        self.get_logger().info('🛑 启动系统停止流程...')
        
        # 生成运行报告
        self.generate_run_report()
        
        # 停止系统
        self.stop_ros_system()
        
    def generate_run_report(self):
        """生成运行报告"""
        try:
            runtime = time.time() - self.start_time
            
            self.get_logger().info('📋 生成运行报告...')
            
            # 获取节点信息
            try:
                result = subprocess.run(['ros2', 'node', 'list'], 
                                      capture_output=True, text=True, timeout=5)
                active_nodes = result.stdout.strip().split('\n') if result.returncode == 0 else []
            except:
                active_nodes = ['无法获取节点列表']
            
            # 获取话题信息
            try:
                result = subprocess.run(['ros2', 'topic', 'list'], 
                                      capture_output=True, text=True, timeout=5)
                active_topics = result.stdout.strip().split('\n') if result.returncode == 0 else []
            except:
                active_topics = ['无法获取话题列表']
            
            # 打印报告
            self.get_logger().info('=' * 60)
            self.get_logger().info('🎯 DOLPHIN SLAM 运行报告')
            self.get_logger().info('=' * 60)
            self.get_logger().info(f'⏰ 总运行时间: {runtime:.2f} 秒')
            self.get_logger().info(f'📅 结束时间: {time.strftime("%Y-%m-%d %H:%M:%S")}')
            self.get_logger().info('')
            self.get_logger().info(f'🔧 活跃节点数: {len(active_nodes)}')
            for node in active_nodes:
                if node.strip():
                    self.get_logger().info(f'   - {node.strip()}')
            self.get_logger().info('')
            self.get_logger().info(f'📡 活跃话题数: {len(active_topics)}')
            dolphin_topics = [t for t in active_topics if 'dolphin' in t.lower()]
            if dolphin_topics:
                self.get_logger().info('   Dolphin SLAM 相关话题:')
                for topic in dolphin_topics:
                    if topic.strip():
                        self.get_logger().info(f'   - {topic.strip()}')
            self.get_logger().info('=' * 60)
            
        except Exception as e:
            self.get_logger().error(f'❌ 生成报告失败: {e}')
    
    def stop_ros_system(self):
        """停止ROS系统"""
        self.get_logger().info('🔄 正在停止ROS系统...')
        
        try:
            # 发送SIGINT信号到当前进程组（优雅停止）
            os.killpg(os.getpgrp(), signal.SIGINT)
            
        except Exception as e:
            self.get_logger().error(f'❌ 停止系统失败: {e}')
            # 如果优雅停止失败，强制停止
            try:
                os.killpg(os.getpgrp(), signal.SIGTERM)
            except:
                pass

def main(args=None):
    rclpy.init(args=args)
    
    try:
        monitor = SystemStopMonitor()
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"监听器错误: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
