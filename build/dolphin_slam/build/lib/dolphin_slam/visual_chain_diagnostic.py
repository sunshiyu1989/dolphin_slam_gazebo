#!/usr/bin/env python3
"""
视觉链路诊断节点 - 检查从相机到位置细胞的整个视觉处理链路
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
import time
import numpy as np

class VisualChainDiagnostic(Node):
    """视觉链路诊断节点"""
    
    def __init__(self):
        super().__init__('visual_chain_diagnostic')
        
        # 统计信息
        self.stats = {
            'camera_frames': 0,
            'descriptor_frames': 0,
            'match_frames': 0,
            'last_camera_time': 0,
            'last_descriptor_time': 0,
            'last_match_time': 0,
            'camera_topic': '/forward_camera/image_raw',
            'descriptors_topic': '/features/descriptors',
            'matches_topic': '/local_view/matches'
        }
        
        # 订阅所有相关话题
        self.camera_sub = self.create_subscription(
            Image,
            self.stats['camera_topic'],
            self.camera_callback,
            10
        )
        
        self.descriptors_sub = self.create_subscription(
            Image,
            self.stats['descriptors_topic'],
            self.descriptors_callback,
            10
        )
        
        self.matches_sub = self.create_subscription(
            Float32MultiArray,
            self.stats['matches_topic'],
            self.matches_callback,
            10
        )
        
        # 诊断定时器
        self.diagnostic_timer = self.create_timer(5.0, self.diagnostic_report)
        
        self.get_logger().info('🔍 视觉链路诊断节点已启动')
        self.get_logger().info(f'📷 监控相机话题: {self.stats["camera_topic"]}')
        self.get_logger().info(f'🔍 监控描述符话题: {self.stats["descriptors_topic"]}')
        self.get_logger().info(f'🎯 监控匹配话题: {self.stats["matches_topic"]}')

    def camera_callback(self, msg):
        """相机数据回调"""
        self.stats['camera_frames'] += 1
        self.stats['last_camera_time'] = time.time()
        
        if self.stats['camera_frames'] % 50 == 1:  # 每50帧打印一次
            self.get_logger().info(f'📷 相机帧 #{self.stats["camera_frames"]}: {msg.width}x{msg.height}')

    def descriptors_callback(self, msg):
        """描述符数据回调"""
        self.stats['descriptor_frames'] += 1
        self.stats['last_descriptor_time'] = time.time()
        
        if self.stats['descriptor_frames'] % 20 == 1:  # 每20帧打印一次
            self.get_logger().info(f'🔍 描述符帧 #{self.stats["descriptor_frames"]}: {msg.width}x{msg.height}')

    def matches_callback(self, msg):
        """匹配数据回调"""
        self.stats['match_frames'] += 1
        self.stats['last_match_time'] = time.time()
        
        if len(msg.data) >= 4:
            similarity = msg.data[0]
            template_id = msg.data[1]
            matched = msg.data[2]
            is_novel = msg.data[3]
            
            if self.stats['match_frames'] % 10 == 1:  # 每10帧打印一次
                self.get_logger().info(f'🎯 匹配帧 #{self.stats["match_frames"]}: 相似度={similarity:.3f}, 模板ID={template_id}, 匹配={matched}, 新颖={is_novel}')

    def diagnostic_report(self):
        """诊断报告"""
        current_time = time.time()
        
        # 检查数据流状态
        camera_active = (current_time - self.stats['last_camera_time']) < 2.0
        descriptors_active = (current_time - self.stats['last_descriptor_time']) < 2.0
        matches_active = (current_time - self.stats['last_match_time']) < 2.0
        
        self.get_logger().info('📊 视觉链路诊断报告:')
        self.get_logger().info(f'   📷 相机: {self.stats["camera_frames"]}帧 {"✅" if camera_active else "❌"}')
        self.get_logger().info(f'   🔍 描述符: {self.stats["descriptor_frames"]}帧 {"✅" if descriptors_active else "❌"}')
        self.get_logger().info(f'   🎯 匹配: {self.stats["match_frames"]}帧 {"✅" if matches_active else "❌"}')
        
        # 链路状态判断
        if camera_active and descriptors_active and matches_active:
            self.get_logger().info('✅ 视觉链路完整 - 所有组件正常工作')
        elif camera_active and descriptors_active:
            self.get_logger().info('⚠️ 视觉链路部分 - 相机和描述符正常，匹配数据缺失')
        elif camera_active:
            self.get_logger().info('⚠️ 视觉链路中断 - 仅相机正常，后续链路断开')
        else:
            self.get_logger().info('❌ 视觉链路完全断开 - 相机数据缺失')

def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    try:
        node = VisualChainDiagnostic()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"诊断节点运行出错: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main() 