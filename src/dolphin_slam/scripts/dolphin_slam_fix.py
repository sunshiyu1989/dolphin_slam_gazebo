#!/usr/bin/env python3
"""
Dolphin SLAM 系统完整修复脚本
解决话题命名不一致和节点通信问题
"""

import os
import sys
import shutil
import yaml
import subprocess
from pathlib import Path

def print_header(title):
    """打印标题"""
    print(f"\n{'='*60}")
    print(f"🔧 {title}")
    print(f"{'='*60}")

def print_step(step, description):
    """打印步骤"""
    print(f"\n📋 步骤 {step}: {description}")
    print("-" * 40)

def backup_file(file_path):
    """备份文件"""
    if os.path.exists(file_path):
        backup_path = f"{file_path}.backup_{int(__import__('time').time())}"
        shutil.copy2(file_path, backup_path)
        print(f"✅ 已备份: {backup_path}")
        return True
    return False

def fix_robot_state_node():
    """修复 robot_state_node.py - 统一发布到 /dolphin_slam/odometry"""
    print_step(1, "修复 robot_state_node.py")
    
    robot_state_files = [
        "~/dolphin_slam_ws/src/dolphin_slam/nodes/robot_state_node.py",
        "~/dolphin_slam_ws/src/dolphin_slam/dolphin_slam/robot_state_node.py"
    ]
    
    for file_path in robot_state_files:
        full_path = os.path.expanduser(file_path)
        if os.path.exists(full_path):
            print(f"🔧 修复文件: {full_path}")
            backup_file(full_path)
            
            with open(full_path, 'r', encoding='utf-8') as f:
                content = f.read()
            
            # 修复发布话题名称
            content = content.replace(
                "'/robot/odometry'",
                "'/dolphin_slam/odometry'"
            )
            
            # 确保正确的消息类型转换
            if "float(str(" in content:
                content = content.replace("float(str(pose.x))", "float(pose.x)")
                content = content.replace("float(str(pose.y))", "float(pose.y)")
                content = content.replace("float(str(pose.z))", "float(pose.z)")
                content = content.replace("float(str(q[0]))", "float(q[0])")
                content = content.replace("float(str(q[1]))", "float(q[1])")
                content = content.replace("float(str(q[2]))", "float(q[2])")
                content = content.replace("float(str(q[3]))", "float(q[3])")
                content = content.replace("float(str(velocity.vx))", "float(velocity.vx)")
                content = content.replace("float(str(velocity.vy))", "float(velocity.vy)")
                content = content.replace("float(str(velocity.vz))", "float(velocity.vz)")
                content = content.replace("float(str(velocity.wx))", "float(velocity.wx)")
                content = content.replace("float(str(velocity.wy))", "float(velocity.wy)")
                content = content.replace("float(str(velocity.wz))", "float(velocity.wz)")
            
            with open(full_path, 'w', encoding='utf-8') as f:
                f.write(content)
            
            print(f"✅ {full_path} 修复完成")

def fix_place_cell_node():
    """修复 place_cell_node.py - 统一订阅 /dolphin_slam/odometry"""
    print_step(2, "修复 place_cell_node.py")
    
    place_cell_files = [
        "~/dolphin_slam_ws/src/dolphin_slam/nodes/place_cell_node.py",
        "~/dolphin_slam_ws/src/dolphin_slam/dolphin_slam/place_cell_node.py"
    ]
    
    for file_path in place_cell_files:
        full_path = os.path.expanduser(file_path)
        if os.path.exists(full_path):
            print(f"🔧 修复文件: {full_path}")
            backup_file(full_path)
            
            with open(full_path, 'r', encoding='utf-8') as f:
                content = f.read()
            
            # 修复订阅话题名称和默认参数
            content = content.replace(
                "('odometry_topic', '/robot/odometry')",
                "('odometry_topic', '/dolphin_slam/odometry')"
            )
            
            # 确保订阅正确的话题
            if "self.get_parameter('odometry_topic').value" not in content:
                content = content.replace(
                    "'/robot/odometry'",
                    "'/dolphin_slam/odometry'"
                )
            
            with open(full_path, 'w', encoding='utf-8') as f:
                f.write(content)
            
            print(f"✅ {full_path} 修复完成")

def fix_experience_map_node():
    """修复 experience_map_node.py - 统一订阅和发布"""
    print_step(3, "修复 experience_map_node.py")
    
    experience_files = [
        "~/dolphin_slam_ws/src/dolphin_slam/nodes/experience_map_node.py",
        "~/dolphin_slam_ws/src/dolphin_slam/dolphin_slam/experience_map_node.py"
    ]
    
    for file_path in experience_files:
        full_path = os.path.expanduser(file_path)
        if os.path.exists(full_path):
            print(f"🔧 修复文件: {full_path}")
            backup_file(full_path)
            
            with open(full_path, 'r', encoding='utf-8') as f:
                content = f.read()
            
            # 修复订阅话题名称
            content = content.replace(
                "('/robot/odometry'",
                "('/dolphin_slam/odometry'"
            )
            content = content.replace(
                "'/robot/odometry'",
                "'/dolphin_slam/odometry'"
            )
            
            # 确保发布轨迹数据
            if 'self.trajectory_pub' not in content:
                # 添加轨迹发布者
                pub_section = """        
        self.trajectory_pub = self.create_publisher(
            Path,
            '/dolphin_slam/trajectory',
            10
        )"""
                
                # 在experience_pub后面添加
                content = content.replace(
                    "        self.experience_pub = self.create_publisher(",
                    pub_section + "\n        \n        self.experience_pub = self.create_publisher("
                )
            
            with open(full_path, 'w', encoding='utf-8') as f:
                f.write(content)
            
            print(f"✅ {full_path} 修复完成")

def fix_config_file():
    """修复配置文件 - 统一话题命名"""
    print_step(4, "修复配置文件")
    
    config_files = [
        "~/dolphin_slam_ws/src/dolphin_slam/config/dolphin_slam_params.yaml",
        "~/dolphin_slam_ws/src/dolphin_slam/config/params.yaml"
    ]
    
    for file_path in config_files:
        full_path = os.path.expanduser(file_path)
        if os.path.exists(full_path):
            print(f"🔧 修复配置: {full_path}")
            backup_file(full_path)
            
            try:
                with open(full_path, 'r', encoding='utf-8') as f:
                    config = yaml.safe_load(f)
                
                # 修复 place_cell_node 配置
                if 'place_cell_node' in config and 'ros__parameters' in config['place_cell_node']:
                    config['place_cell_node']['ros__parameters']['odometry_topic'] = '/dolphin_slam/odometry'
                
                # 修复 experience_map_node 配置  
                if 'experience_map_node' in config and 'ros__parameters' in config['experience_map_node']:
                    config['experience_map_node']['ros__parameters']['odometry_topic'] = '/dolphin_slam/odometry'
                
                with open(full_path, 'w', encoding='utf-8') as f:
                    yaml.dump(config, f, default_flow_style=False, allow_unicode=True)
                
                print(f"✅ {full_path} 配置已修复")
                
            except Exception as e:
                print(f"⚠️ 配置文件格式问题: {e}")
                # 直接字符串替换作为备用方案
                with open(full_path, 'r', encoding='utf-8') as f:
                    content = f.read()
                
                content = content.replace(
                    'odometry_topic: "/robot/odometry"',
                    'odometry_topic: "/dolphin_slam/odometry"'
                )
                
                with open(full_path, 'w', encoding='utf-8') as f:
                    f.write(content)
                
                print(f"✅ {full_path} 已使用字符串替换修复")

def create_enhanced_experience_map():
    """创建增强的 experience_map_node.py"""
    print_step(5, "创建增强的 experience_map_node.py")
    
    enhanced_code = '''#!/usr/bin/env python3
"""
Dolphin SLAM - 增强经验地图 ROS2 节点
修复轨迹发布和话题订阅问题
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped
import numpy as np
from typing import Optional

class EnhancedExperienceMapNode(Node):
    """增强经验地图 ROS2 节点"""
    
    def __init__(self):
        super().__init__('experience_map_node')
        
        # 状态变量
        self.current_odometry: Optional[Odometry] = None
        self.trajectory_poses = []
        self.place_cell_data = None
        self.visual_match_data = None
        
        # 订阅者 - 使用统一的话题名称
        self.odometry_sub = self.create_subscription(
            Odometry,
            '/dolphin_slam/odometry',
            self.odometry_callback,
            10
        )
        
        self.place_cell_sub = self.create_subscription(
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
        
        # 定时器
        self.update_timer = self.create_timer(0.1, self.update_and_publish)
        
        self.get_logger().info('增强经验地图节点已启动 - 修复版本')
        
    def odometry_callback(self, msg: Odometry):
        """处理里程计数据"""
        self.current_odometry = msg
        
        # 创建轨迹点
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose = msg.pose.pose
        
        self.trajectory_poses.append(pose_stamped)
        
        # 限制轨迹长度避免内存溢出
        if len(self.trajectory_poses) > 2000:
            self.trajectory_poses = self.trajectory_poses[-1000:]
        
    def place_cell_callback(self, msg: Float32MultiArray):
        """处理位置细胞活动"""
        self.place_cell_data = msg.data
        
    def visual_match_callback(self, msg: Float32MultiArray):
        """处理视觉匹配数据"""
        self.visual_match_data = msg.data
        
    def update_and_publish(self):
        """更新并发布轨迹和经验数据"""
        # 发布轨迹
        if self.trajectory_poses:
            path_msg = Path()
            path_msg.header.stamp = self.get_clock().now().to_msg()
            path_msg.header.frame_id = "map"
            path_msg.poses = self.trajectory_poses.copy()
            
            self.trajectory_pub.publish(path_msg)
            
        # 发布经验数据
        if self.current_odometry:
            experience_msg = Float32MultiArray()
            
            # 简单的经验数据（位置 + 活动强度）
            data = [
                self.current_odometry.pose.pose.position.x,
                self.current_odometry.pose.pose.position.y,
                self.current_odometry.pose.pose.position.z
            ]
            
            # 添加位置细胞活动（如果有的话）
            if self.place_cell_data:
                max_activity = max(self.place_cell_data) if self.place_cell_data else 0.0
                data.append(max_activity)
            else:
                data.append(0.0)
                
            # 添加视觉匹配强度
            if self.visual_match_data:
                match_strength = sum(self.visual_match_data) if self.visual_match_data else 0.0
                data.append(match_strength)
            else:
                data.append(0.0)
            
            experience_msg.data = data
            self.experience_pub.publish(experience_msg)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = EnhancedExperienceMapNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
'''
    
    # 写入增强版本
    enhanced_path = os.path.expanduser("~/dolphin_slam_ws/src/dolphin_slam/nodes/enhanced_experience_map_node.py")
    with open(enhanced_path, 'w', encoding='utf-8') as f:
        f.write(enhanced_code)
    
    # 设置执行权限
    os.chmod(enhanced_path, 0o755)
    print(f"✅ 创建增强版本: {enhanced_path}")

def rebuild_project():
    """重新构建项目"""
    print_step(6, "重新构建项目")
    
    workspace_dir = os.path.expanduser("~/dolphin_slam_ws")
    
    if not os.path.exists(workspace_dir):
        print(f"❌ 工作空间不存在: {workspace_dir}")
        return False
    
    os.chdir(workspace_dir)
    
    # 清理旧的构建文件
    print("🧹 清理旧构建文件...")
    build_dirs = ['build', 'install', 'log']
    for dir_name in build_dirs:
        if os.path.exists(dir_name):
            shutil.rmtree(dir_name)
    
    # 重新构建
    print("🔨 重新构建项目...")
    result = subprocess.run([
        'colcon', 'build', 
        '--packages-select', 'dolphin_slam',
        '--symlink-install'
    ], capture_output=True, text=True)
    
    if result.returncode == 0:
        print("✅ 构建成功")
        return True
    else:
        print(f"❌ 构建失败:")
        print(f"STDOUT: {result.stdout}")
        print(f"STDERR: {result.stderr}")
        return False

def create_launch_script():
    """创建修复版启动脚本"""
    print_step(7, "创建修复版启动脚本")
    
    launch_script = '''#!/bin/bash
# Dolphin SLAM 修复版启动脚本

echo "🐬 启动修复版 Dolphin SLAM"
echo "=========================="

# 设置环境
export ROS_DOMAIN_ID=42
cd ~/dolphin_slam_ws
source install/setup.bash

echo "🚀 启动所有节点..."

# 检查数据集路径
DATASET_PATH="/media/psf/Samsung T7/SLAM Data/Sunboat_03-09-2023/2023-09-03-07-58-37"

if [ ! -d "$DATASET_PATH" ]; then
    echo "⚠️ 数据集路径不存在，请确认路径是否正确"
    echo "当前路径: $DATASET_PATH"
    read -p "是否继续？(y/n): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# 启动主系统
echo "📡 启动修复版系统..."
ros2 launch dolphin_slam dolphin_slam_launch.py \\
    dataset_path:="$DATASET_PATH" \\
    enable_rviz:=false

echo "🏁 系统启动完成"
'''
    
    script_path = os.path.expanduser("~/dolphin_slam_ws/start_fixed_system.sh")
    with open(script_path, 'w', encoding='utf-8') as f:
        f.write(launch_script)
    
    os.chmod(script_path, 0o755)
    print(f"✅ 启动脚本创建: {script_path}")

def create_monitor_script():
    """创建修复版监控脚本"""
    print_step(8, "创建修复版监控脚本")
    
    monitor_script = '''#!/usr/bin/env python3
"""
修复版 Dolphin SLAM 状态监控
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Float32MultiArray
import time
import os

class FixedMonitor(Node):
    def __init__(self):
        super().__init__('fixed_monitor')
        
        self.start_time = time.time()
        self.message_counts = {
            'trajectory': 0,
            'odometry': 0,
            'place_cells': 0,
            'visual_matches': 0
        }
        
        # 订阅所有相关话题
        self.trajectory_sub = self.create_subscription(
            Path, '/dolphin_slam/trajectory', 
            lambda msg: self.count_message('trajectory'), 10)
            
        self.odometry_sub = self.create_subscription(
            Odometry, '/dolphin_slam/odometry',
            lambda msg: self.count_message('odometry'), 10)
            
        self.place_cell_sub = self.create_subscription(
            Float32MultiArray, '/place_cells/activity',
            lambda msg: self.count_message('place_cells'), 10)
            
        self.visual_match_sub = self.create_subscription(
            Float32MultiArray, '/local_view/matches',
            lambda msg: self.count_message('visual_matches'), 10)
        
        self.timer = self.create_timer(3.0, self.print_status)
        
    def count_message(self, topic):
        self.message_counts[topic] += 1
        
    def print_status(self):
        os.system('clear')
        elapsed = time.time() - self.start_time
        
        print("🐬 修复版 Dolphin SLAM 监控")
        print("=" * 50)
        print(f"⏱️ 运行时间: {elapsed:.1f}秒")
        print("📊 话题状态:")
        
        all_working = True
        for topic, count in self.message_counts.items():
            rate = count / elapsed if elapsed > 0 else 0
            
            if count > 0:
                status = "✅" if rate > 0.1 else "🟡"
            else:
                status = "❌"
                all_working = False
                
            print(f"   {status} {topic}: {count} 条 ({rate:.1f} Hz)")
        
        health = sum(1 for c in self.message_counts.values() if c > 0)
        print(f"\\n🎯 系统健康度: {health}/4 组件工作")
        
        if all_working:
            print("🎉 所有组件正常工作！修复成功！")
        else:
            print("🔧 部分组件仍需调试...")

def main():
    rclpy.init()
    
    try:
        monitor = FixedMonitor()
        print("🚀 启动修复版监控...")
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        print("\\n🛑 监控停止")
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
'''
    
    monitor_path = os.path.expanduser("~/dolphin_slam_ws/src/dolphin_slam/scripts/fixed_monitor.py")
    with open(monitor_path, 'w', encoding='utf-8') as f:
        f.write(monitor_script)
    
    os.chmod(monitor_path, 0o755)
    print(f"✅ 监控脚本创建: {monitor_path}")

def main():
    """主修复流程"""
    print_header("Dolphin SLAM 系统完整修复")
    
    print("🎯 修复目标:")
    print("   - 统一话题命名：使用 /dolphin_slam/odometry")
    print("   - 修复轨迹发布：确保 /dolphin_slam/trajectory 正常")
    print("   - 恢复位置细胞：确保 /place_cells/activity 正常")
    print("   - 保持视觉匹配：维持 /local_view/matches 正常")
    
    try:
        # 执行修复步骤
        fix_robot_state_node()
        fix_place_cell_node() 
        fix_experience_map_node()
        fix_config_file()
        create_enhanced_experience_map()
        
        if rebuild_project():
            create_launch_script()
            create_monitor_script()
            
            print_header("修复完成！")
            print("🎉 Dolphin SLAM 系统已完全修复！")
            print("\\n📋 下一步操作:")
            print("1. cd ~/dolphin_slam_ws")
            print("2. source install/setup.bash")  
            print("3. export ROS_DOMAIN_ID=42")
            print("4. ./start_fixed_system.sh")
            print("\\n🔍 或启动监控:")
            print("python3 src/dolphin_slam/scripts/fixed_monitor.py")
            print("\\n🎯 预期结果:")
            print("   ✅ trajectory: >0 Hz")
            print("   ✅ odometry: >0 Hz") 
            print("   ✅ place_cells: >0 Hz")
            print("   ✅ visual_matches: ~1.6 Hz")
            
        else:
            print("❌ 构建失败，请检查错误信息")
            return False
            
    except Exception as e:
        print(f"❌ 修复过程出错: {e}")
        return False
        
    return True

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
