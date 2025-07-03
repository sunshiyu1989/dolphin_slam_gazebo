#!/usr/bin/env python3
"""
Dolphin SLAM 最终修复脚本
========================

修复最后两个小问题：
1. place_cell_node.py: Path导入问题
2. robot_state_node.py: navigation.csv列名问题 (应该是latitude/longitude/depth，不是x/y/z)
"""

import os
import shutil
from pathlib import Path

def fix_place_cell_node():
    """修复place_cell_node.py的Path导入问题"""
    print("🔧 修复 place_cell_node.py - Path导入问题...")
    
    file_path = Path("~/dolphin_slam_ws/src/dolphin_slam/nodes/place_cell_node.py").expanduser()
    
    if not file_path.exists():
        print(f"❌ 文件不存在: {file_path}")
        return False
    
    # 读取文件内容
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()
    
    # 添加缺失的导入
    if 'from pathlib import Path' not in content:
        # 在其他导入后添加pathlib导入
        import_section = """import sys
import os
import traceback
from pathlib import Path"""
        
        content = content.replace(
            """import sys
import os
import traceback""",
            import_section
        )
        
        # 写回文件
        with open(file_path, 'w', encoding='utf-8') as f:
            f.write(content)
        
        print("✅ 已添加 'from pathlib import Path' 导入")
        return True
    else:
        print("✅ Path导入已存在")
        return True

def fix_robot_state_node():
    """修复robot_state_node.py的CSV列名检查"""
    print("🔧 修复 robot_state_node.py - CSV列名问题...")
    
    file_path = Path("~/dolphin_slam_ws/src/dolphin_slam/nodes/robot_state_node.py").expanduser()
    
    if not file_path.exists():
        print(f"❌ 文件不存在: {file_path}")
        return False
    
    # 读取文件内容
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()
    
    # 修复列名检查
    old_columns = "required_columns = ['timestamp', 'x', 'y', 'z', 'roll', 'pitch', 'yaw']"
    new_columns = "required_columns = ['timestamp', 'latitude', 'longitude', 'depth', 'roll', 'pitch', 'yaw']"
    
    if old_columns in content:
        content = content.replace(old_columns, new_columns)
        
        # 同时修复状态更新逻辑
        old_pose_update = '''self.current_pose = {
                    'x': float(row['x']),
                    'y': float(row['y']), 
                    'z': float(row['z']),'''
        
        new_pose_update = '''# 转换经纬度到局部坐标（简化版本）
                if not hasattr(self, 'origin_lat'):
                    self.origin_lat = float(row['latitude'])
                    self.origin_lon = float(row['longitude'])
                    
                # 简化的经纬度转换（适用于小范围）
                R_earth = 6371000  # 地球半径（米）
                lat_rad = np.radians(float(row['latitude']))
                origin_lat_rad = np.radians(self.origin_lat)
                
                x = R_earth * np.radians(float(row['longitude']) - self.origin_lon) * np.cos(origin_lat_rad)
                y = R_earth * np.radians(float(row['latitude']) - self.origin_lat)
                z = -float(row['depth'])  # 深度为负值
                
                self.current_pose = {
                    'x': x,
                    'y': y, 
                    'z': z,'''
        
        if old_pose_update in content:
            content = content.replace(old_pose_update, new_pose_update)
        
        # 写回文件
        with open(file_path, 'w', encoding='utf-8') as f:
            f.write(content)
        
        print("✅ 已修复CSV列名检查和坐标转换")
        return True
    else:
        print("✅ CSV列名检查已正确")
        return True

def rebuild_project():
    """重新构建项目"""
    print("🔨 重新构建项目...")
    
    workspace_dir = Path("~/dolphin_slam_ws").expanduser()
    os.chdir(workspace_dir)
    
    # 快速构建
    import subprocess
    try:
        result = subprocess.run(
            ['colcon', 'build', '--packages-select', 'dolphin_slam'],
            capture_output=True,
            text=True,
            cwd=workspace_dir
        )
        
        if result.returncode == 0:
            print("✅ 项目构建成功！")
            return True
        else:
            print(f"❌ 构建失败: {result.stderr}")
            return False
            
    except FileNotFoundError:
        print("❌ 未找到 colcon 命令")
        return False

def main():
    """主函数"""
    print("🚀 Dolphin SLAM 最终修复")
    print("=" * 40)
    
    success_count = 0
    
    # 修复place_cell_node.py
    if fix_place_cell_node():
        success_count += 1
    
    # 修复robot_state_node.py
    if fix_robot_state_node():
        success_count += 1
    
    # 重新构建
    if success_count == 2:
        if rebuild_project():
            print("\\n🎉 最终修复完成！")
            print("=" * 30)
            print("✅ 修复内容:")
            print("  1. place_cell_node.py - 添加Path导入")
            print("  2. robot_state_node.py - 修正CSV列名 (latitude/longitude/depth)")
            print("  3. 添加经纬度到XYZ坐标转换")
            print("\\n🚀 现在重新测试:")
            print("  source install/setup.bash")
            print("  ./start_dolphin.sh")
            print("\\n📋 预期结果:")
            print("  ✅ ✅ 成功导入PlaceCellNetwork!")
            print("  ✅ 导航数据正常加载")
            print("  ✅ 神经元活跃度正常")
            print("  ✅ 时间同步正常")
            return True
        else:
            print("\\n❌ 构建失败")
            return False
    else:
        print("\\n❌ 修复失败")
        return False

if __name__ == "__main__":
    success = main()
    if success:
        print("\\n✨ 所有问题已修复！重新启动测试吧！")
    else:
        print("\\n💡 如有问题请检查错误信息")
