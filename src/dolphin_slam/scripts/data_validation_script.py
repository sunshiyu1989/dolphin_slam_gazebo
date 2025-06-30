#!/usr/bin/env python3
"""
Dolphin SLAM 数据文件验证和修复脚本
检查数据集文件是否存在和格式是否正确
"""

import os
import pandas as pd
import numpy as np
from pathlib import Path

def main():
    print("🔍 Dolphin SLAM 数据验证工具")
    print("=" * 40)
    
    # 数据集基础路径
    base_path = "/media/psf/Samsung T7/SLAM Data/Sunboat_03-09-2023/2023-09-03-07-58-37"
    
    print(f"📂 检查数据集路径: {base_path}")
    
    if not os.path.exists(base_path):
        print(f"❌ 数据集基础路径不存在: {base_path}")
        print("请检查路径是否正确，或者数据集是否已挂载")
        return False
    
    print("✅ 数据集基础路径存在")
    
    # 检查各种可能的文件位置
    possible_nav_paths = [
        f"{base_path}/navigation/navigation.csv",
        f"{base_path}/navigation.csv", 
        f"{base_path}/navigation/nav.csv",
        f"{base_path}/nav.csv"
    ]
    
    nav_file = None
    print("\n📋 查找导航文件...")
    for path in possible_nav_paths:
        if os.path.exists(path):
            nav_file = path
            print(f"✅ 找到导航文件: {path}")
            break
        else:
            print(f"❌ 不存在: {path}")
    
    if not nav_file:
        print("\n❌ 未找到导航文件！")
        print("请检查以下位置是否有导航数据文件：")
        for path in possible_nav_paths:
            print(f"  - {path}")
        
        # 列出navigation目录的内容
        nav_dir = f"{base_path}/navigation"
        if os.path.exists(nav_dir):
            print(f"\n📁 {nav_dir} 目录内容：")
            for item in os.listdir(nav_dir):
                print(f"  - {item}")
        return False
    
    # 验证导航文件格式
    print(f"\n🔬 验证导航文件格式: {nav_file}")
    try:
        # 读取文件前几行来检查格式
        with open(nav_file, 'r') as f:
            lines = f.readlines()[:10]
        
        print(f"文件总行数: {len(open(nav_file).readlines())}")
        print("文件前5行预览:")
        for i, line in enumerate(lines[:5]):
            print(f"  {i+1}: {line.strip()}")
        
        # 尝试用pandas读取
        df = pd.read_csv(nav_file)
        print(f"\n✅ CSV 文件读取成功")
        print(f"数据形状: {df.shape}")
        print(f"列名: {list(df.columns)}")
        
        # 检查必需的列
        required_columns = ['timestamp', 'x', 'y', 'z']  # 根据实际需要调整
        missing_columns = []
        
        for col in required_columns:
            if col not in df.columns:
                # 尝试不同的可能列名
                possible_names = {
                    'timestamp': ['time', 'Time', 'TIMESTAMP', 't'],
                    'x': ['X', 'pos_x', 'position_x', 'longitude', 'lon'],
                    'y': ['Y', 'pos_y', 'position_y', 'latitude', 'lat'], 
                    'z': ['Z', 'pos_z', 'position_z', 'depth', 'altitude']
                }
                
                found = False
                if col in possible_names:
                    for alt_name in possible_names[col]:
                        if alt_name in df.columns:
                            print(f"🔄 找到替代列名: {col} -> {alt_name}")
                            found = True
                            break
                
                if not found:
                    missing_columns.append(col)
        
        if missing_columns:
            print(f"⚠️  缺少必需列: {missing_columns}")
            print("可用的列:", list(df.columns))
        else:
            print("✅ 所有必需列都存在")
        
        # 检查数据类型
        print("\n📊 数据类型检查:")
        for col in df.columns:
            dtype = df[col].dtype
            sample_value = df[col].iloc[0] if len(df) > 0 else "N/A"
            print(f"  {col}: {dtype} (示例: {sample_value})")
            
            # 检查数值列是否可以转换为float
            if col.lower() in ['x', 'y', 'z', 'longitude', 'latitude', 'depth']:
                try:
                    pd.to_numeric(df[col], errors='coerce')
                    print(f"    ✅ {col} 可以转换为数值")
                except:
                    print(f"    ❌ {col} 无法转换为数值")
    
    except Exception as e:
        print(f"❌ 读取导航文件失败: {e}")
        return False
    
    # 检查其他文件
    other_files = {
        'camera.csv': f"{base_path}/camera.csv",
        'sonar.csv': f"{base_path}/sonar.csv"
    }
    
    print(f"\n📁 检查其他数据文件...")
    for name, path in other_files.items():
        if os.path.exists(path):
            print(f"✅ {name}: 存在")
            try:
                df_temp = pd.read_csv(path)
                print(f"   形状: {df_temp.shape}, 列: {list(df_temp.columns)}")
            except Exception as e:
                print(f"   ❌ 读取失败: {e}")
        else:
            print(f"❌ {name}: 不存在于 {path}")
    
    # 检查图像和声呐目录
    media_dirs = {
        'camera': f"{base_path}/camera",
        'sonar': f"{base_path}/sonar"
    }
    
    print(f"\n🖼️  检查媒体文件目录...")
    for name, path in media_dirs.items():
        if os.path.exists(path):
            files = os.listdir(path)
            image_files = [f for f in files if f.lower().endswith(('.png', '.jpg', '.jpeg', '.tiff', '.bmp'))]
            print(f"✅ {name}: {len(image_files)} 个图像文件")
            if len(image_files) > 0:
                print(f"   示例: {image_files[0]}")
        else:
            print(f"❌ {name}: 目录不存在")
    
    print(f"\n🎯 生成修正后的配置...")
    
    # 生成配置建议
    config_suggestions = f"""
# 根据验证结果建议的配置：
robot_state_node:
  ros__parameters:
    navigation_csv: "{nav_file}"
    camera_csv: "{base_path}/camera.csv"
    sonar_csv: "{base_path}/sonar.csv"
    
image_processing_node:
  ros__parameters:
    camera_path: "{base_path}/camera"
    sonar_path: "{base_path}/sonar"
"""
    
    print(config_suggestions)
    
    # 保存配置到文件
    config_file = "verified_dolphin_slam_params.yaml"
    with open(config_file, 'w') as f:
        f.write(config_suggestions)
    
    print(f"✅ 配置建议已保存到: {config_file}")
    
    return True

if __name__ == "__main__":
    success = main()
    if success:
        print("\n🎉 数据验证完成！")
        print("💡 建议步骤：")
        print("1. 复制上面的配置到 dolphin_slam_params.yaml")
        print("2. 重新构建并启动系统")
        print("3. 检查数据类型是否匹配")
    else:
        print("\n❌ 数据验证失败")
        print("请检查数据集路径和文件格式")
