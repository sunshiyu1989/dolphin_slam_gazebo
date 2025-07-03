#!/usr/bin/env python3
"""
数据集诊断脚本 - 分析时间戳同步问题
"""

import pandas as pd
import numpy as np
import os
import matplotlib.pyplot as plt

def analyze_dataset(dataset_path):
    """分析数据集的时间戳同步情况"""
    
    print(f"🔍 分析数据集: {dataset_path}")
    print("=" * 60)
    
    # 文件路径
    files = {
        'navigation': os.path.join(dataset_path, 'navigation.csv'),
        'camera': os.path.join(dataset_path, 'camera.csv'),
        'sonar': os.path.join(dataset_path, 'sonar.csv')
    }
    
    # 加载数据
    data = {}
    for name, path in files.items():
        if os.path.exists(path):
            df = pd.read_csv(path)
            data[name] = df
            print(f"✅ {name}.csv: {len(df)} 条记录")
            print(f"   时间范围: {df['timestamp'].min():.2f} - {df['timestamp'].max():.2f}")
            print(f"   时间跨度: {df['timestamp'].max() - df['timestamp'].min():.2f} 秒")
            print()
        else:
            print(f"❌ 文件不存在: {path}")
    
    if len(data) < 2:
        print("❌ 数据文件不足，无法分析同步情况")
        return
    
    # 分析时间同步
    print("\n📊 时间同步分析")
    print("-" * 40)
    
    # 找出共同时间范围
    min_start = max([df['timestamp'].min() for df in data.values()])
    max_end = min([df['timestamp'].max() for df in data.values()])
    common_duration = max_end - min_start
    
    print(f"共同时间范围: {min_start:.2f} - {max_end:.2f}")
    print(f"共同时间跨度: {common_duration:.2f} 秒")
    print()
    
    # 分析每种数据在共同时间范围内的记录数
    print("📈 共同时间范围内的数据分布:")
    for name, df in data.items():
        common_data = df[(df['timestamp'] >= min_start) & (df['timestamp'] <= max_end)]
        print(f"  {name}: {len(common_data)} 条记录")
        
        # 计算数据频率
        if len(common_data) > 1:
            time_diffs = np.diff(common_data['timestamp'].values)
            avg_interval = np.mean(time_diffs)
            frequency = 1.0 / avg_interval if avg_interval > 0 else 0
            print(f"    平均间隔: {avg_interval:.3f} 秒")
            print(f"    平均频率: {frequency:.2f} Hz")
        print()
    
    # 检查数据缺失
    print("🔍 数据缺失分析:")
    for name, df in data.items():
        if 'filename' in df.columns:
            # 检查文件是否存在
            missing_files = 0
            total_files = 0
            for _, row in df.head(10).iterrows():  # 检查前10个文件
                filename = row['filename']
                if pd.notna(filename):
                    total_files += 1
                    file_path = os.path.join(dataset_path, name, filename)
                    if not os.path.exists(file_path):
                        missing_files += 1
            
            if total_files > 0:
                missing_rate = missing_files / total_files * 100
                print(f"  {name}: {missing_files}/{total_files} 文件缺失 ({missing_rate:.1f}%)")
        else:
            print(f"  {name}: 无 filename 列，无法检查文件")
    print()
    
    # 分析时间戳间隙
    print("⏱️  时间戳间隙分析:")
    for name, df in data.items():
        timestamps = df['timestamp'].values
        time_diffs = np.diff(timestamps)
        
        # 找出异常大的时间间隙
        median_diff = np.median(time_diffs)
        large_gaps = time_diffs > median_diff * 5  # 大于中位数5倍的间隙
        
        if np.any(large_gaps):
            gap_count = np.sum(large_gaps)
            max_gap = np.max(time_diffs[large_gaps])
            print(f"  {name}: 发现 {gap_count} 个大时间间隙，最大间隙: {max_gap:.2f} 秒")
        else:
            print(f"  {name}: 时间间隙正常")
    print()
    
    # 生成建议
    print("💡 修复建议:")
    print("-" * 30)
    
    # 检查是否需要数据裁剪
    full_ranges = [(name, df['timestamp'].min(), df['timestamp'].max()) for name, df in data.items()]
    full_ranges.sort(key=lambda x: x[1])  # 按开始时间排序
    
    latest_start = max([x[1] for x in full_ranges])
    earliest_end = min([x[2] for x in full_ranges])
    
    if latest_start > min([x[1] for x in full_ranges]):
        print(f"1. 建议裁剪数据到共同时间范围: {latest_start:.2f} - {earliest_end:.2f}")
        print(f"   这将确保所有数据类型都有对应的记录")
    
    # 检查频率差异
    frequencies = {}
    for name, df in data.items():
        if len(df) > 1:
            time_span = df['timestamp'].max() - df['timestamp'].min()
            freq = len(df) / time_span if time_span > 0 else 0
            frequencies[name] = freq
    
    if len(frequencies) > 1:
        freq_values = list(frequencies.values())
        if max(freq_values) / min(freq_values) > 2:  # 频率差异大于2倍
            print("2. 检测到数据频率差异较大，建议:")
            for name, freq in frequencies.items():
                print(f"   {name}: {freq:.2f} Hz")
            print("   考虑使用插值或降采样来统一频率")
    
    print("3. 建议在 dataset_player_node 中添加数据结束信号")
    print("4. 建议修复 TF 坐标变换配置")

if __name__ == "__main__":
    # 你的数据集路径
    dataset_path = "/media/psf/Samsung T7/SLAM Data/Sunboat_03-09-2023/2023-09-03-07-58-37"
    analyze_dataset(dataset_path)