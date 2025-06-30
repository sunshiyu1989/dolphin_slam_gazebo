#!/usr/bin/env python3
"""
Dolphin SLAM - 结果可视化工具
可视化 SLAM 运行结果，包括轨迹、地图、特征等
"""

import argparse
import os
import pickle
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, FancyBboxPatch
from matplotlib.collections import LineCollection
import pandas as pd
import cv2
from datetime import datetime
import json

class ResultVisualizer:
    """SLAM 结果可视化器"""
    
    def __init__(self, result_dir):
        self.result_dir = result_dir
        self.figures = []
        
    def visualize_all(self):
        """生成所有可视化"""
        print(f"可视化结果目录: {self.result_dir}")
        
        # 检查可用数据
        available_data = self.check_available_data()
        
        if available_data['experience_map']:
            self.visualize_experience_map()
            
        if available_data['trajectory']:
            self.visualize_trajectory()
            
        if available_data['features']:
            self.visualize_features()
            
        if available_data['place_cells']:
            self.visualize_place_cells()
            
        if available_data['statistics']:
            self.visualize_statistics()
            
        # 保存所有图表
        self.save_all_figures()
        
    def check_available_data(self):
        """检查可用的数据文件"""
        available = {
            'experience_map': False,
            'trajectory': False,
            'features': False,
            'place_cells': False,
            'statistics': False
        }
        
        # 检查经验地图
        map_files = [f for f in os.listdir(self.result_dir) 
                    if f.startswith('experience_map') and f.endswith('.pkl')]
        if map_files:
            self.map_file = os.path.join(self.result_dir, sorted(map_files)[-1])
            available['experience_map'] = True
            print(f"  找到经验地图: {self.map_file}")
            
        # 检查轨迹数据
        traj_file = os.path.join(self.result_dir, 'trajectory.csv')
        if os.path.exists(traj_file):
            self.trajectory_file = traj_file
            available['trajectory'] = True
            print(f"  找到轨迹数据: {traj_file}")
            
        # 检查特征数据
        feature_dir = os.path.join(self.result_dir, 'features')
        if os.path.isdir(feature_dir):
            self.feature_dir = feature_dir
            available['features'] = True
            print(f"  找到特征数据目录: {feature_dir}")
            
        # 检查位置细胞数据
        pc_file = os.path.join(self.result_dir, 'place_cells.npy')
        if os.path.exists(pc_file):
            self.pc_file = pc_file
            available['place_cells'] = True
            print(f"  找到位置细胞数据: {pc_file}")
            
        # 检查统计数据
        stats_file = os.path.join(self.result_dir, 'statistics.json')
        if os.path.exists(stats_file):
            self.stats_file = stats_file
            available['statistics'] = True
            print(f"  找到统计数据: {stats_file}")
            
        return available
        
    def visualize_experience_map(self):
        """可视化经验地图"""
        print("\n可视化经验地图...")
        
        # 加载地图
        with open(self.map_file, 'rb') as f:
            map_data = pickle.load(f)
            
        experiences = map_data['experiences']
        connections = map_data['connections']
        
        # 创建图形
        fig = plt.figure(figsize=(12, 10))
        ax = fig.add_subplot(111, projection='3d')
        
        # 绘制经验节点
        positions = np.array([[exp.x, exp.y, exp.z] 
                            for exp in experiences.values()])
        
        if len(positions) > 0:
            ax.scatter(positions[:, 0], positions[:, 1], positions[:, 2],
                      c=range(len(positions)), cmap='viridis',
                      s=50, alpha=0.6, edgecolors='black')
            
            # 绘制连接
            for exp1_id, neighbors in connections.items():
                if exp1_id in experiences:
                    exp1 = experiences[exp1_id]
                    for exp2_id in neighbors:
                        if exp2_id in experiences:
                            exp2 = experiences[exp2_id]
                            ax.plot([exp1.x, exp2.x],
                                   [exp1.y, exp2.y],
                                   [exp1.z, exp2.z],
                                   'b-', alpha=0.3, linewidth=0.5)
                                   
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        ax.set_title(f'Experience Map - {len(experiences)} nodes')
        
        # 添加统计信息
        info_text = (f"Nodes: {len(experiences)}\n"
                    f"Connections: {sum(len(n) for n in connections.values())//2}\n"
                    f"Loop closures: {map_data.get('loop_closures_detected', 0)}")
        ax.text2D(0.02, 0.98, info_text, transform=ax.transAxes,
                 verticalalignment='top', bbox=dict(boxstyle='round', 
                 facecolor='wheat', alpha=0.5))
        
        self.figures.append(('experience_map_3d.png', fig))
        
        # 2D 俯视图
        fig2, ax2 = plt.subplots(figsize=(10, 10))
        
        if len(positions) > 0:
            # 节点
            scatter = ax2.scatter(positions[:, 0], positions[:, 1],
                                c=range(len(positions)), cmap='viridis',
                                s=100, alpha=0.6, edgecolors='black')
            
            # 连接
            for exp1_id, neighbors in connections.items():
                if exp1_id in experiences:
                    exp1 = experiences[exp1_id]
                    for exp2_id in neighbors:
                        if exp2_id in experiences and exp2_id > exp1_id:
                            exp2 = experiences[exp2_id]
                            ax2.plot([exp1.x, exp2.x], [exp1.y, exp2.y],
                                   'b-', alpha=0.3, linewidth=1)
                                   
            # 标注起点和终点
            ax2.plot(positions[0, 0], positions[0, 1], 'go', 
                    markersize=15, label='Start')
            ax2.plot(positions[-1, 0], positions[-1, 1], 'ro', 
                    markersize=15, label='End')
            
            plt.colorbar(scatter, ax=ax2, label='Experience ID')
            
        ax2.set_xlabel('X (m)')
        ax2.set_ylabel('Y (m)')
        ax2.set_title('Experience Map - Top View')
        ax2.axis('equal')
        ax2.grid(True, alpha=0.3)
        ax2.legend()
        
        self.figures.append(('experience_map_2d.png', fig2))
        
    def visualize_trajectory(self):
        """可视化机器人轨迹"""
        print("\n可视化轨迹...")
        
        # 加载轨迹数据
        traj_df = pd.read_csv(self.trajectory_file)
        
        fig, axes = plt.subplots(2, 2, figsize=(12, 10))
        
        # XY 轨迹
        ax = axes[0, 0]
        ax.plot(traj_df['x'], traj_df['y'], 'b-', linewidth=1)
        ax.plot(traj_df['x'].iloc[0], traj_df['y'].iloc[0], 'go', 
               markersize=10, label='Start')
        ax.plot(traj_df['x'].iloc[-1], traj_df['y'].iloc[-1], 'ro', 
               markersize=10, label='End')
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_title('XY Trajectory')
        ax.axis('equal')
        ax.grid(True)
        ax.legend()
        
        # 深度剖面
        ax = axes[0, 1]
        if 'timestamp' in traj_df.columns:
            ax.plot(traj_df['timestamp'], traj_df['z'], 'r-')
        else:
            ax.plot(traj_df['z'], 'r-')
        ax.set_xlabel('Time')
        ax.set_ylabel('Z (m)')
        ax.set_title('Depth Profile')
        ax.grid(True)
        ax.invert_yaxis()
        
        # 速度
        ax = axes[1, 0]
        if all(col in traj_df.columns for col in ['vx', 'vy', 'vz']):
            speeds = np.sqrt(traj_df['vx']**2 + traj_df['vy']**2 + traj_df['vz']**2)
            ax.plot(speeds, 'g-')
            ax.set_xlabel('Sample')
            ax.set_ylabel('Speed (m/s)')
            ax.set_title('Vehicle Speed')
            ax.grid(True)
            
        # 偏航角
        ax = axes[1, 1]
        if 'yaw' in traj_df.columns:
            ax.plot(np.degrees(traj_df['yaw']), 'c-')
            ax.set_xlabel('Sample')
            ax.set_ylabel('Yaw (degrees)')
            ax.set_title('Vehicle Heading')
            ax.grid(True)
            
        plt.tight_layout()
        self.figures.append(('trajectory_analysis.png', fig))
        
    def visualize_features(self):
        """可视化特征提取结果"""
        print("\n可视化特征...")
        
        # 找几个示例图像
        feature_files = [f for f in os.listdir(self.feature_dir) 
                        if f.endswith('_features.jpg')][:4]
        
        if feature_files:
            fig, axes = plt.subplots(2, 2, figsize=(12, 10))
            axes = axes.ravel()
            
            for i, fname in enumerate(feature_files):
                img_path = os.path.join(self.feature_dir, fname)
                img = cv2.imread(img_path)
                if img is not None:
                    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                    axes[i].imshow(img_rgb)
                    axes[i].set_title(f'Features - {fname}')
                    axes[i].axis('off')
                    
            plt.tight_layout()
            self.figures.append(('feature_examples.png', fig))
            
    def visualize_place_cells(self):
        """可视化位置细胞活动"""
        print("\n可视化位置细胞...")
        
        # 加载位置细胞数据
        pc_data = np.load(self.pc_file)
        
        # 如果是 3D 数据，取一个切片
        if pc_data.ndim == 3:
            # 取中间切片
            slice_idx = pc_data.shape[2] // 2
            pc_slice = pc_data[:, :, slice_idx]
        else:
            pc_slice = pc_data
            
        fig, ax = plt.subplots(figsize=(10, 8))
        
        im = ax.imshow(pc_slice, cmap='hot', interpolation='bilinear')
        ax.set_xlabel('X cells')
        ax.set_ylabel('Y cells')
        ax.set_title('Place Cell Activity (Z slice)')
        
        plt.colorbar(im, ax=ax, label='Activity')
        
        # 标记峰值位置
        peak_idx = np.unravel_index(np.argmax(pc_slice), pc_slice.shape)
        circle = Circle(peak_idx[::-1], radius=2, fill=False, 
                       edgecolor='red', linewidth=2)
        ax.add_patch(circle)
        
        self.figures.append(('place_cells_activity.png', fig))
        
    def visualize_statistics(self):
        """可视化统计数据"""
        print("\n可视化统计...")
        
        with open(self.stats_file, 'r') as f:
            stats = json.load(f)
            
        fig, axes = plt.subplots(2, 2, figsize=(12, 10))
        
        # 经验创建时间线
        ax = axes[0, 0]
        if 'experience_creation_times' in stats:
            times = stats['experience_creation_times']
            ax.plot(times, range(len(times)), 'b-')
            ax.set_xlabel('Time (s)')
            ax.set_ylabel('Experience ID')
            ax.set_title('Experience Creation Timeline')
            ax.grid(True)
            
        # 模板匹配率
        ax = axes[0, 1]
        if 'template_match_rate' in stats:
            windows = stats['template_match_rate']['windows']
            rates = stats['template_match_rate']['rates']
            ax.plot(windows, rates, 'g-')
            ax.set_xlabel('Time Window')
            ax.set_ylabel('Match Rate')
            ax.set_title('Visual Template Match Rate')
            ax.grid(True)
            ax.set_ylim([0, 1])
            
        # 闭环检测
        ax = axes[1, 0]
        if 'loop_closures' in stats:
            lc_times = [lc['time'] for lc in stats['loop_closures']]
            lc_ids = [lc['experience_id'] for lc in stats['loop_closures']]
            ax.scatter(lc_times, lc_ids, c='red', s=100, alpha=0.6)
            ax.set_xlabel('Time (s)')
            ax.set_ylabel('Experience ID')
            ax.set_title(f'Loop Closures ({len(lc_times)} detected)')
            ax.grid(True)
            
        # 性能指标
        ax = axes[1, 1]
        if 'performance' in stats:
            metrics = stats['performance']
            labels = list(metrics.keys())
            values = list(metrics.values())
            
            y_pos = np.arange(len(labels))
            ax.barh(y_pos, values)
            ax.set_yticks(y_pos)
            ax.set_yticklabels(labels)
            ax.set_xlabel('Time (ms)')
            ax.set_title('Processing Time per Module')
            ax.grid(True, axis='x')
            
        plt.tight_layout()
        self.figures.append(('statistics.png', fig))
        
    def save_all_figures(self):
        """保存所有图表"""
        output_dir = os.path.join(self.result_dir, 'visualizations')
        os.makedirs(output_dir, exist_ok=True)
        
        for filename, fig in self.figures:
            output_path = os.path.join(output_dir, filename)
            fig.savefig(output_path, dpi=150, bbox_inches='tight')
            print(f"  保存: {output_path}")
            plt.close(fig)
            
        # 生成 HTML 报告
        self.generate_html_report(output_dir)
        
    def generate_html_report(self, output_dir):
        """生成 HTML 报告"""
        html_content = f"""
<!DOCTYPE html>
<html>
<head>
    <title>Dolphin SLAM Results</title>
    <style>
        body {{
            font-family: Arial, sans-serif;
            margin: 20px;
            background-color: #f5f5f5;
        }}
        h1 {{
            color: #333;
            text-align: center;
        }}
        .container {{
            max-width: 1200px;
            margin: 0 auto;
            background-color: white;
            padding: 20px;
            box-shadow: 0 0 10px rgba(0,0,0,0.1);
        }}
        .image-grid {{
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(500px, 1fr));
            gap: 20px;
            margin-top: 20px;
        }}
        .image-item {{
            text-align: center;
        }}
        .image-item img {{
            max-width: 100%;
            height: auto;
            border: 1px solid #ddd;
        }}
        .timestamp {{
            text-align: center;
            color: #666;
            margin-bottom: 20px;
        }}
    </style>
</head>
<body>
    <div class="container">
        <h1>Dolphin SLAM Visualization Results</h1>
        <p class="timestamp">Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}</p>
        
        <div class="image-grid">
"""
        
        # 添加所有图像
        for filename, _ in self.figures:
            html_content += f"""
            <div class="image-item">
                <h3>{filename.replace('_', ' ').replace('.png', '').title()}</h3>
                <img src="{filename}" alt="{filename}">
            </div>
"""
            
        html_content += """
        </div>
    </div>
</body>
</html>
"""
        
        html_path = os.path.join(output_dir, 'report.html')
        with open(html_path, 'w') as f:
            f.write(html_content)
            
        print(f"\nHTML 报告已生成: {html_path}")

def main():
    parser = argparse.ArgumentParser(description='可视化 Dolphin SLAM 结果')
    parser.add_argument('result_dir', help='结果目录路径')
    parser.add_argument('--format', choices=['png', 'pdf', 'svg'], 
                       default='png', help='输出格式')
    
    args = parser.parse_args()
    
    # 检查路径
    if not os.path.exists(args.result_dir):
        print(f"错误：结果目录不存在: {args.result_dir}")
        return 1
        
    # 执行可视化
    visualizer = ResultVisualizer(args.result_dir)
    visualizer.visualize_all()
    
    return 0

if __name__ == '__main__':
    exit(main())
