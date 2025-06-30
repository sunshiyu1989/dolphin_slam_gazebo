# Dolphin SLAM - Python ROS2 实现

生物启发的水下 SLAM 系统，使用神经网络进行空间表征。

## 系统架构

```
┌─────────────────┐     ┌──────────────────┐     ┌─────────────────┐
│  Image Processing│────▶│ Local View Cells │────▶│Experience Map   │
│  (SURF/SIFT)    │     │   (FAB-MAP)      │     │(Topological Map)│
└─────────────────┘     └──────────────────┘     └─────────────────┘
         │                       │                         ▲
         │                       │                         │
         │                       ▼                         │
         │              ┌──────────────────┐              │
         └─────────────▶│Place Cell Network│──────────────┘
                        │     (CANN)       │
                        └──────────────────┘
                                ▲
                                │
                        ┌──────────────────┐
                        │  Robot State     │
                        │ (Sensor Fusion)  │
                        └──────────────────┘
```

## 快速开始

### 1. 环境设置

```bash
# 创建工作空间
mkdir -p ~/dolphin_slam_ws/src
cd ~/dolphin_slam_ws/src

# 克隆或复制项目文件
# 将所有文件放入 dolphin_slam/ 目录

# 安装 Python 依赖
cd dolphin_slam
pip install -r requirements.txt

# 安装 ROS2 依赖
sudo apt update
sudo apt install ros-humble-desktop python3-colcon-common-extensions
```

### 2. 构建项目

```bash
cd ~/dolphin_slam_ws
colcon build --symlink-install
source install/setup.bash
```

### 3. 配置数据集

编辑 `config/dolphin_slam_params.yaml`：

```yaml
dataset:
  base_path: "/path/to/your/AUV-Dataset"
  camera_path: "/path/to/your/AUV-Dataset/camera"
  sonar_path: "/path/to/your/AUV-Dataset/sonar"
  navigation_csv: "/path/to/your/AUV-Dataset/navigation.csv"
```

### 4. 运行系统

```bash
# 使用便捷脚本
./run_dolphin_slam.sh -d /path/to/AUV-Dataset

# 或手动启动
ros2 launch dolphin_slam dolphin_slam_launch.py dataset_path:=/path/to/AUV-Dataset
```

## 主要功能

### 🎯 特征提取
- SURF/SIFT/ORB 特征检测
- 自适应对比度增强（CLAHE）
- 声呐图像专用处理

### 🧠 生物启发模块
- **位置细胞网络**：3D 竞争性吸引子神经网络
- **局部视觉细胞**：FAB-MAP 概率场景识别
- **经验地图**：拓扑-度量混合表示

### 📊 可视化
- RViz2 实时 3D 显示
- 特征点检测结果
- 位置细胞活动热图
- 拓扑地图和轨迹

### 💾 数据管理
- 自动保存地图和模板
- 支持地图加载/恢复
- ROS bag 录制支持

## 节点说明

| 节点 | 功能 | 主要话题 |
|------|------|----------|
| image_processing_node | 特征提取 | /camera/image_raw → /features/descriptors |
| local_view_node | 场景识别 | /features/descriptors → /local_view/matches |
| place_cell_node | 空间表征 | /robot/odometry → /place_cells/activity |
| experience_map_node | 地图构建 | /place_cells/activity → /experience_map/markers |
| robot_state_node | 状态估计 | navigation.csv → /robot/odometry |
| dataset_player_node | 数据播放 | 文件 → /camera/image_raw, /sonar/image_raw |

## 参数调优

### ARM64 (M2) 优化

```yaml
# 降低计算负载
image_processing:
  max_features: 500  # 从 1000 降低
  
place_cell:
  neurons_per_dimension: 16  # 从 20 降低
  
performance:
  reduce_features: true
  reduced_network_size: 16
```

### 实时性能

```yaml
# 调整处理频率
image_processing:
  process_every_n_frames: 2  # 隔帧处理
  
place_cell:
  update_rate: 5.0  # 降低更新频率
```

## 常见问题

### Q: OpenCV 找不到 SURF/SIFT？
A: 安装 opencv-contrib-python：
```bash
pip install opencv-contrib-python
```

### Q: 词汇表未加载？
A: 首次运行需要训练词汇表：
```bash
# 启动系统后
ros2 service call /local_view/train_vocabulary std_srvs/srv/Trigger
```

### Q: 内存使用过高？
A: 调整参数：
- 减少 max_templates
- 降低 neurons_per_dimension
- 启用 map_pruning

### Q: 如何使用自己的数据？
A: 确保数据格式匹配：
- camera/: 图像文件 (PNG/JPG)
- sonar/: 声呐图像
- navigation.csv: 包含必需列的 CSV
- camera.csv, sonar.csv: 时间戳映射

## 扩展开发

### 添加新的特征提取器

```python
# 在 image_processing.py 中
def _init_feature_detector(self):
    if self.feature_type == 'YOUR_DETECTOR':
        self.detector = YourDetector()
```

### 实现新的神经网络模型

```python
# 继承 PlaceCellNetwork
class YourNetwork(PlaceCellNetwork):
    def apply_recurrent_dynamics(self):
        # 自定义动力学
        pass
```

### 集成其他传感器

```python
# 在 robot_state_node.py 中添加订阅
self.your_sensor_sub = self.create_subscription(
    YourSensorMsg,
    '/your_sensor/data',
    self.your_sensor_callback,
    10
)
```

## 性能指标

典型性能（Intel i7 + 16GB RAM）：
- 图像处理：10 FPS
- 特征匹配：< 50ms
- 位置细胞更新：< 10ms
- 地图更新：< 20ms

## 引用

如果您使用此代码，请引用：
```
@article{dolphin_slam,
  title={Bio-inspired SLAM for Underwater Vehicles},
  author={Your Name},
  journal={Your Journal},
  year={2024}
}
```

## 许可证

Apache License 2.0

## 贡献

欢迎提交 Issue 和 Pull Request！

## 致谢

- 原始 Dolphin SLAM 算法
- FAB-MAP 实现
- ROS2 社区
