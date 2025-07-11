# Dolphin SLAM - 生物启发的自主水下航行器SLAM系统

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![Python](https://img.shields.io/badge/Python-3.10+-green.svg)](https://www.python.org/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Simulation-orange.svg)](http://gazebosim.org/)
[![License](https://img.shields.io/badge/License-Apache%202.0-red.svg)](LICENSE)

## 📖 项目概述

Dolphin SLAM 是一个基于生物启发算法的自主水下航行器（AUV）同时定位与地图构建（SLAM）系统。该项目模拟海豚的空间认知机制，结合竞争性吸引子神经网络（CANN）和视觉场景识别，实现水下环境中的自主导航和地图构建。

### 🎯 核心特性

- **生物启发算法**: 基于海豚空间认知机制的神经网络架构
- **多传感器融合**: 集成相机、声纳、IMU、DVL、GPS等传感器
- **实时SLAM**: 支持实时位置估计和地图构建
- **水下环境优化**: 专门针对水下环境的视觉处理算法
- **ROS2集成**: 完整的ROS2节点架构，支持分布式计算
- **Gazebo仿真**: 高保真的水下环境仿真测试平台

## 🧠 算法架构与原理

### 1. 整体架构

Dolphin SLAM 采用分层架构设计，主要包含以下核心模块：

```
┌─────────────────────────────────────────────────────────────┐
│                    Dolphin SLAM 系统架构                      │
├─────────────────────────────────────────────────────────────┤
│  传感器层  │  相机  │  声纳  │  IMU  │  DVL  │  GPS  │
├─────────────────────────────────────────────────────────────┤
│  数据融合层  │  机器人状态估计  │  传感器同步  │  数据预处理  │
├─────────────────────────────────────────────────────────────┤
│  SLAM核心层  │  位置细胞网络  │  局部视觉细胞网络  │  经验地图  │
├─────────────────────────────────────────────────────────────┤
│  控制层     │  航点控制器  │  轨迹规划  │  运动控制  │
├─────────────────────────────────────────────────────────────┤
│  可视化层   │  RViz显示  │  轨迹评估  │  性能监控  │
└─────────────────────────────────────────────────────────────┘
```

### 2. 核心算法模块

#### 2.1 位置细胞网络 (Place Cell Network)

**原理**: 模拟海豚大脑中的空间认知系统，使用竞争性吸引子神经网络（CANN）维持位置估计。

**关键特性**:
- **Mexican Hat连接模式**: 相近神经元相互激活，远处神经元相互抑制
- **路径积分**: 基于运动传感器进行位置预测
- **视觉校正**: 通过视觉输入修正位置估计
- **3D空间支持**: 支持水下三维空间定位

```python
# 核心参数
neurons_per_dim = 20      # 每维度神经元数量
grid_step = 0.2          # 空间分辨率
recurrent_conn_std = 3.0  # 递归连接标准差
```

#### 2.2 局部视觉细胞 (Local View Cells)

**原理**: 基于FAB-MAP算法进行场景识别，管理视觉模板库。

**关键特性**:
- **视觉词汇表**: 使用K-means聚类构建视觉词汇
- **BoW描述符**: 基于词袋模型的场景表示
- **模板匹配**: 支持FAB-MAP和传统BoW匹配
- **水下环境优化**: 针对水下图像质量进行优化

```python
# 核心参数
vocabulary_size = 1000    # 视觉词汇表大小
similarity_threshold = 0.65  # 匹配阈值
max_templates = 5000      # 最大模板数量
```

#### 2.3 经验地图 (Experience Map)

**原理**: 构建拓扑-度量混合地图，管理位置节点和连接关系。

**关键特性**:
- **拓扑结构**: 维护位置节点间的连接关系
- **度量信息**: 存储精确的位置坐标
- **闭环检测**: 自动检测和修正闭环
- **路径规划**: 支持基于拓扑的路径规划

```python
# 核心参数
match_threshold = 0.75    # 经验匹配阈值
lv_factor = 0.5          # 视觉权重因子
pc_factor = 0.5          # 位置细胞权重因子
```

### 3. 算法数据流

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   传感器数据     │───▶│   图像处理节点   │───▶│   特征描述符     │
│  (相机/声纳)     │    │  (SIFT特征提取)  │    │  (BoW描述符)     │
└─────────────────┘    └─────────────────┘    └─────────────────┘
         │                       │                       │
         ▼                       ▼                       ▼
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   运动传感器     │    │   局部视觉细胞   │    │   视觉模板匹配   │
│  (IMU/DVL)      │    │   网络管理      │    │  (FAB-MAP算法)   │
└─────────────────┘    └─────────────────┘    └─────────────────┘
         │                       │                       │
         ▼                       ▼                       ▼
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   路径积分      │    │   位置细胞网络   │    │   经验地图构建   │
│  (运动预测)     │───▶│  (CANN算法)     │───▶│  (拓扑-度量)     │
└─────────────────┘    └─────────────────┘    └─────────────────┘
         │                       │                       │
         ▼                       ▼                       ▼
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   机器人状态     │    │   位置估计      │    │   轨迹输出      │
│   融合与估计     │    │   (3D坐标)      │    │   (RViz显示)    │
└─────────────────┘    └─────────────────┘    └─────────────────┘
```

## 🤖 仿真模块与构建

### 1. 机器人模型

**AUV设计**: 基于真实水下航行器设计的仿真模型

- **船体**: 2米长圆柱形主体，橙色外观
- **传感器配置**:
  - 前视相机 (船头位置，无遮挡视野)
  - 侧扫声纳 (船体中部)
  - IMU (船体中心)
  - DVL (船体底部)
  - GPS (船体顶部)
- **推进系统**: 主推进器和控制鳍
- **物理参数**: 25kg质量，合理的惯性特性

### 2. 仿真环境

**水下世界**: 基于Gazebo的高保真水下环境

- **地形**: 海底地形、礁石、沉船等障碍物
- **流体动力学**: 真实的水下物理特性
- **光照**: 水下光照衰减和散射效果
- **传感器噪声**: 模拟真实传感器噪声

### 3. ROS2节点架构

```
dolphin_slam/
├── image_processing_node      # 图像特征提取
├── local_view_node           # 视觉场景识别
├── place_cell_node           # 位置细胞网络
├── experience_map_node       # 经验地图构建
├── robot_state_node          # 机器人状态估计
├── simple_odom_publisher     # 里程计发布
├── trajectory_evaluator      # 轨迹评估
├── enhanced_waypoint_controller  # 航点控制
└── unified_data_controller   # 数据统一管理
```

## 🚀 运行指南

### 1. 环境要求

```bash
# 系统要求
- Ubuntu 22.04 LTS
- ROS2 Humble
- Python 3.10+
- Gazebo Classic

# 依赖安装
sudo apt update
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-plugins
pip install -r requirements.txt
```

### 2. 构建项目

```bash
# 克隆项目
git clone https://github.com/sunshiyu1989/dolphin_slam_gazebo.git
cd dolphin_slam_ws

# 构建
colcon build --packages-select dolphin_slam
source install/setup.bash
```

### 3. 启动仿真

```bash
# 一键启动完整系统
./start_dolphin_slam_ultimate.sh

# 或分步启动
ros2 launch dolphin_slam dolphin_slam_sim_launch.py
```

### 4. 控制机器人

```bash
# 发布速度命令
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# 发布航点
ros2 topic pub /waypoints geometry_msgs/msg/PoseArray "poses: [{position: {x: 10.0, y: 5.0, z: -2.0}}]"
```

### 5. 可视化监控

```bash
# 启动RViz (分屏布局)
ros2 run rviz2 rviz2 -d src/dolphin_slam/rviz/dolphin_slam_simple.rviz

# 查看话题
ros2 topic list
ros2 topic echo /place_cells/activity
ros2 topic echo /experience_map/experiences

# 清理历史轨迹
python3 src/dolphin_slam/scripts/force_clear_trajectory.py
```

**RViz分屏布局说明**:
- **左侧视图**: 前向相机实时图像显示
- **右侧视图**: 3D机器人模型和运动轨迹
- **轨迹显示**: 绿色轨迹线，1000点缓冲区
- **界面优化**: 隐藏侧边栏，最大化视图区域

## 🔧 调试指南

### 1. 日志级别调整

```bash
# 设置节点日志级别
ros2 run dolphin_slam place_cell_node --ros-args --log-level DEBUG
```

### 2. 参数调优

编辑 `config/dolphin_slam_params.yaml` 调整算法参数：

```yaml
# 位置细胞网络参数
place_cell_node:
  ros__parameters:
    neurons_per_dimension: 16      # 神经元维度
    spatial_scale: 30.0           # 空间尺度(米)
    visual_similarity_threshold: 0.7  # 视觉相似度阈值
    excitation_radius: 1.3        # 兴奋半径
    inhibition_strength: 0.3      # 抑制强度
    decay_rate: 0.12              # 衰减率

# 局部视觉细胞参数
local_view_node:
  ros__parameters:
    similarity_threshold: 0.5     # 相似度阈值
    max_templates: 25             # 最大模板数
    underwater_mode: true         # 水下模式
    frame_skip_threshold: 0.9     # 帧跳过阈值

# 图像处理参数
image_processing_node:
  ros__parameters:
    feature_type: "SIFT"          # 特征类型
    max_features: 200             # 最大特征数
    sift_contrastThreshold: 0.03  # SIFT对比度阈值
    sift_edgeThreshold: 20        # SIFT边缘阈值
```

### 3. 性能监控

```bash
# 查看系统资源
htop

# 监控ROS话题频率
ros2 topic hz /forward_camera/image_raw
ros2 topic hz /place_cells/activity

# 录制数据包
ros2 bag record -a -o dolphin_slam_test
```

### 4. 常见问题解决

**RViz启动失败**:
```bash
# 设置OpenGL环境变量
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_GL_VERSION_OVERRIDE=3.3
export QT_QPA_PLATFORM=xcb
export DISPLAY=:0
```

**轨迹清理**:
```bash
# 清理RViz历史轨迹
python3 src/dolphin_slam/scripts/force_clear_trajectory.py
```

**Gazebo模型加载失败**:
```bash
# 重新生成模型缓存
gazebo --verbose worlds/underwater_world_enhanced.world
```

## 📈 性能评估

### 1. 评估指标

- **定位精度**: 与GPS ground truth的对比
- **地图一致性**: 闭环检测成功率
- **计算效率**: CPU使用率和内存占用
- **鲁棒性**: 不同环境条件下的表现

### 2. 测试场景

- **简单环境**: 开阔水域，少量障碍物
- **复杂环境**: 密集障碍物，复杂地形
- **动态环境**: 移动障碍物，水流影响
- **传感器故障**: 部分传感器失效情况

## 🔮 改进计划

### 1. 短期改进 (1-3个月)

#### 1.1 声纳图像集成
- **目标**: 集成侧扫声纳图像处理
- **实现**: 
  ```python
  # 声纳图像处理节点
  class SonarProcessingNode(Node):
      def __init__(self):
          super().__init__('sonar_processing_node')
          self.sonar_sub = self.create_subscription(
              Image, '/sonar/image_raw', self.sonar_callback, 10)
  ```
- **预期效果**: 提高水下环境感知能力

#### 1.2 GPS Ground Truth对比
- **目标**: 实现与GPS轨迹的实时对比
- **实现**:
  ```python
  # 轨迹评估器增强
  def compare_with_gps(self, slam_trajectory, gps_trajectory):
      # 计算定位误差
      position_error = np.linalg.norm(slam_trajectory - gps_trajectory)
      return position_error
  ```
- **预期效果**: 量化SLAM系统性能

#### 1.3 多传感器融合优化
- **目标**: 改进传感器数据融合算法
- **实现**: 卡尔曼滤波 + 粒子滤波混合方法
- **预期效果**: 提高定位精度和鲁棒性

### 2. 中期改进 (3-6个月)

#### 2.1 深度学习集成
- **目标**: 集成深度学习特征提取
- **实现**: 
  - 使用预训练的CNN提取图像特征
  - 端到端的视觉里程计
  - 基于深度学习的场景识别
- **预期效果**: 提高特征提取质量和场景识别准确性

#### 2.2 实时性能优化
- **目标**: 优化计算效率，支持实时运行
- **实现**:
  - GPU加速计算
  - 并行处理架构
  - 内存管理优化
- **预期效果**: 降低CPU使用率，提高处理速度

#### 2.3 自适应参数调整
- **目标**: 根据环境自动调整算法参数
- **实现**: 基于环境复杂度的参数自适应
- **预期效果**: 提高系统在不同环境下的适应性

### 3. 长期改进 (6-12个月)

#### 3.1 多机器人协作
- **目标**: 支持多AUV协同SLAM
- **实现**:
  - 分布式地图构建
  - 机器人间通信协议
  - 协同定位算法
- **预期效果**: 提高大范围环境探索效率

#### 3.2 语义SLAM
- **目标**: 集成语义信息到SLAM系统
- **实现**:
  - 物体识别和分类
  - 语义地图构建
  - 基于语义的路径规划
- **预期效果**: 提高导航的智能性和实用性

#### 3.3 长期自主导航
- **目标**: 实现完全自主的长期导航
- **实现**:
  - 能源管理
  - 任务规划
  - 故障恢复
- **预期效果**: 支持长时间自主运行

## 📊 项目结构

```
dolphin_slam_ws/
├── src/dolphin_slam/
│   ├── dolphin_slam/           # 核心算法实现
│   │   ├── place_cell_network.py      # 位置细胞网络
│   │   ├── local_view_cells.py        # 局部视觉细胞网络
│   │   ├── experience_map.py          # 经验地图
│   │   ├── image_processing.py        # 图像处理
│   │   └── utils.py                   # 工具函数
│   ├── nodes/                  # ROS2节点
│   │   ├── place_cell_node.py         # 位置细胞节点
│   │   ├── local_view_node.py         # 局部视觉节点
│   │   ├── experience_map_node.py     # 经验地图节点
│   │   └── ...
│   ├── config/                 # 配置文件
│   │   ├── dolphin_slam_params.yaml   # 算法参数
│   │   └── camera_calibration.yaml    # 相机标定
│   ├── launch/                 # 启动文件
│   │   ├── dolphin_slam_sim_launch.py # 仿真启动
│   │   └── dolphin_slam_launch.py     # 真实环境启动
│   ├── urdf/                   # 机器人模型
│   │   └── auv_robot.xacro            # AUV模型定义
│   ├── worlds/                 # 仿真环境
│   │   └── underwater_world_enhanced.world
│   ├── rviz/                   # 可视化配置
│   │   ├── dolphin_slam_simple.rviz   # 分屏布局配置
│   │   ├── dolphin_slam_optimized.rviz # 优化配置
│   │   └── dolphin_slam_sim.rviz      # 仿真配置
│   ├── scripts/                # 工具脚本
│   │   ├── force_clear_trajectory.py  # 强制轨迹清理
│   │   └── clear_trajectory.py        # 轨迹清理
│   └── test/                   # 测试文件
├── start_dolphin_slam_ultimate.sh     # 终极启动脚本
└── README.md                   # 项目文档
```

## 🤝 贡献指南

### 1. 开发环境设置

```bash
# 安装开发依赖
pip install -r requirements.txt
pip install pytest pytest-cov black flake8

# 代码格式化
black src/dolphin_slam/
flake8 src/dolphin_slam/
```

### 2. 测试

```bash
# 运行单元测试
pytest src/dolphin_slam/test/

# 运行集成测试
colcon test --packages-select dolphin_slam
```

### 3. 提交规范

- 使用清晰的提交信息
- 包含测试用例
- 更新相关文档

## 📄 许可证

本项目采用 Apache 2.0 许可证 - 详见 [LICENSE](LICENSE) 文件。

## 📞 联系方式

- **项目维护者**: Shiyu Sun
- **邮箱**: sunshiyu123@gmail.com
- **项目地址**: https://github.com/sunshiyu1989/dolphin_slam_gazebo

## 🙏 致谢

感谢以下开源项目的支持：
- [ROS2](https://docs.ros.org/) - 机器人操作系统
- [Gazebo](http://gazebosim.org/) - 物理仿真引擎
- [OpenCV](https://opencv.org/) - 计算机视觉库
- [NumPy](https://numpy.org/) - 科学计算库

---

**注意**: 本项目仍在积极开发中，欢迎提交Issue和Pull Request！ 