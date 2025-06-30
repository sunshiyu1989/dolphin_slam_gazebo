#!/bin/bash
# Dolphin SLAM 运行脚本
# 用于在 Linux 虚拟机上运行 ROS2 仿真

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 打印带颜色的信息
print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# 检查 ROS2 环境
check_ros2() {
    if [ -z "$ROS_DISTRO" ]; then
        print_error "ROS2 环境未设置！"
        print_info "请运行: source /opt/ros/humble/setup.bash"
        exit 1
    fi
    print_success "ROS2 $ROS_DISTRO 环境已加载"
}

# 项目路径
WORKSPACE_DIR="$HOME/dolphin_slam_ws"
DATASET_PATH=""
USE_RVIZ=true
PLAYBACK_SPEED=1.0

# 解析命令行参数
while [[ $# -gt 0 ]]; do
    case $1 in
        -d|--dataset)
            DATASET_PATH="$2"
            shift 2
            ;;
        -w|--workspace)
            WORKSPACE_DIR="$2"
            shift 2
            ;;
        --no-rviz)
            USE_RVIZ=false
            shift
            ;;
        -s|--speed)
            PLAYBACK_SPEED="$2"
            shift 2
            ;;
        -h|--help)
            echo "用法: $0 [选项]"
            echo "选项:"
            echo "  -d, --dataset PATH     数据集路径 (必需)"
            echo "  -w, --workspace PATH   工作空间路径 (默认: ~/dolphin_slam_ws)"
            echo "  --no-rviz             不启动 RViz"
            echo "  -s, --speed SPEED     播放速度 (默认: 1.0)"
            echo "  -h, --help            显示帮助信息"
            exit 0
            ;;
        *)
            print_error "未知选项: $1"
            exit 1
            ;;
    esac
done

# 检查数据集路径
if [ -z "$DATASET_PATH" ]; then
    print_error "请指定数据集路径！"
    echo "用法: $0 -d /path/to/AUV-Dataset"
    exit 1
fi

if [ ! -d "$DATASET_PATH" ]; then
    print_error "数据集路径不存在: $DATASET_PATH"
    exit 1
fi

# 检查必要文件
print_info "检查数据集文件..."
required_files=("navigation.csv" "camera.csv" "sonar.csv")
for file in "${required_files[@]}"; do
    if [ ! -f "$DATASET_PATH/$file" ]; then
        print_error "缺少必要文件: $DATASET_PATH/$file"
        exit 1
    fi
done
print_success "数据集文件检查完成"

# 检查 ROS2
check_ros2

# 进入工作空间
cd "$WORKSPACE_DIR"

# 检查是否需要构建
if [ ! -d "install" ]; then
    print_warning "工作空间未构建，开始构建..."
    colcon build --symlink-install
    if [ $? -ne 0 ]; then
        print_error "构建失败！"
        exit 1
    fi
fi

# Source 工作空间
print_info "加载工作空间..."
source install/setup.bash

# 创建日志目录
LOG_DIR="$WORKSPACE_DIR/logs/$(date +%Y%m%d_%H%M%S)"
mkdir -p "$LOG_DIR"
print_info "日志将保存到: $LOG_DIR"

# 启动参数
LAUNCH_ARGS=""
LAUNCH_ARGS="$LAUNCH_ARGS dataset_path:=$DATASET_PATH"
LAUNCH_ARGS="$LAUNCH_ARGS enable_rviz:=$USE_RVIZ"
LAUNCH_ARGS="$LAUNCH_ARGS use_sim_time:=true"

# 检查是否需要录制 bag
read -p "是否录制 ROS bag 文件? (y/n) " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    BAG_DIR="$WORKSPACE_DIR/bags"
    mkdir -p "$BAG_DIR"
    BAG_FILE="$BAG_DIR/dolphin_slam_$(date +%Y%m%d_%H%M%S).bag"
    
    print_info "开始录制 bag 文件: $BAG_FILE"
    ros2 bag record -a -o "$BAG_FILE" &
    BAG_PID=$!
fi

# 显示系统信息
print_info "========== 系统信息 =========="
echo "工作空间: $WORKSPACE_DIR"
echo "数据集: $DATASET_PATH"
echo "RViz: $USE_RVIZ"
echo "播放速度: ${PLAYBACK_SPEED}x"
echo "日志目录: $LOG_DIR"
print_info "============================="

# 启动 Dolphin SLAM
print_info "启动 Dolphin SLAM..."
ros2 launch dolphin_slam dolphin_slam_launch.py $LAUNCH_ARGS 2>&1 | tee "$LOG_DIR/dolphin_slam.log"

# 停止 bag 录制
if [ ! -z "$BAG_PID" ]; then
    print_info "停止 bag 录制..."
    kill $BAG_PID 2>/dev/null || true
fi

print_success "Dolphin SLAM 运行完成！"

# 询问是否保存地图
read -p "是否保存地图? (y/n) " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    MAP_FILE="$WORKSPACE_DIR/maps/map_$(date +%Y%m%d_%H%M%S).pkl"
    mkdir -p "$(dirname $MAP_FILE)"
    
    # 调用保存地图服务（需要实现相应的服务）
    ros2 service call /dolphin_slam/save_map std_srvs/srv/Trigger
    
    print_success "地图已保存到: $MAP_FILE"
fi

# 生成运行报告
print_info "生成运行报告..."
cat > "$LOG_DIR/run_report.txt" << EOF
Dolphin SLAM 运行报告
=====================
运行时间: $(date)
工作空间: $WORKSPACE_DIR
数据集: $DATASET_PATH
ROS 版本: $ROS_DISTRO

性能统计:
- 处理帧数: $(grep -c "检测到.*特征点" "$LOG_DIR/dolphin_slam.log" || echo "N/A")
- 创建经验数: $(grep -c "创建新经验" "$LOG_DIR/dolphin_slam.log" || echo "N/A")
- 检测闭环数: $(grep -c "检测到闭环" "$LOG_DIR/dolphin_slam.log" || echo "N/A")

日志文件:
- 主日志: $LOG_DIR/dolphin_slam.log
EOF

if [ ! -z "$BAG_FILE" ]; then
    echo "- Bag 文件: $BAG_FILE" >> "$LOG_DIR/run_report.txt"
fi

print_success "运行报告已保存到: $LOG_DIR/run_report.txt"

# 显示简要统计
echo
print_info "========== 运行统计 =========="
grep -E "(创建新经验|检测到闭环|特征点)" "$LOG_DIR/dolphin_slam.log" | tail -10
print_info "============================="
