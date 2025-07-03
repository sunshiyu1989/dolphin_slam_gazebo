#!/bin/bash

# ====================================================================
# Dolphin SLAM 系统检查和启动脚本
# 专为水下无人机SLAM系统设计
# 支持相机、声呐图像同步显示和实时地图可视化
# ====================================================================

set -e  # 遇到错误即退出

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# 日志函数
log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

log_header() {
    echo -e "${CYAN}$1${NC}"
}

# 显示帮助信息
show_help() {
    cat << EOF
Dolphin SLAM 系统检查和启动脚本

用法: $0 [选项] [数据集路径]

选项:
    -h, --help              显示此帮助信息
    -c, --check             仅运行系统检查
    -d, --dataset PATH      指定数据集路径
    -r, --rviz              启动RViz可视化 (默认)
    --no-rviz              不启动RViz
    --speed RATE           播放速度倍数 (默认: 1.0)
    --record               录制bag文件
    --performance          启用性能监控
    --compressed           使用压缩图像传输
    --debug                启用调试模式

示例:
    # 基础启动（路径无空格）
    $0 -d /path/to/dataset
    
    # 路径包含空格时的正确用法：
    $0 -d "/media/psf/Samsung T7/SLAM Data/dataset"
    $0 "/media/psf/Samsung T7/SLAM Data/dataset"
    
    # 其他选项
    $0 -d "/path/to/dataset" --record     # 启动并录制
    $0 -c                                 # 仅系统检查
    $0 --help                             # 显示帮助

注意：如果数据集路径包含空格，请使用引号将整个路径包围起来。

EOF
}

# 检查ROS2环境
check_ros2_environment() {
    log_header "=== 检查ROS2环境 ==="
    
    if [ -z "$ROS_DISTRO" ]; then
        log_error "ROS2环境未设置！请先source ROS2环境"
        log_info "运行: source /opt/ros/humble/setup.bash  # 或其他ROS2发行版"
        exit 1
    fi
    
    log_success "ROS2发行版: $ROS_DISTRO"
    
    # 检查ROS2命令
    if ! command -v ros2 &> /dev/null; then
        log_error "ros2命令未找到！"
        exit 1
    fi
    
    log_success "ROS2命令可用"
    
    # 检查colcon
    if ! command -v colcon &> /dev/null; then
        log_warning "colcon未安装，可能影响包构建"
    else
        log_success "colcon可用"
    fi
}

# 检查Python依赖
check_python_dependencies() {
    log_header "=== 检查Python依赖 ==="
    
    local required_packages=(
        "numpy"
        "opencv-python"
        "opencv-contrib-python"
        "scipy"
        "scikit-learn"
        "pandas"
        "matplotlib"
        "tqdm"
        "pyyaml"
    )
    
    local missing_packages=()
    
    for package in "${required_packages[@]}"; do
        if python3 -c "import ${package//-/_}" 2>/dev/null; then
            log_success "$package 已安装"
        else
            log_error "$package 未安装"
            missing_packages+=("$package")
        fi
    done
    
    if [ ${#missing_packages[@]} -gt 0 ]; then
        log_error "缺少Python依赖包："
        printf '%s\n' "${missing_packages[@]}" | sed 's/^/  - /'
        log_info "运行以下命令安装："
        echo "pip install ${missing_packages[*]}"
        exit 1
    fi
    
    log_success "所有Python依赖已满足"
}

# 检查ROS2包依赖
check_ros2_packages() {
    log_header "=== 检查ROS2包依赖 ==="
    
    local required_packages=(
        "sensor_msgs"
        "geometry_msgs"
        "nav_msgs"
        "tf2_ros"
        "tf2_geometry_msgs"
        "image_transport"
        "cv_bridge"
        "message_filters"
        "rviz2"
    )
    
    for package in "${required_packages[@]}"; do
        if ros2 pkg list | grep -q "^$package$"; then
            log_success "$package 可用"
        else
            log_error "$package 未找到"
            log_info "安装命令: sudo apt install ros-$ROS_DISTRO-${package//_/-}"
        fi
    done
}

# 检查硬件配置
check_hardware() {
    log_header "=== 检查硬件配置 ==="
    
    # 检查CPU
    local cpu_cores=$(nproc)
    log_info "CPU核心数: $cpu_cores"
    
    if [ "$cpu_cores" -lt 4 ]; then
        log_warning "CPU核心数较少，建议降低处理参数"
    fi
    
    # 检查内存
    local total_mem=$(free -m | awk 'NR==2{printf "%.1f", $2/1024}')
    log_info "总内存: ${total_mem}GB"
    
    if (( $(echo "$total_mem < 8.0" | bc -l) )); then
        log_warning "内存较少，建议启用内存优化选项"
    fi
    
    # 检查GPU（如果有）
    if command -v nvidia-smi &> /dev/null; then
        local gpu_info=$(nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null | head -1)
        if [ -n "$gpu_info" ]; then
            log_success "检测到GPU: $gpu_info"
        fi
    fi
    
    # 检查MacOS M芯片
    if [[ "$OSTYPE" == "darwin"* ]]; then
        local chip_info=$(system_profiler SPHardwareDataType | grep "Chip:" | awk '{print $2, $3}')
        if [[ "$chip_info" == *"Apple"* ]]; then
            log_success "检测到Apple芯片: $chip_info"
            log_info "将使用ARM64优化配置"
        fi
    fi
}

# 检查数据集
check_dataset() {
    local dataset_path="$1"
    
    if [ -z "$dataset_path" ]; then
        log_warning "未指定数据集路径"
        return 1
    fi
    
    log_header "=== 检查数据集: $dataset_path ==="
    
    if [ ! -d "$dataset_path" ]; then
        log_error "数据集目录不存在: $dataset_path"
        return 1
    fi
    
    # 检查必需文件
    local required_items=(
        "navigation.csv"
        "camera"
        "sonar"
    )
    
    for item in "${required_items[@]}"; do
        if [ -e "$dataset_path/$item" ]; then
            log_success "找到: $item"
            
            # 详细检查
            case "$item" in
                "camera"|"sonar")
                    local count=$(find "$dataset_path/$item" -name "*.png" -o -name "*.jpg" -o -name "*.jpeg" | wc -l)
                    log_info "  图像数量: $count"
                    if [ "$count" -eq 0 ]; then
                        log_warning "  没有找到图像文件"
                    fi
                    ;;
                "navigation.csv")
                    local lines=$(wc -l < "$dataset_path/$item")
                    log_info "  记录数量: $lines"
                    ;;
            esac
        else
            log_error "缺少: $item"
            return 1
        fi
    done
    
    # 运行数据集分析
    if [ -f "$(find . -name 'analyze_dataset.py' 2>/dev/null | head -1)" ]; then
        log_info "运行数据集分析..."
        python3 scripts/analyze_dataset.py "$dataset_path" --quick
    fi
    
    log_success "数据集检查通过"
    return 0
}

# 检查工作空间
check_workspace() {
    log_header "=== 检查工作空间 ==="
    
    if [ ! -f "src/dolphin_slam/package.xml" ]; then
        log_error "不在正确的工作空间目录！"
        log_info "请切换到dolphin_slam_ws目录"
        exit 1
    fi
    
    log_success "在正确的工作空间目录"
    
    # 检查是否已构建
    if [ ! -d "install" ]; then
        log_warning "工作空间未构建"
        log_info "开始构建工作空间..."
        colcon build --symlink-install
        if [ $? -eq 0 ]; then
            log_success "工作空间构建完成"
        else
            log_error "工作空间构建失败"
            exit 1
        fi
    else
        log_success "工作空间已构建"
    fi
    
    # Source工作空间
    if [ -f "install/setup.bash" ]; then
        source install/setup.bash
        log_success "工作空间环境已加载"
    fi
}

# 启动系统
launch_system() {
    local dataset_path="$1"
    local enable_rviz="$2"
    local playback_speed="$3"
    local enable_recording="$4"
    local enable_performance="$5"
    local use_compressed="$6"
    
    log_header "=== 启动Dolphin SLAM系统 ==="
    
    # 构建启动参数
    local launch_args="dataset_path:=$dataset_path"
    launch_args="$launch_args enable_rviz:=$enable_rviz"
    launch_args="$launch_args playback_speed:=$playback_speed"
    launch_args="$launch_args enable_recording:=$enable_recording"
    launch_args="$launch_args enable_performance_monitoring:=$enable_performance"
    launch_args="$launch_args use_compressed:=$use_compressed"
    
    log_info "启动参数: $launch_args"
    
    # 创建日志目录
    local log_dir="logs/$(date +%Y%m%d_%H%M%S)"
    mkdir -p "$log_dir"
    
    log_info "日志目录: $log_dir"
    log_info "正在启动系统..."
    
    # 启动系统
    ros2 launch dolphin_slam dolphin_slam_enhanced_launch.py $launch_args 2>&1 | tee "$log_dir/system.log"
}

# 显示系统状态
show_system_status() {
    log_header "=== 系统状态总结 ==="
    
    echo "✓ ROS2环境: $ROS_DISTRO"
    echo "✓ Python依赖: 已满足"
    echo "✓ 硬件配置: 已检查"
    echo "✓ 工作空间: 已构建"
    
    if [ -n "$DATASET_PATH" ] && [ -d "$DATASET_PATH" ]; then
        echo "✓ 数据集: $DATASET_PATH"
    else
        echo "⚠ 数据集: 未指定或无效"
    fi
    
    echo ""
    log_success "系统检查完成，可以启动SLAM系统"
}

# 主函数
main() {
    # 默认参数
    local check_only=false
    local dataset_path=""
    local enable_rviz="true"
    local playback_speed="1.0"
    local enable_recording="false"
    local enable_performance="false"
    local use_compressed="false"
    local debug_mode=false
    
    # 解析命令行参数
    while [[ $# -gt 0 ]]; do
        case $1 in
            -h|--help)
                show_help
                exit 0
                ;;
            -c|--check)
                check_only=true
                shift
                ;;
            -d|--dataset)
                dataset_path="$2"
                shift 2
                ;;
            -r|--rviz)
                enable_rviz="true"
                shift
                ;;
            --no-rviz)
                enable_rviz="false"
                shift
                ;;
            --speed)
                playback_speed="$2"
                shift 2
                ;;
            --record)
                enable_recording="true"
                shift
                ;;
            --performance)
                enable_performance="true"
                shift
                ;;
            --compressed)
                use_compressed="true"
                shift
                ;;
            --debug)
                debug_mode=true
                shift
                ;;
            -*)
                log_error "未知选项: $1"
                show_help
                exit 1
                ;;
            *)
                # 如果没有通过 -d 指定数据集路径，将剩余所有参数组合为路径
                if [ -z "$dataset_path" ]; then
                    # 重新组合所有剩余参数为一个路径
                    dataset_path="$*"
                    break
                else
                    log_error "意外的参数: $1"
                    log_info "数据集路径已设置为: $dataset_path"
                    show_help
                    exit 1
                fi
                ;;
        esac
    done
    
    # 调试模式
    if [ "$debug_mode" = true ]; then
        set -x
    fi
    
    # 显示启动信息
    log_header "======================================================================"
    log_header "               Dolphin SLAM 水下无人机SLAM系统"
    log_header "======================================================================"
    
    # 执行检查
    check_ros2_environment
    check_python_dependencies
    check_ros2_packages
    check_hardware
    check_workspace
    
    if [ -n "$dataset_path" ]; then
        check_dataset "$dataset_path"
        export DATASET_PATH="$dataset_path"
    fi
    
    show_system_status
    
    # 如果只是检查模式，则退出
    if [ "$check_only" = true ]; then
        log_info "系统检查完成"
        exit 0
    fi
    
    # 如果没有指定数据集，请求用户输入
    if [ -z "$dataset_path" ]; then
        echo ""
        read -p "请输入数据集路径: " dataset_path
        if [ ! -d "$dataset_path" ]; then
            log_error "无效的数据集路径"
            exit 1
        fi
        check_dataset "$dataset_path"
    fi
    
    # 启动确认
    echo ""
    log_info "准备启动系统，配置如下："
    echo "  数据集: $dataset_path"
    echo "  RViz可视化: $enable_rviz"
    echo "  播放速度: ${playback_speed}x"
    echo "  录制bag: $enable_recording"
    echo "  性能监控: $enable_performance"
    echo ""
    
    read -p "确认启动? (y/N) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        launch_system "$dataset_path" "$enable_rviz" "$playback_speed" "$enable_recording" "$enable_performance" "$use_compressed"
    else
        log_info "启动已取消"
    fi
}

# 错误处理
trap 'log_error "脚本执行出错，退出码: $?"' ERR

# 运行主函数
main "$@"
