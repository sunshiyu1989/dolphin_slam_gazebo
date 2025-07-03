#!/bin/bash
# Dolphin SLAM 增强启动脚本
# 支持命令行参数控制

# 颜色定义
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

# 默认配置
WORKSPACE_DIR="/home/parallels/dolphin_slam_ws"
DATASET_PATH="/media/psf/Samsung T7/SLAM Data/Sunboat_03-09-2023/2023-09-03-07-58-37"
ROS_DOMAIN_ID=42
ENABLE_RVIZ=true  # 默认启用 RViz
SHOW_HELP=false

# 函数：显示帮助信息
show_help() {
    echo -e "${BLUE}🌊 Dolphin SLAM 启动脚本${NC}"
    echo ""
    echo "用法: $0 [选项]"
    echo ""
    echo "选项:"
    echo "  --rviz              启用 RViz 可视化 (默认)"
    echo "  --no-rviz           禁用 RViz 可视化"
    echo "  --dataset PATH      指定数据集路径"
    echo "  --domain ID         设置 ROS_DOMAIN_ID (默认: 42)"
    echo "  -h, --help          显示此帮助信息"
    echo ""
    echo "示例:"
    echo "  $0                  # 默认启动（带 RViz）"
    echo "  $0 --no-rviz        # 启动但不显示 RViz"
    echo "  $0 --rviz           # 明确启动 RViz"
    echo "  $0 --dataset /path/to/data --no-rviz"
    echo ""
}

# 解析命令行参数
while [[ $# -gt 0 ]]; do
    case $1 in
        --rviz)
            ENABLE_RVIZ=true
            shift
            ;;
        --no-rviz)
            ENABLE_RVIZ=false
            shift
            ;;
        --dataset)
            DATASET_PATH="$2"
            shift 2
            ;;
        --domain)
            ROS_DOMAIN_ID="$2"
            shift 2
            ;;
        -h|--help)
            show_help
            exit 0
            ;;
        *)
            echo -e "${RED}❌ 未知选项: $1${NC}"
            echo "使用 --help 查看帮助信息"
            exit 1
            ;;
    esac
done

# 显示启动配置
echo -e "${BLUE}🚀 启动 Dolphin SLAM${NC}"
echo -e "${YELLOW}配置信息:${NC}"
echo "  数据集路径: $DATASET_PATH"
echo "  RViz 可视化: $([ "$ENABLE_RVIZ" = true ] && echo '✅ 启用' || echo '❌ 禁用')"
echo "  ROS Domain ID: $ROS_DOMAIN_ID"
echo ""

# 检查数据集路径是否存在
if [ ! -d "$DATASET_PATH" ]; then
    echo -e "${RED}❌ 数据集路径不存在: $DATASET_PATH${NC}"
    echo "请使用 --dataset 指定正确的数据集路径"
    exit 1
fi

# 停止现有进程
echo -e "${BLUE}🧹 清理现有进程...${NC}"
pkill -f dolphin_slam &>/dev/null
sleep 2

# 设置环境
echo -e "${BLUE}⚙️  设置环境...${NC}"
cd "$WORKSPACE_DIR"
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=$ROS_DOMAIN_ID

# 重启守护进程（如果需要）
if ! ros2 node list &>/dev/null; then
    echo -e "${BLUE}🔄 重启ROS2守护进程...${NC}"
    ros2 daemon stop &>/dev/null || true
    sleep 2
    ros2 daemon start
    sleep 2
fi

# 启动确认
echo -e "${GREEN}✅ 准备启动 Dolphin SLAM${NC}"
if [ "$ENABLE_RVIZ" = true ]; then
    echo -e "${GREEN}📊 RViz 可视化窗口将会打开${NC}"
fi

# 启动系统
echo -e "${BLUE}🚀 启动中...${NC}"
ros2 launch dolphin_slam dolphin_slam_launch.py \
    dataset_path:="$DATASET_PATH" \
    enable_rviz:=$ENABLE_RVIZ

echo -e "${GREEN}🏁 Dolphin SLAM 已退出${NC}"
