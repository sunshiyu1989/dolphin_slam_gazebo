#!/bin/bash

echo "🚀 启动优化版 Dolphin SLAM..."

# 加载渲染优化环境
source ~/.gazebo_render_env

# 进入工作目录
cd ~/dolphin_slam_ws
source install/setup.bash

# 清理任何残留进程
echo "🧹 清理环境..."
sudo pkill -f gzserver 2>/dev/null
sudo pkill -f gzclient 2>/dev/null
sleep 3

# 显示系统状态
echo "📊 系统状态:"
if command -v nvidia-smi &> /dev/null; then
    echo "GPU: $(nvidia-smi --query-gpu=name,temperature.gpu,utilization.gpu --format=csv,noheader)"
fi

echo "环境: $XDG_SESSION_TYPE"
echo "Prime: $(prime-select query 2>/dev/null || echo 'Not available')"

# 启动仿真
echo "🌊 启动 Dolphin SLAM 水下仿真..."
ros2 launch dolphin_slam dolphin_slam_sim_launch.py
