#!/bin/bash

echo "🐬 Dolphin SLAM 终极启动脚本"
echo "🎯 水下机器人SLAM系统 - 优化RViz版"
echo ""

# 清理之前的进程
echo "🧹 清理之前的进程..."
pkill -f gazebo
pkill -f ros2
pkill -f rviz2
sleep 2

# 设置环境变量
echo "🔧 设置环境变量..."
export GAZEBO_RESOURCE_PATH=/usr/share/gazebo-11:$GAZEBO_RESOURCE_PATH
export QT_QPA_PLATFORM=xcb
export DISPLAY=:0
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_GL_VERSION_OVERRIDE=3.3

# 设置ROS2环境
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh
source install/setup.bash

echo "🌊 启动 Dolphin SLAM 水下仿真系统..."
echo "   - 日志级别: WARN (精简输出)"
echo "   - RViz配置: 分屏布局 (dolphin_slam_simple.rviz)"
echo "   - 显示内容: 左侧相机图像，右侧3D运动轨迹"
echo ""

# 启动完整的SLAM系统 - 使用环境变量设置日志级别
export RCUTILS_LOGGING_USE_STDOUT=1
export RCUTILS_LOGGING_BUFFERED_STREAM=1
export RCUTILS_LOGGING_LEVEL=WARN

ros2 launch dolphin_slam dolphin_slam_sim_launch.py gui:=true enable_rviz:=true &
LAUNCH_PID=$!

echo "⏳ 等待系统启动..."
sleep 25

# 清理历史轨迹
echo "🧹 清理历史轨迹数据..."
python3 src/dolphin_slam/scripts/force_clear_trajectory.py
sleep 3

echo ""
echo "🎉 系统启动完成！"
echo ""
echo "📊 系统状态:"
echo "   🌊 Gazebo 仿真器: 运行中"
echo "   🤖 水下机器人: 已生成"
echo "   🧠 SLAM 节点: 运行中 (WARN级别)"
echo "   🎮 航点控制器: 扩展探索版"
echo "   🔧 力命令发布器: 运行中"
echo "   📍 RViz 可视化: 分屏布局"
echo ""
echo "🎯 RViz 分屏布局:"
echo "   📷 左侧: 前向相机实时图像"
echo "   🤖 右侧: 3D机器人模型和运动轨迹"
echo "   📍 轨迹显示: 绿色轨迹线 (1000点缓冲区)"
echo "   🧭 TF变换: 坐标系变换关系"
echo ""
echo "🔧 界面调整:"
echo "   🖱️  鼠标左键: 旋转视角"
echo "   🖱️  鼠标中键: 平移视角"
echo "   🖱️  鼠标滚轮: 缩放视角"
echo "   📋 左侧面板: 显示/隐藏各种元素"
echo "   ⚙️  右侧面板: 调整显示参数"
echo ""
echo "💡 使用提示:"
echo "   📷 左侧视图: 显示前向相机实时图像"
echo "   🤖 右侧视图: 3D机器人模型和运动轨迹"
echo "   📍 在Displays面板中可以启用/禁用各种显示"
echo "   🎨 可以调整轨迹颜色、线宽等参数"
echo "   📐 可以调整3D视角和距离"
echo "   📊 轨迹缓冲区: 1000点，显示完整路径"
echo ""
echo "🎯 3D探索路径:"
echo "   📍 航点1: (5,5,-6) → 航点2: (10,0,-8) → 航点3: (10,10,-4)"
echo "   📍 航点4: (0,10,-10) → 航点5: (-10,10,-6) → 航点6: (-10,0,-12)"
echo "   📍 航点7: (-10,-10,-8) → 航点8: (0,-10,-5) → 航点9: (10,-10,-10)"
echo "   📍 航点10: (0,0,-8) - 完成探索"
echo ""
echo "按 Ctrl+C 停止系统"

# 等待用户中断
wait 