#!/bin/bash

# RViz OpenGL 修复脚本 - 针对Parallels Desktop虚拟机

echo "🔧 配置RViz在Parallels Desktop中正常运行..."

# 方法1: 使用软件渲染
echo "方法1: 启用软件渲染RViz"
echo "export LIBGL_ALWAYS_SOFTWARE=1" >> ~/.bashrc
echo "export QT_QPA_PLATFORM=xcb" >> ~/.bashrc

# 方法2: 配置Mesa 3D软件渲染
echo "方法2: 安装Mesa软件渲染支持"
sudo apt update
sudo apt install -y mesa-utils mesa-utils-extra libosmesa6-dev

# 方法3: 降级OpenGL版本
echo "方法3: 强制使用OpenGL 2.1"
echo "export MESA_GL_VERSION_OVERRIDE=2.1" >> ~/.bashrc
echo "export MESA_GLSL_VERSION_OVERRIDE=120" >> ~/.bashrc

# 方法4: 禁用一些RViz功能
echo "方法4: 禁用GPU加速功能"
echo "export QT_OPENGL=software" >> ~/.bashrc
echo "export QT_QUICK_BACKEND=software" >> ~/.bashrc

# 创建RViz启动脚本
cat > ~/run_rviz_safe.sh << 'EOF'
#!/bin/bash
# 安全的RViz启动脚本

echo "🚀 启动安全模式RViz..."

# 设置环境变量
export LIBGL_ALWAYS_SOFTWARE=1
export QT_QPA_PLATFORM=xcb
export MESA_GL_VERSION_OVERRIDE=2.1
export MESA_GLSL_VERSION_OVERRIDE=120
export QT_OPENGL=software
export QT_QUICK_BACKEND=software

# 启动RViz
if [ $# -eq 0 ]; then
    echo "用法: $0 [RViz配置文件路径]"
    echo "启动默认RViz..."
    rviz2
else
    echo "启动RViz配置: $1"
    rviz2 -d "$1"
fi
EOF

chmod +x ~/run_rviz_safe.sh

echo "✅ RViz修复配置完成！"
echo ""
echo "使用方法："
echo "1. 重新启动终端或运行: source ~/.bashrc"
echo "2. 使用安全脚本启动RViz: ~/run_rviz_safe.sh /path/to/config.rviz"
echo "3. 或者重新启动系统应用所有配置"

# 验证OpenGL配置
echo ""
echo "🔍 验证OpenGL配置："
glxinfo | grep "OpenGL version" || echo "需要安装 mesa-utils: sudo apt install mesa-utils"
