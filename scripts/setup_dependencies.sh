#!/bin/bash

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

echo "================================================================"
echo "Scorpio 依赖自动安装脚本"
echo "================================================================"
echo "本脚本将自动安装:"
echo "  1. Intel RealSense SDK 2.0 + ROS2 Wrapper"
echo "  2. YDLidar SDK"
echo "  3. RGLGazeboPlugin (GPU Lidar 仿真插件)"
echo "  4. YDLidar udev 规则"
echo ""
echo "需要 sudo 权限,请准备输入密码。"
echo "================================================================"
echo ""

# ============================================================
# 1. Intel RealSense SDK 2.0
# ============================================================
echo "[1/4] 安装 Intel RealSense SDK 2.0..."

echo "  -> 注册 RealSense GPG 密钥..."
sudo mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null

echo "  -> 添加 RealSense 软件源..."
echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" | \
sudo tee /etc/apt/sources.list.d/librealsense.list
sudo apt-get update

echo "  -> 安装 librealsense2..."
sudo apt-get install -y librealsense2-dkms librealsense2-utils

echo "  -> 安装 ROS2 RealSense Wrapper..."
sudo apt-get install -y ros-$ROS_DISTRO-realsense2-*

echo -e "\e[32m✓ RealSense SDK 安装完成\e[0m"
echo ""

# ============================================================
# 2. YDLidar SDK
# ============================================================
echo "[2/4] 安装 YDLidar SDK..."

YDLIDAR_DIR="$HOME/Programs/YDLidar-SDK"

if [ -d "$YDLIDAR_DIR" ]; then
    echo "  -> YDLidar-SDK 目录已存在,跳过 clone"
else
    echo "  -> Clone YDLidar-SDK..."
    mkdir -p ~/Programs
    git clone https://github.com/YDLIDAR/YDLidar-SDK.git "$YDLIDAR_DIR"
fi

echo "  -> 编译并安装 YDLidar-SDK..."
cd "$YDLIDAR_DIR"
mkdir -p build
cd build
cmake ..
make
sudo make install

echo -e "\e[32m✓ YDLidar SDK 安装完成\e[0m"
echo ""

# ============================================================
# 3. YDLidar udev 规则
# ============================================================
echo "[3/4] 配置 YDLidar udev 规则..."

UDEV_RULES=(
    "/etc/udev/rules.d/99-ydlidar.rules:KERNEL==\"ttyUSB*\", ATTRS{idVendor}==\"10c4\", ATTRS{idProduct}==\"ea60\", MODE:=\"0666\", GROUP:=\"dialout\", SYMLINK+=\"ydlidar\""
    "/etc/udev/rules.d/99-ydlidar-V2.rules:KERNEL==\"ttyACM*\", ATTRS{idVendor}==\"0483\", ATTRS{idProduct}==\"5740\", MODE:=\"0666\", GROUP:=\"dialout\", SYMLINK+=\"ydlidar\""
    "/etc/udev/rules.d/99-ydlidar-2303.rules:KERNEL==\"ttyUSB*\", ATTRS{idVendor}==\"067b\", ATTRS{idProduct}==\"2303\", MODE:=\"0666\", GROUP:=\"dialout\", SYMLINK+=\"ydlidar\""
)

echo "  -> 写入 udev 规则..."
for rule in "${UDEV_RULES[@]}"; do
    FILE="${rule%%:*}"
    CONTENT="${rule#*:}"
    echo "$CONTENT" | sudo tee "$FILE" > /dev/null
done

echo "  -> 重载并重启 udev 服务..."
sudo service udev reload
sleep 2
sudo service udev restart

CURRENT_USER=$(whoami)
echo "  -> 将用户 $CURRENT_USER 添加到 dialout 组..."
sudo usermod -aG dialout "$CURRENT_USER"

echo -e "\e[32m✓ YDLidar udev 规则已设置\e[0m"
echo ""

# ============================================================
# 4. RGLGazeboPlugin (GPU Lidar 仿真插件)
# ============================================================
echo "[4/4] 安装 RGLGazeboPlugin..."

TEMP_DIR="/tmp/rgl_plugin_install"
mkdir -p "$TEMP_DIR"
cd "$TEMP_DIR"

echo "  -> 下载 RGLGazeboPlugin..."
wget https://github.com/RobotecAI/RGLGazeboPlugin/releases/download/v0.2.0-fortress/RGLGazeboPlugin_ubuntu22.zip

echo "  -> 解压插件..."
unzip -q RGLGazeboPlugin_ubuntu22.zip -d RGLGazeboPlugin_ubuntu22

echo "  -> 复制插件到系统目录..."
cd RGLGazeboPlugin_ubuntu22
sudo cp RGLServerPlugin/libRobotecGPULidar.so /usr/lib/x86_64-linux-gnu/ign-gazebo-6/plugins/
sudo cp RGLServerPlugin/libRGLServerPluginInstance.so /usr/lib/x86_64-linux-gnu/ign-gazebo-6/plugins/
sudo cp RGLServerPlugin/libRGLServerPluginManager.so /usr/lib/x86_64-linux-gnu/ign-gazebo-6/plugins/
sudo cp RGLVisualize/libRGLVisualize.so /usr/lib/x86_64-linux-gnu/ign-gazebo-6/plugins/gui/

echo "  -> 清理临时文件..."
cd /tmp
sudo rm -rf "$TEMP_DIR"

echo "  -> 下载 Livox Mid360 激光雷达模式文件..."
LIDAR_PATTERN_DIR="$REPO_ROOT/scorpio_description/resource/models/mid360/lidar_patterns"
mkdir -p "$LIDAR_PATTERN_DIR"
wget https://raw.githubusercontent.com/RobotecAI/RGLGazeboPlugin/fortress/lidar_patterns/LivoxMid360.mat3x4f \
     -O "$LIDAR_PATTERN_DIR/LivoxMid360.mat3x4f"

echo -e "\e[32m✓ RGLGazeboPlugin 安装完成\e[0m"
echo ""

# ============================================================
# 完成
# ============================================================
echo "================================================================"
echo -e "\e[32m✓ 所有依赖安装完成!\e[0m"
echo "================================================================"
echo ""
echo "Tips:"
echo "  1. 请注销并重新登录以使 dialout 组权限生效"
echo "  2. 然后继续执行 README 的 2.3 Build 步骤"
echo ""
