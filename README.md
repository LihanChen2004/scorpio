# scorpio

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![Build and Test](https://github.com/LihanChen2004/scorpio/actions/workflows/build_and_test.yml/badge.svg)](https://github.com/LihanChen2004/scorpio/actions/workflows/build_and_test.yml)
[![pre-commit](https://img.shields.io/badge/pre--commit-enabled-brightgreen?logo=pre-commit)](https://github.com/pre-commit/pre-commit)

## 1. Overview

scorpio 是非官方创建的 [NXROBO-scorpio](https://github.com/NXROBO/scorpio.git) ROS2 版本

## 2. Quick Start

### 2.1 Setup Environment

#### 2.1.1 Install the ROS2 distribution

  Ubuntu 22.04: [ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

#### 2.1.2 Install drivers

- Install the latest Intel® RealSense™ SDK 2.0 and ROS Wrapper

  Register the server's public key:

  ```sh
  sudo mkdir -p /etc/apt/keyrings
  curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null
  ```

  Add the server to the list of repositories:

  ```sh
  echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \
  sudo tee /etc/apt/sources.list.d/librealsense.list
  sudo apt-get update
  ```

  Install the libraries:

  ```sh
  sudo apt-get install librealsense2-dkms librealsense2-utils
  ```

  Install ROS Wrapper for Intel® RealSense™ cameras

  ```sh
  sudo apt install ros-$ROS_DISTRO-realsense2-*
  ```

- Install YDLidar-SDK

  ```sh
  mkdir -p ~/Programs && cd ~/Programs
  git clone https://github.com/YDLIDAR/YDLidar-SDK.git
  cd YDLidar-SDK
  mkdir build
  cd build
  cmake ..
  make
  sudo make install
  ```

### 2.2 Create Workspace

### 2.2.1 Clone

```sh
pip3 install vcs2l
pip3 install xmacro
```

```sh
mkdir -p ~/scorpio_ws
cd ~/scorpio_ws
```

```sh
git clone https://github.com/LihanChen2004/scorpio.git src/scorpio
```

```sh
vcs import src/scorpio < src/scorpio/dependencies.repos
```

## 2.2.2 安装 RGLGazeboPlugin（GPU Lidar 仿真插件）

1. 下载插件压缩包：

    ```sh
    wget https://github.com/RobotecAI/RGLGazeboPlugin/releases/download/v0.2.0-fortress/RGLGazeboPlugin_ubuntu22.zip
    ```

2. 解压并复制插件文件：

    ```sh
    unzip RGLGazeboPlugin_ubuntu22.zip -d RGLGazeboPlugin_ubuntu22
    cd RGLGazeboPlugin_ubuntu22
    sudo cp RGLServerPlugin/libRobotecGPULidar.so /usr/lib/x86_64-linux-gnu/ign-gazebo-6/plugins/
    sudo cp RGLServerPlugin/libRGLServerPluginInstance.so /usr/lib/x86_64-linux-gnu/ign-gazebo-6/plugins/
    sudo cp RGLServerPlugin/libRGLServerPluginManager.so /usr/lib/x86_64-linux-gnu/ign-gazebo-6/plugins/
    sudo cp RGLVisualize/libRGLVisualize.so /usr/lib/x86_64-linux-gnu/ign-gazebo-6/plugins/gui/

    cd ..
    rm -rf RGLGazeboPlugin_ubuntu22.zip RGLGazeboPlugin_ubuntu22
    ```

3. 下载 LivoxMid360 激光雷达模式文件：

    ```sh
    wget https://raw.githubusercontent.com/RobotecAI/RGLGazeboPlugin/fortress/lidar_patterns/LivoxMid360.mat3x4f -O src/scorpio/scorpio_description/resource/models/mid360/lidar_patterns/LivoxMid360.mat3x4f
    ```

### 2.3 Build

```sh
rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```

```sh
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=release
```

### 2.4 Running

启动仿真环境

```sh
ros2 launch scorpio_simulator bringup_sim_launch.py
```

> [!NOTE]
> **注意：需要点击 Gazebo 左下角橙红色的 `启动` 按钮**

启动导航

```sh
ros2 launch scorpio_nav2_bringup scorpio_simulation_launch.py slam:=True
```

#### 2.4.1 Test Commands

控制机器人移动

```sh
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## 维护者及开源许可证

Maintainer: Lihan Chen, <lihanchen2004@163.com>

scorpio is provided under Apache License 2.0.
