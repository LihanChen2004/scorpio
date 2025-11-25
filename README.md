# scorpio

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![Build and Test](https://github.com/LihanChen2004/scorpio/actions/workflows/build_and_test.yml/badge.svg)](https://github.com/LihanChen2004/scorpio/actions/workflows/build_and_test.yml)
[![pre-commit](https://img.shields.io/badge/pre--commit-enabled-brightgreen?logo=pre-commit)](https://github.com/pre-commit/pre-commit)

## 1. Overview

scorpio 是非官方创建的 [NXROBO-scorpio](https://github.com/NXROBO/scorpio.git) ROS2 版本

## 2. Quick Start

### 2.1 Setup Environment

Ubuntu 22.04: [ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

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

### 2.2.2 Install dependencies

Run the automated setup script:

```sh
./src/scorpio/scripts/setup_dependencies.sh
```

This [script](scripts/setup_dependencies.sh) will install:

- Intel RealSense SDK 2.0 + ROS2 Wrapper
- YDLidar SDK + udev rules
- RGLGazeboPlugin (GPU Lidar simulation)

> [!IMPORTANT]
> After the script completes, **log out and log back in** to apply dialout group permissions.

```sh
rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```

### 2.3 Build

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
