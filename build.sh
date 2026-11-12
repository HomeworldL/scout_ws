#!/usr/bin/env bash
set -e

cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
source /opt/ros/humble/setup.bash

# 1. 编译 Livox-SDK2。
colcon build --symlink-install --packages-select livox_sdk2
source install/local_setup.bash

# 2. 准备 ROS 2 清单，保留已有的本地修改。
if [[ ! -f src/third_party/livox_ros_driver2/package.xml ]]; then
  cp src/third_party/livox_ros_driver2/package_ROS2.xml \
     src/third_party/livox_ros_driver2/package.xml
fi

# 3. 使用当前工作空间的 SDK2 编译 Livox 驱动。
colcon build --symlink-install \
  --packages-select livox_ros_driver2 \
  --cmake-args \
    -DROS_EDITION=ROS2 \
    -DDISTRO_ROS=humble \
    "-DLIVOX_LIDAR_SDK_LIBRARY=$PWD/install/livox_sdk2/lib/liblivox_lidar_sdk_shared.so" \
    "-DLIVOX_LIDAR_SDK_INCLUDE_DIR=$PWD/install/livox_sdk2/include"

# 4. 补齐 FASTLIO2 对 Sophus 的目标链接；已经应用时跳过。
if ! git -C src/third_party/FASTLIO2_ROS2 apply --reverse --check \
    "$PWD/patches/fastlio2-sophus.patch" 2>/dev/null; then
  git -C src/third_party/FASTLIO2_ROS2 apply "$PWD/patches/fastlio2-sophus.patch"
fi

# 5. 编译工作空间中的其余功能包。
source install/local_setup.bash
colcon build --symlink-install
