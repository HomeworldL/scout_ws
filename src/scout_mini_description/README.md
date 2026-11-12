# scout_mini_description

本车 Scout Mini + MID360 的实车描述包，直接由主仓库管理。上游模型、固定版本与版权说明见 [UPSTREAM.md](UPSTREAM.md)。

## 依赖与编译

ROS 2 Humble、ament_cmake、xacro、robot_state_publisher、RViz2：

```bash
sudo apt install ros-humble-xacro ros-humble-robot-state-publisher ros-humble-rviz2
colcon build --symlink-install --packages-select scout_mini_description
```

根目录 `./build.sh` 的全量构建步骤也会自动包含本包。

使用 ROS 的系统 Python 环境构建。当前开发机若激活 Conda，CMake 可能选到不含 catkin_pkg 的 Python；本轮首次构建通过 `--cmake-args -DPython3_EXECUTABLE=/usr/bin/python3` 指定系统 Python，后续构建沿用该缓存。

## 启动

在工作空间根目录，按当前 shell 加载环境：

```zsh
source /opt/ros/humble/setup.zsh
source install/local_setup.zsh
ros2 launch scout_mini_description description.launch.py
```

默认同时打开 RViz，Fixed Frame 为 body；只发布模型和固定 TF：

```bash
ros2 launch scout_mini_description description.launch.py rviz:=false
```

RViz 仍加载本包的 `rviz/description.rviz`。该文件以系统 `rviz_common/default.rviz` 为基础，只修改 Fixed Frame 并增加 RobotModel、TF 显示；相机工具、Orbit 视角、窗口和面板使用系统默认值。需要旋转时选择工具栏 Move Camera，左键拖动旋转、滚轮缩放。修改配置后关闭并重新启动 RViz，使配置重新加载。

## 输入、输出与功能

- 输入是本包 xacro 的安装尺寸参数；车轮采用固定关节，不需要 joint_states。
- 输出 `/robot_description` 与 `/tf_static`。不发布 map/odom 到 body 的动态位姿，不启动雷达或 CAN 驱动。
- 有 LIO 时，完整链由动态 `map → lidar/odom → body` 加下述固定子树组成；独立预览则以 body 为 Fixed Frame。

```text
body                       FASTLIO2 IMU 参考；URDF 根节点
├── base_link              车体中心，X 前、Y 左、Z 上
│   ├── front_left_wheel_link
│   ├── rear_left_wheel_link
│   ├── front_right_wheel_link
│   └── rear_right_wheel_link
└── lidar_link             物理雷达参考
    └── livox_frame        原始雷达消息 frame_id
```

默认不包含 base_footprint、独立 imu_link、相机、Piper、Gazebo 或轮胎柔顺仿真。雷达 link 当前只表示坐标，不用示意几何冒充实测传感器外形。底盘独立驱动默认发出的 odom→base_link 需要在后续集成时隔离，避免与这棵树给 base_link 两个父节点。

## 尺寸、安装与计算

| 参数 | 当前值与来源 |
| --- | --- |
| 整车平面包络 | 0.62260 × 0.53302 m，旧工程本车测量 |
| base_link 离地高度 | 0.18108 m |
| 轮胎 | 半径 0.085 m、宽度 0.080 m |
| base_link → LiDAR 平移 | [0.202, 0, 0.10150] m，沿用旧工程安装参数，本轮未重新测量 |
| base_link → LiDAR yaw | +90°，用户确认 LiDAR +X 朝车身左侧 |
| body → LiDAR | R=I，t=[-0.011,-0.02329,0.04412] m，与当前 lio.yaml 的 r_il/t_il 一致 |

以 T_A_B 表示 B 在 A 中的位姿：`T_body_base = T_body_lidar × inverse(T_base_lidar)`。当前安装 roll/pitch 为零，计算得：

```text
body → base_link: xyz=[-0.011, 0.17871, -0.05738] m，yaw=-90°
body → lidar_link: xyz=[-0.011,-0.02329,0.04412] m，RPY=[0,0,0]
lidar_link → livox_frame: 单位变换
```

两条路径组合后，base_link→lidar_link 必须恢复为 `[0.202,0,0.10150]` 和 yaw=+90°。这也解释了为什么旧实车的 body→base yaw=-90° 与用户确认的安装一致，旧仿真 URDF 的安装符号需要调整。

雷达平移可追溯到旧工程 `assets/robots/scout_mini/urdf/tzb_lunar_sampler.urdf.xacro` 的 `lidar_joint`，以及 `src/lunar_hardware_bringup/lunar_hardware_bringup/vehicle_mount.py`。旧代码注释称其为实测安装位置，目前未核实原始测量记录。这些不是松灵官方模型给出的 MID360 安装参数。

`lidar_link` 与 `livox_frame` 完全重合；它们与 `body` 方向一致，但原点相差上述 LiDAR–IMU 平移。LIO 的 `lidar` 是本次启动的局部世界系，承担类似 odom 的角色，不跟随车身运动；`body` 随车运动。重定位使用保存 PCD 的 map 坐标，更新 map→lidar 修正，不重置 LIO 的局部原点。

参数目前在 `urdf/scout_mini.urdf.xacro` 集中维护；launch 额外提供 `lidar_yaw`（弧度，默认 pi/2）及 `use_sim_time`。改变 LiDAR–IMU 标定或实际安装时，应同步核对本包和 FASTLIO2 配置；当前公式针对 r_il=I、安装 roll/pitch=0，不自动支持其他外参约定。

官方 mesh 保留原文件，按旧实测模型的方法居中和缩放；车壳形状是近似外观，碰撞使用简单箱体/圆柱。Nav2 footprint 应单独配置为真实车体包络，不由该视觉模型自动生成。

## 验证

```bash
xacro src/scout_mini_description/urdf/scout_mini.urdf.xacro > /tmp/scout_mini.urdf
check_urdf /tmp/scout_mini.urdf
```

预期只有一个根节点 body，8 个 link、7 个固定 joint；所有 mesh 使用本包 package URI。模型和 TF 验证不代表完成实车运动或导航验收。

2026-11-12：已通过本包编译、xacro/URDF 解析、安装变换组合和独立 ROS 域中的 robot_state_publisher 验证，实际接收到全部 7 段固定 TF；base_link→livox_frame 的平移与 +90° 安装方向一致。两份 mesh 的 SHA256 与固定上游版本一致。
