# lunar_navigation

Scout Mini 的静态二维地图导航。复用 Nav2 Humble 自带的 NavFn、RPP、行为树和生命周期管理；独立于建图、重定位及底盘入口。

完整的硬件准备、建图保存、二维地图转换/修图和三阶段运行顺序见[根 README](../../README.md)。本入口只提供第三阶段的 Nav2，不包含重定位；已有定位进程应继续运行，不重复启动。

## 依赖与编译

```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-nav2-regulated-pure-pursuit-controller ros-humble-diagnostic-updater
colcon build --symlink-install --packages-select lunar_navigation
```

当前开发机安装的 Nav2 为 1.1.20。本包只安装 launch、YAML、RViz 配置，不编译 Navigation2 源码。

本机曾因 diagnostic_updater 4.0.2 与新 bondcpp 不匹配，导致 lifecycle_manager 报缺少 `libdiagnostic_updater.so`；当前已安装 4.0.7，并确认所需库可被正常解析。其他机器若遇到相同错误，执行 `sudo apt update` 后运行上述安装命令更新依赖。

## 启动

每个终端先加载 ROS 与工作空间环境：

```zsh
source /opt/ros/humble/setup.zsh
source install/local_setup.zsh
```

分别启动以下入口，不存在整车总 launch：

```bash
# 终端 1：停止建图后运行；当前安装、水平启动、同层平地的高度初值
ros2 launch lunar_hardware_bringup localization.launch.py \
  pcd:="$PWD/maps/site/map.pcd" base_z:=-0.05738 rviz:=false

# 终端 2：底盘
ros2 launch lunar_hardware_bringup chassis.launch.py port_name:=can0

# 终端 3：二维地图与 Nav2
ros2 launch lunar_navigation navigation.launch.py map:="$PWD/maps/site/map.yaml"
```

上述 `base_z=-0.05738` 来自当前安装及建图 IMU 原点约定，适用于水平启动后在同一层平地重定位；不是任意楼层或额外变换后地图的通用高度。二维地图需要提前生成并人工确认，必须与三维 PCD 共用 map 坐标；PCD 转图工具尚未实现。转换或编辑二维图只影响导航，不修改用于 ICP 的 PCD。

在导航 RViz 中先用 **2D Pose Estimate** 指定 base_link 初始位置和车头朝向，确认三维定位匹配；之后用 **Nav2 Goal** 下发导航目标。初始位姿和导航目标是两个工具。导航 launch 不启动 AMCL、不重复启动雷达/LIO/description/底盘。可先运行无底盘的定位与导航查看地图及 TF，再启动底盘执行。

`map` 为必填二维 YAML 路径；`params_file` 默认为本包 `config/nav2.yaml`，`use_sim_time=false`、`autostart=true`、`rviz=true`。

## 接口与配置

| 输入/输出 | 用途 |
| --- | --- |
| /map | map_server 发布二维 OccupancyGrid |
| map → lidar → body → base_link | 外部三维定位与描述包提供 TF |
| /scout/wheel_odom | 车体实际线/角速度反馈，不用于全局定位 |
| /initialpose | RViz 发给硬件包的桥接节点，按 base_link 解释 |
| /navigate_to_pose | Nav2 导航 action |
| /plan | 全局路径 |
| /cmd_vel_nav → /cmd_vel | Nav2 控制器输出经标准速度平滑后到达底盘；恢复行为沿用官方的 /cmd_vel 接线 |

唯一配置为 `config/nav2.yaml`。全局 costmap 在 map；滚动局部 costmap 在 LIO 局部世界系 lidar；二者均只用 StaticLayer + InflationLayer，不订阅实时雷达、深度或 scan 障碍观测。RPP 和恢复行为保留对静态地图的碰撞检查。未知区域不作为本版规划通道（allow_unknown=false）。

全局 NavFn 配置 `use_astar=false`，采用 Dijkstra 搜索；RPP 跟踪参考路径，不允许沿路径倒车，但默认行为树仍可执行 BackUp 恢复。没有启用基于实时障碍观测的局部避障：局部 costmap 和控制器继续处理静态图中的障碍，新增纸箱、行人不会进入代价图，人工擦掉的障碍也不会被点云补回。

footprint 使用现有车体包络 0.62260 × 0.53302 m，padding=0.02 m。初始线速上限 0.15 m/s、角速上限 0.30 rad/s；到点容差 0.15 m/0.15 rad，膨胀半径 0.55 m。它们是接入阶段的低速初值，并非从旧工程继承的已调参结论。Nav2 标准入口还启动 smoother_server、waypoint_follower、velocity_smoother，保持官方运行接线；本包不增加自定义规划或控制插件。

世界 TF 保留完整三维位姿，二维导航主要使用 XY 与 yaw，当前针对近水平路面。原始 LIO 的 body 速度和缺失角速度不直接作为 Nav2 速度反馈。底盘 TF 隔离由 hardware_bringup 负责。

## 验证范围

先验证包构建、launch 参数、标准插件加载、地图加载、TF 与 ComputePathToPose，再在独立 ROS 域用合成位姿/里程计检查导航 action 和速度输出。实车路径跟踪与停车精度需要现场测试；静态地图不会反映新增的动态障碍。

2026-11-12：两个新包编译及 launch 参数检查通过；隔离 ROS 域中，真实 localizer 使用合成三维点云匹配后，base_link 回到 RViz 指定初值。Nav2 完成生命周期激活、NavFn 规划（38 个路径点）、NavigateToPose 目标接收、RPP/平滑速度输出和取消。没有启动雷达或底盘。该次检查临时从官方 diagnostic_updater 4.0.7 deb 解出运行库使用；之后系统包也已更新到 4.0.7，所需动态库已确认可解析。临时测试数据不在仓库中。
