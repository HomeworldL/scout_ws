# lunar_navigation

Scout Mini 的静态二维地图导航。复用 Nav2 Humble 自带的 NavFn、RPP、行为树和生命周期管理；独立于建图、重定位及底盘入口。

完整的硬件准备、建图保存、二维地图转换/修图和三阶段运行顺序见[根 README](../../README.md)。本入口只提供第三阶段的 Nav2，不包含重定位；已有定位进程应继续运行，不重复启动。

## 依赖与编译

```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-nav2-regulated-pure-pursuit-controller ros-humble-diagnostic-updater
sudo apt install libpcl-dev python3-pil
colcon build --symlink-install --packages-up-to lunar_navigation
```

当前开发机安装的 Nav2 为 1.1.20。本包包含 launch、YAML、RViz 配置、默认第三方转换适配及一个备用离线 PCL 转图工具，不编译 Navigation2 源码。`--packages-up-to` 同时构建第三方 pcd2pgm；首次也可直接运行根目录 ./build.sh。

本机曾因 diagnostic_updater 4.0.2 与新 bondcpp 不匹配，导致 lifecycle_manager 报缺少 `libdiagnostic_updater.so`；当前已安装 4.0.7，并确认所需库可被正常解析。其他机器若遇到相同错误，执行 `sudo apt update` 后运行上述安装命令更新依赖。

## PCD 转二维栅格（默认）

加载 ROS 和工作空间环境后运行：

```bash
ros2 launch lunar_navigation pcd2pgm.launch.py
```

默认使用第三方 [LihanChen2004/pcd2pgm](https://github.com/LihanChen2004/pcd2pgm) 的转换节点发布 `/pcd2pgm/map`，本地 launch 同时运行 Nav2 map_saver_cli 自动导出 PGM＋YAML；不需要手动另开终端保存。默认 PCD 为工作区根目录 maps/site/map.pcd，输出前缀为 maps/site/map，不随 cwd 改变。

`rviz:=false` 不打开预览。保存成功后转换节点继续运行供预览，按 Ctrl+C 结束。再次运行会覆盖同名 YAML/PGM，不修改原 PCD 或 PNG。地图和过滤点云使用 `/pcd2pgm/map`、`/pcd2pgm/cloud`，不覆盖导航的 `/map`。

```bash
ros2 launch lunar_navigation pcd2pgm.launch.py \
  pcd:=/absolute/site/map.pcd output:=/absolute/site/map rviz:=false
```

本地完整配置在 `config/pcd2pgm.yaml`，可通过 params_file:=绝对路径替换。路径由 launch 注入，不修改上游 YAML；PCD 的坐标变换保持六个零，保留与三维定位地图相同的 map 坐标。

初始配置为 0.05 m 分辨率、0.2 m 半径去噪、最少 2 个邻居；保留绝对 Z=-0.13846..0.41154 m 的点，对应当前平地高度约 -0.23846 m 以上 0.10..0.65 m 的障碍带。其他地图高度需调整 thre_z_min/max。flag_pass_through=false 是保留该高度带，true 是反选，不是关闭滤波。

上游将障碍包围框内未标障碍的位置视为自由，不能据此证明所有空白都已观测可通行；边界、未扫描区域和缺失障碍需人工确认。保存的图使用 trinary/negate=0、阈值 0.65/0.196。之后按根 README 转 PNG、人工修图，保留 resolution、origin 与图像尺寸。

第三方源码以子模块固定在提交 `f24907dd61bb33f44a7a3c6b2feb8577935da353`，没有本地源码修改。较新提交引入本机 Humble 的 ament_cmake_auto 尚不支持的 USE_SCOPED_HEADER_INSTALL_DIR；固定版本与最新版的转换算法源码相同。

## 备用转换器

自写工具保留为备用，不参与默认转换 launch：

```bash
ros2 run lunar_navigation pcd_to_grid
```

它同样默认读 maps/site/map.pcd、输出 map.yaml/map.pgm，支持 ASCII、binary、binary_compressed。区别是自动用初始 IMU 附近 3 m 内、z<0 的点拟合近水平地面，地面观测标自由、地面上 0.10..0.65 m 的点标障碍，无观测格保留未知。可用 --ground-z 指定水平地面高度，其余参数见 --help。

两个工具都不修改 PCD，但自由/未知的判断不同；备用工具会覆盖同名 YAML/PGM。生成后检查并选择实际要用于导航的地图。

## 启动

每个终端先加载 ROS 与工作空间环境：

```zsh
source /opt/ros/humble/setup.zsh
source install/local_setup.zsh
```

先单独启动雷达驱动并保持运行，然后分别启动以下入口；不存在整车总 launch：

```bash
ros2 launch lunar_hardware_bringup lidar.launch.py
```

```bash
# 终端 1：停止建图后运行，默认 maps/site/map.pcd，自动推导同层高度
ros2 launch lunar_hardware_bringup localization.launch.py rviz:=false

# 终端 2：底盘
ros2 launch lunar_hardware_bringup chassis.launch.py port_name:=can0

# 终端 3：二维地图与 Nav2
ros2 launch lunar_navigation navigation.launch.py
```

重定位默认 base_z=auto，按 URDF 和初始 IMU 原点推导同层平地初值。导航默认读取 maps/site/map.yaml；其他地图分别传 pcd:=绝对路径和 map:=绝对路径覆盖。二维图必须与三维 PCD 共用 map 坐标，转换或编辑二维图只影响导航，不修改用于 ICP 的 PCD。

在导航 RViz 中先用 **2D Pose Estimate** 指定 base_link 初始位置和车头朝向，确认三维定位匹配；之后用 **Nav2 Goal** 下发导航目标。初始位姿和导航目标是两个工具。导航 launch 不启动 AMCL、不重复启动雷达/LIO/description/底盘。可先运行无底盘的定位与导航查看地图及 TF，再启动底盘执行。

`map` 默认工作区根目录 maps/site/map.yaml；`params_file` 默认为本包 `config/nav2.yaml`，`use_sim_time=false`、`autostart=true`、`rviz=true`。

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

转换回归在构建后执行：

```bash
python3 -m pytest src/lunar_navigation/test -q
```

备用工具回归覆盖三种 PCD 编码、负坐标原点、PGM 上下方向、障碍覆盖优先级、未知格、排除高处回波、近水平地面拟合和无效输入；默认入口回归覆盖第三方节点/自动保存接线、RViz 开关和缺失 PCD 的报错。隔离 ROS 实际验证了第三方转换并自动导出 PGM/YAML，原 PCD 不变，输出地图原点与点云坐标一致。
