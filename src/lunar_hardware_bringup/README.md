# lunar_hardware_bringup

Scout Mini + MID360 的实车启动与接口适配。建图、定位、底盘分别启动；不包含导航控制器，不修改上游算法。

从硬件供电/网卡/CAN 准备，到建图保存、二维地图处理、单独重定位及导航的执行顺序，见[根目录完整运行流程](../../README.md)。建图和定位二选一；导航阶段保持定位运行，另外启动底盘与导航入口。

## 依赖与编译

ROS 2 Humble，以及工作空间中的 Livox 驱动、FASTLIO2、PGO、localizer、interface、scout_base、scout_mini_description。

```bash
sudo apt install ros-humble-pcl-ros python3-numpy
./build.sh
```

首次完整构建后，修改本包可单独编译：

```bash
colcon build --symlink-install --packages-select lunar_hardware_bringup
```

每个终端在工作空间根目录加载环境，Bash 用户将 zsh 换成 bash：

```zsh
source /opt/ros/humble/setup.zsh
source install/local_setup.zsh
```

## 雷达网络配置

`config/mid360.json` 从当前官方驱动样例复制，主机 `192.168.1.5`、雷达 `192.168.1.12`，使用前按设备修改。保持驱动外参为零，雷达到车体的安装变换由 description 提供，LiDAR–IMU 外参由 LIO 配置提供。

雷达驱动与算法入口已解耦。先单独启动一次，并在建图或重定位期间保持运行：

```bash
ros2 launch lunar_hardware_bringup lidar.launch.py
```

再另开终端选择建图或重定位；两个入口只订阅已有 `/livox/lidar`、`/livox/imu`，不会再启动驱动。切换建图/定位时保留驱动，不再提供 start_lidar 参数。播放含这些话题的录包时，不启动实物雷达驱动。

## 1. 纯 SLAM 建图

```bash
ros2 launch lunar_hardware_bringup mapping.launch.py
```

先保持单独的雷达驱动运行；本入口启动 description、LIO、PGO 和可选 RViz，不启动底盘或 Nav2。输出 `/fastlio2/body_cloud`、`/fastlio2/world_cloud`、`/fastlio2/lio_odom`、TF `map → lidar → body` 和描述包静态子树。

保存三维地图仍使用上游服务：

```bash
mkdir -p maps/site
ros2 service call /pgo/save_maps interface/srv/SaveMaps \
  "{file_path: '$PWD/maps/site', save_patches: false}"
```

输出 `map.pcd`、`poses.txt`；不生成二维 YAML/PGM。

## 2. 已有地图重定位

先停止建图入口，保留雷达驱动，再启动：

```bash
ros2 launch lunar_hardware_bringup localization.launch.py
```

本入口启动 description、LIO、localizer、initial_pose_bridge 和 PCL 自带的 PCD 显示节点，不启动雷达驱动。默认 PCD 为工作区根目录 maps/site/map.pcd，不受终端目录影响；可传 pcd:=绝对路径覆盖。保存地图显示在 `/localization/reference_cloud`、坐标为 map，点击前就可查看；它只供显示，不参与匹配，也不启动导航。`rviz:=false` 可关闭本入口的 RViz。

在 RViz 将 Fixed Frame 设为 **map**，选择 **2D Pose Estimate**，在地图上点击车身 base_link 的位置并拖动箭头朝向车头。箭头通过 `/initialpose`（PoseWithCovarianceStamped）送入桥接节点；桥接调用现有 `/localizer/relocalize`，重复点击可重新初始化。启动时不会自动发起重定位。

### base_link 到 body 的转换

桥接读取 URDF 的静态 TF，计算 `T_map_body = T_map_base_link × T_base_link_body`，不重复硬编码雷达安装位置或 90° 偏航。

2D Pose Estimate 不提供地面高程。`base_z` 默认字符串 auto：使用 URDF 固定 TF 和建图初始化 body 高度为 0 的约定，计算同层平地的 base_link 高度，然后转换为 body 初值；当前安装的车身高度初值约 -0.05738 m，无需手动测量或硬编码雷达偏移。此时忽略 RViz 消息 z。若实际位于不同高度，可传 `base_z:=数值` 明确指定 map 中 base_link 的绝对高度。朝向取消息四元数，三维 ICP 随后求实际位姿。协方差不传给上游，因为其服务没有该字段。

当前安装在车头沿 map +X 时，body 初值相对 base_link 的水平偏移为 `[0.17871, 0.011] m`，yaw 增加 90°；这不是把 RViz 箭头直接当成 body。角度转换遵守上游 `Rz(yaw) × Rx(roll) × Ry(pitch)` 顺序。

服务返回 success 表示地图和初值已接受，不表示 ICP 已匹配成功。检查本次初始匹配：

```bash
ros2 service call /localizer/relocalize_check interface/srv/IsValid '{code: 0}'
```

确认点云与保存地图对齐后再使用导航。上游使用带初值 ICP，箭头应接近实际位置和朝向；没有实现任意位置全局搜索。

## 3. 底盘单独启动

```bash
ros2 launch lunar_hardware_bringup chassis.launch.py port_name:=can0
```

CAN 接口需事先按实车配置启用。参数位于 `config/scout.yaml`，固定使用非全向 Scout Mini。底盘订阅 `/cmd_vel`，发布 `/scout/wheel_odom`，其 twist 在 base_link 中，包含线速度和角速度；Nav2 用这部分速度反馈，不用其积分位姿代替 LIO。

上游无关闭里程计 TF 的参数，本入口将该节点的 `/tf` 重映射到 `/scout/wheel_tf`。因此 `/scout/wheel_odom` 仍可用，而公共 TF 只有 LIO 与 description 提供的 `map → lidar → body → base_link`。不加载上游 Scout 2.0 模型。

## 参数

| 参数 | 默认 | 入口 |
| --- | --- | --- |
| rviz | true | 建图、重定位 |
| use_sim_time | false | 所有入口 |
| lidar_config | 本包 config/mid360.json | 仅独立雷达入口 |
| lidar_yaw | pi/2 | 建图、重定位，传给 description |
| lio_config | 上游 fastlio2/config/lio.yaml | 建图、重定位 |
| pgo_config | 上游 pgo/config/pgo.yaml | 建图 |
| localizer_config | 上游 localizer/config/localizer.yaml | 重定位 |
| pcd | 工作区根目录 maps/site/map.pcd | 重定位 |
| base_z | auto，由安装 TF 推导；也可指定数值 | 重定位 |
| port_name | can0 | 底盘 |

坐标约定保持上游 `world_frame=lidar`、`body_frame=body`、`map_frame=map`；改名时要同步检查 description、桥接与导航配置。

## 验证

回归包含车身四个航向、安装平移、非平面旋转、上游角度顺序、自动高度及无效位姿；另对建图/重定位两个入口分别检查默认、显式 true 和 false 的 RViz 开关。description 的 include 使用独立作用域，内部 rviz=false 不覆盖外层参数：默认或 true 只启动业务入口的一个 RViz，false 不启动 RViz。

```bash
python3 -m pytest src/lunar_hardware_bringup/test -q
```

这些软件检查与实车建图、匹配和运动调试分别进行。

RViz 开关回归由真实 LaunchService 展开嵌套 include、计算条件，仅拦截节点进程的启动，不连接硬件或打开 GUI；两个入口共 6 个用例，同时检查默认 PCD 路径、自动高度和不启动雷达驱动。

2026-11-12：两个包编译、四个硬件入口的 launch 参数解析和 12 项位姿转换回归通过。关闭驱动/RViz 的纯建图入口成功启动 description、LIO、PGO 并提供保存服务。另在隔离 ROS 域实际启动重定位入口，用合成点云验证保存地图在点击前可见、点击前不发起重定位、点击后上游 ICP 匹配成功，所得 map→base_link 与点击位置/车头方向一致。未连接真实传感器或底盘。
