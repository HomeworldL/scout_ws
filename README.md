# Scout workspace

Scout Mini + Livox MID360，Ubuntu 22.04 / ROS 2 Humble。使用 FASTLIO2 建图和三维重定位，使用标准 Nav2 在人工整理的静态二维地图上规划、跟踪路径。当前不接实时障碍观测。

## 运行顺序与地图用途

```text
阶段一：建图 → 保存 map.pcd → 转二维栅格 → 人工修图 → map.yaml + map.png/pgm
阶段二：加载 map.pcd → RViz 指定车身初值 → 三维重定位
阶段三：保留重定位 → 单独启动底盘与 Nav2 → 下发导航目标
```

**三维 PCD 用于重定位；二维 YAML/PNG/PGM 用于导航。** 转二维图、修改图像像素不会改变 PCD 或 ICP 重定位结果，但会改变导航路线和地图碰撞判断。编辑时必须保持二维图与 PCD 的 map 坐标一致，不改变图像尺寸、分辨率和 origin。

| 入口 | 启动内容 | 不包含 |
| --- | --- | --- |
| `lunar_hardware_bringup mapping.launch.py` | 雷达、description、FASTLIO2、PGO、可选 RViz | 底盘、Nav2 |
| `lunar_hardware_bringup localization.launch.py` | 雷达、description、FASTLIO2、localizer、初值桥接、PCD 预览、可选 RViz | 底盘、Nav2 |
| `lunar_hardware_bringup chassis.launch.py` | Scout Mini 底盘驱动、速度反馈 | 雷达、定位、Nav2 |
| `lunar_navigation navigation.launch.py` | 二维 map_server、Nav2、可选 RViz | 雷达、定位、底盘 |

三个业务入口保持独立：**第三个导航 launch 不包含重定位。** 建图与重定位二选一；导航阶段运行“重定位 + 底盘 + 导航”。阶段二已经在运行时，阶段三沿用该定位进程。

当前建图、保存、重定位和 Nav2 接线已有实现及软件验证。**PCD 转二维栅格工具尚未实现，这是从新建 PCD 到完整导航流程中仍缺的一步。** 有与 PCD 对齐的二维地图时可直接进入阶段三；仅验证重定位不需要二维地图。

## 1. 下载与依赖

```bash
git clone --recurse-submodules https://github.com/HomeworldL/scout_ws.git
cd scout_ws
```

已经克隆时初始化子模块：

```bash
git submodule update --init --recursive
```

安装 ROS 2 Humble 后，安装工作空间依赖：

```bash
sudo apt update
sudo apt install \
  build-essential cmake git python3-colcon-common-extensions \
  libpcl-dev libeigen3-dev libyaml-cpp-dev libasio-dev libapr1-dev \
  ros-humble-sophus ros-humble-gtsam \
  ros-humble-pcl-conversions ros-humble-pcl-ros \
  ros-humble-ament-cmake-auto ros-humble-rosidl-default-generators \
  ros-humble-ament-lint-auto ros-humble-ament-lint-common \
  ros-humble-rviz2 ros-humble-robot-state-publisher ros-humble-xacro \
  python3-numpy python3-pil can-utils
```

导航阶段需要：

```bash
sudo apt install \
  ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-nav2-regulated-pure-pursuit-controller ros-humble-diagnostic-updater
```

PCL、Eigen 使用系统库；Sophus、GTSAM 使用 ROS 软件源二进制包，不重复克隆。当前开发机使用 Sophus 1.22.9102、GTSAM 4.2.0、Nav2 1.1.20。Livox-SDK2、UGV SDK 由源码构建到本工作空间，不要求安装到 `/usr/local`。

本机曾因旧 diagnostic_updater 4.0.2 缺少 `libdiagnostic_updater.so` 导致 Nav2 生命周期管理器启动失败，现已更新到 4.0.7。其他机器遇到相同错误时先运行 `sudo apt update`，再执行上面的安装命令更新对应包。

## 2. 编译与终端环境

在工作空间根目录执行：

```bash
./build.sh
```

脚本顺序编译 Livox-SDK2、准备 Livox ROS 2 清单、指定 SDK 路径编译驱动、应用 FASTLIO2 Sophus 链接补丁，再执行全量 `colcon build --symlink-install`。首次准备完成后，日常直接使用：

```bash
colcon build --symlink-install
```

使用 ROS 系统 Python 环境构建；Conda 可能让 CMake 选中缺少 catkin_pkg 的解释器。不要在嵌套目录调用上游 Livox 的 `build.sh`，使用根目录脚本。

下文命令均在工作空间根目录执行。**每个新终端都加载环境**，当前 zsh 使用：

```zsh
source /opt/ros/humble/setup.zsh
source install/local_setup.zsh
```

Bash 用户将 `.zsh` 换成 `.bash`。实车默认 `use_sim_time=false`。

## 3. 硬件准备

### 3.1 MID360 电源与网络

1. 给 MID360 供电，将雷达网线接到运行 ROS 的电脑/车载机。
2. 编辑本仓库的 `src/lunar_hardware_bringup/config/mid360.json`，不必修改第三方驱动配置。当前样例主机为 `192.168.1.5`、雷达为 `192.168.1.12`；将 `host_net_info` 各接收 IP 设为本机雷达网卡地址，将 `lidar_configs[].ip` 设为实际雷达地址。
3. 在系统网络设置中，为雷达连接的网卡配置同一网段的静态 IPv4。例如按当前样例使用 `192.168.1.5/24`；不要与雷达使用相同 IP。
4. 驱动外参 `extrinsic_parameter` 保持零。雷达到车体的安装关系由 URDF 提供，LiDAR–IMU 外参由 FASTLIO2 配置提供。

可先单独验证驱动：

```bash
ros2 launch lunar_hardware_bringup lidar.launch.py
```

另一个已加载环境的终端检查：

```bash
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
```

建图、重定位入口默认自行启动雷达。独立测试结束后停止驱动，再运行业务入口；若保留已运行的驱动，则业务入口传 `start_lidar:=false`。不要重复启动两个雷达驱动。

### 3.2 Scout Mini 电源、CAN 与控制

给底盘供电，连接 CAN 适配器与底盘。只做遥控移动建图或静态重定位时，不需要启动 ROS 底盘驱动；执行 Nav2 速度命令时才需要底盘驱动。

官方驱动使用 SocketCAN，参考波特率为 500 kbit/s。USB-CAN 使用 gs_usb 时，在接口尚未启用的情况下执行：

```bash
sudo modprobe gs_usb
sudo ip link set can0 up type can bitrate 500000
ip -details link show can0
```

内置 CAN 使用相应硬件驱动，不需要 gs_usb；实际接口名不是 can0 时，在命令和后续 launch 中换成实际名字。若看不到反馈，可用 `candump can0` 检查是否收到帧，按 Ctrl+C 结束观察。

在需要软件控制时单独启动：

```bash
ros2 launch lunar_hardware_bringup chassis.launch.py port_name:=can0
```

上游驱动连接成功后会调用 `EnableCommandedMode()`。同时按本车说明设置遥控器/控制模式，急停应处于允许运行状态。该入口订阅 `/cmd_vel`，发布 `/scout/wheel_odom`；可检查：

```bash
ros2 topic hz /scout/wheel_odom
ros2 topic echo /scout/wheel_odom --once
```

底盘参数在 `src/lunar_hardware_bringup/config/scout.yaml`，已选择非全向 Scout Mini。原驱动的 TF 被本地 launch 重映射到 `/scout/wheel_tf`，避免与 LIO 给 base_link 两个父节点；速度反馈仍保留。

## 4. 阶段一：建图、保存与准备导航地图

### 4.1 启动纯 SLAM 建图

```bash
ros2 launch lunar_hardware_bringup mapping.launch.py
```

包含雷达、机器人描述、FASTLIO2、PGO 和 RViz。初始化时保持车静止，等点云与轨迹稳定后再移动采集；可使用实体遥控器移动底盘，不需要 Nav2。

常用参数：`rviz:=false` 关闭显示，`start_lidar:=false` 复用外部驱动。算法配置直接使用当前上游文件：

- `src/third_party/FASTLIO2_ROS2/fastlio2/config/lio.yaml`：LIO、话题、滤波、LiDAR–IMU 外参。
- `src/third_party/FASTLIO2_ROS2/pgo/config/pgo.yaml`：关键帧、回环和地图坐标。

此入口已经启动 LIO，不再另外运行上游 `lio_launch.py` 或 `pgo_launch.py`。

### 4.2 保存三维地图

保持建图运行，另开终端保存：

```bash
mkdir -p maps/site
ros2 service call /pgo/save_maps interface/srv/SaveMaps \
  "{file_path: '$PWD/maps/site', save_patches: false}"
```

`file_path` 为已存在目录的绝对路径。确认服务返回成功、PCD 已写入后，再停止建图入口。目录内容为：

```text
maps/site/
├── map.pcd       三维重定位地图，PGO 保存
├── poses.txt     PGO 保存；save_patches=false 时可能为空
├── patches/      仅 save_patches=true 时输出，供后续离线优化
├── map.yaml      二维导航地图元数据，需另行生成
└── map.png/pgm   二维导航图像，需另行生成
```

当前流程不依赖 HBA。`maps/` 已被 Git 忽略，不提交实测地图。

### 4.3 PCD 转二维栅格

**此步骤的转换工具尚未实现，当前没有可执行的 PCD 转图命令。** PGO 保存服务只生成三维地图；`map_saver_cli` 也不能直接把 PCD 转成二维图。下面是该步骤需要产生的结果，不代表转换已经跑通。

转换需要保留 PCD 的 map 坐标，区分地面和车高范围内的障碍，按 XY 栅格化，输出 `map.yaml` 和灰度图。建议起始分辨率为 0.05 m/格。无点格不能直接全部认定为自由，需要结合地面观测与人工确认。

YAML 至少包含图像路径 `image`、分辨率 `resolution`、左下角原点 `origin`、模式 `mode` 和阈值。使用 `mode: trinary`、`negate: 0`、`occupied_thresh: 0.65`、`free_thresh: 0.196` 时，图像语义为：

| 灰度 / HEX | 含义 |
| --- | --- |
| 0 / #000000 | 障碍 |
| 254 或 255 / #FEFEFE 或 #FFFFFF | 自由 |
| 205 / #CDCDCD | 未知 |

Nav2 支持 PGM 或 PNG，YAML 的 image 指向实际文件。生成时的 resolution 和 origin 必须与图像及 PCD 对齐，不能复制另一场地地图的原点。

### 4.4 人工修图

可使用 VS Code 的 [Luna Paint — Image Editor](https://marketplace.visualstudio.com/items?itemName=Tyriar.luna-paint)：

```bash
code --install-extension Tyriar.luna-paint
```

该扩展编辑 PNG；若已有 PGM，先无损转换格式：

```bash
python3 -c "from PIL import Image; Image.open('maps/site/map.pgm').convert('L').save('maps/site/map.png')"
```

右键 map.png，选择 Open With / Reopen Editor With → Luna Paint。用不带抗锯齿的铅笔或硬边画笔，将确认可通行的假障碍涂白、真实障碍涂黑、未知区域保留灰色。不要使用透明橡皮擦，不缩放、裁剪、旋转或改变画布尺寸。

保存后，将 map.yaml 的 image 改为 map.png，保留原 resolution、origin 和阈值。在 RViz 叠加二维地图与定位点云，确认墙面位置一致。PGM 仅预览还可使用 [PBM/PPM/PGM Viewer](https://marketplace.visualstudio.com/items?itemName=ngtystr.ppm-pgm-viewer-for-vscode)。

**修图影响导航的可走区域和碰撞判断，不影响基于原始 map.pcd 的重定位。** 保存障碍图不会自动更新三维 PCD；也不要为了修改二维路线而改写 PCD。

## 5. 阶段二：单独验证重定位

停止阶段一建图进程，再运行：

```bash
ros2 launch lunar_hardware_bringup localization.launch.py \
  pcd:="$PWD/maps/site/map.pcd" base_z:=-0.05738
```

当前代码保留 base_z 参数。按当前 URDF、建图启动水平且重定位位于同一层平地的约定，采用上面的 -0.05738 m 初值；它来自 body→base_link 的安装高度，不需要每次重新量地面。它是初始猜测，最终三维位姿由 ICP 求解；不同高度楼层或经过额外坐标变换的地图需提供对应初始高度。

启动后保存的 PCD 会先显示在 RViz。操作顺序：

1. Fixed Frame 使用 map。
2. 选择 **2D Pose Estimate**，点击车身 base_link 所在位置，拖动箭头朝车头方向。
3. 桥接订阅 `/initialpose`，通过 URDF 的固定 TF 将 map→base_link 转成 map→body，再调用 `/localizer/relocalize`；重复点击可以重试。
4. 检查当前点云与保存地图是否对齐，并查询初始匹配：

```bash
ros2 service call /localizer/relocalize_check interface/srv/IsValid '{code: 0}'
```

重定位服务接受请求不等于 ICP 匹配成功；上述 valid 表示本次初始化是否曾匹配成功，不是持续定位状态保证。该算法需要接近实际的初值，不是无初值的任意位置全局搜索。阶段二不需要二维地图、不启动底盘或 Nav2。

此入口使用 `src/third_party/FASTLIO2_ROS2/localizer/config/localizer.yaml` 中的两阶段粗/精 ICP 参数；LIO 配置与建图一致。启动后持续运行定位，不能在确认一次匹配后就关闭。

## 6. 阶段三：重定位持续运行并导航

### 6.1 分终端启动

如果阶段二仍在运行，保留它，直接启动底盘与导航；不要重复启动定位。若从零开始，按下面三个终端分别启动，每个终端先加载环境：

```bash
# 终端 A：三维重定位，使用导航 RViz 统一显示
ros2 launch lunar_hardware_bringup localization.launch.py \
  pcd:="$PWD/maps/site/map.pcd" base_z:=-0.05738 rviz:=false
```

```bash
# 终端 B：底盘；CAN 已按第 3 节启用
ros2 launch lunar_hardware_bringup chassis.launch.py port_name:=can0
```

```bash
# 终端 C：二维地图和 Nav2
ros2 launch lunar_navigation navigation.launch.py map:="$PWD/maps/site/map.yaml"
```

导航 launch 自己启动 map_server，但不启动 AMCL、雷达、LIO、localizer、description 或底盘。PCD 与二维 YAML 必须属于同一场地、同一 map 坐标。

### 6.2 下发目标与结束运行

在导航 RViz 中，首次启动先用 **2D Pose Estimate** 完成阶段二的初始匹配；如果已有有效匹配，沿用即可。之后选择 **Nav2 Goal**，点击目标并拖动指定停车朝向。

**2D Pose Estimate 是车辆当前位置的初值，Nav2 Goal 是要去的位置。** 前者触发重定位，后者调用 NavigateToPose；不要用目标箭头代替初始位姿。

运行时可查看 `/plan` 全局路径和 `/scout/wheel_odom` 速度反馈；需要查看局部 costmap 时，在 RViz 添加 Map 显示，话题选择 `/local_costmap/costmap`。Nav2 根据定位 TF 确定车的位置，输出 `/cmd_vel` 给底盘。结束时先在 RViz 取消导航，确认停稳，再分别停止导航、底盘和定位进程。

### 6.3 使用的配置和算法

唯一导航参数文件为 `src/lunar_navigation/config/nav2.yaml`，可通过 `params_file:=绝对路径` 替换，不经过 profile 合成器。

| 模块 | 当前选择 |
| --- | --- |
| 定位 | FASTLIO2 localizer 三维 ICP，持续发布全局修正；不用 AMCL |
| 全局地图 | Nav2 map_server 加载人工整理后的二维 YAML/PNG/PGM |
| 全局规划 | NavFn，use_astar=false，使用 Dijkstra 搜索，不规划穿过未知区 |
| 路径跟踪 | Regulated Pure Pursuit（RPP），允许原地转向，不允许沿路径倒车 |
| 全局 costmap | map 坐标，StaticLayer + InflationLayer |
| 局部 costmap | lidar 局部世界坐标，4 m × 4 m 滚动窗口，同样只有静态层和膨胀层 |
| 行为树 / 恢复 | Nav2 默认重规划/恢复树，spin、backup、drive_on_heading、wait |
| 速度处理 | Nav2 velocity_smoother，OPEN_LOOP；控制器读取底盘实际 twist |
| 车辆包络 | 0.62260 × 0.53302 m 矩形，额外 padding=0.02 m |
| 初始速度与容差 | 线速上限 0.15 m/s，角速上限 0.30 rad/s；到点容差 0.15 m / 0.15 rad |

这些速度、膨胀和到点参数是接入初值，尚未完成实车调参。RPP 不沿路径倒车不代表禁用了默认行为树中的 BackUp 恢复动作。

### 6.4 是否打开局部避障

**没有启用基于实时雷达/深度观测的局部避障。** 两个 costmap 都没有 ObstacleLayer、VoxelLayer 或实时障碍输入，雷达数据只用于 LIO 和定位。

局部 costmap 和 RPP 控制器仍运行：它们根据静态二维地图和车体包络跟踪路径、检查碰撞；`use_collision_detection=true` 保留。因此车仍会考虑地图里已有的墙和障碍，但不会因为实时点云中出现新纸箱或行人而更新障碍、绕行或停车。人工擦掉的障碍同样不会被实时点云补回。

## 7. 坐标与模块来源

```text
map                         保存地图、全局规划、导航目标
└── lidar                   LIO 的局部世界系，承担 odom 的角色
    └── body                MID360 内置 IMU，LIO 动态估计
        ├── lidar_link      雷达点云原点，固定安装外参
        │   └── livox_frame 与 lidar_link 重合
        └── base_link       车身参考，Nav2 使用
            └── 四个固定车轮 link
```

LIO 初始化以 IMU 为局部原点，PGO 初始 map→lidar 为单位变换；车动后 body 随车运动，lidar 保持本次运行参考，PGO 或 localizer 更新全局修正。body 与 livox_frame 轴方向一致，原点相差 `[-0.011,-0.02329,0.04412] m`；该值与 MID360 官方内置 IMU 位置关系相符。雷达 +X 朝车左，安装 yaw=+90° 已由用户确认，安装平移沿用旧工程，未在本轮重新测量。

自有包说明：[机器人描述](src/scout_mini_description/README.md)、[硬件入口与初值桥接](src/lunar_hardware_bringup/README.md)、[导航接口与配置](src/lunar_navigation/README.md)。官方模型固定来源、mesh 及许可记录见 [UPSTREAM.md](src/scout_mini_description/UPSTREAM.md)。

第三方源码统一以子模块放在 `src/third_party/`，主仓库固定具体 commit：

| 仓库 | 分支 | 用途 |
| --- | --- | --- |
| [FASTLIO2_ROS2](https://github.com/liangheming/FASTLIO2_ROS2) | main | LIO、PGO、localizer、服务接口；HBA 暂不用于本流程 |
| [Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2) | master | 雷达底层 SDK |
| [livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2) | master | 雷达/IMU 驱动 |
| [ugv_sdk](https://github.com/agilexrobotics/ugv_sdk) | main | 底盘通信 SDK |
| [scout_ros2](https://github.com/agilexrobotics/scout_ros2) | humble | 底盘驱动及消息；不使用其中的 Scout 2.0 模型 |

FASTLIO2 的唯一现有构建补丁为 `patches/fastlio2-sophus.patch`，为 lio_node 链接 Sophus::Sophus，根脚本幂等应用。`.gitmodules` 的 `ignore=dirty` 仅隐藏该子模块工作区修改提示；提交版本变更仍可见，补丁仍由主仓库保存，不需要 fork。

## 8. 当前验证范围

已完成两个新包编译、launch 参数解析、12 项位姿转换回归，以及隔离 ROS 域的建图入口启动、PCD 预览、真实 localizer 对合成点云的初始匹配、Nav2 生命周期激活、NavFn 路径规划、RPP 速度输出和目标取消。软件联调没有连接雷达或真实底盘。

尚待完成：PCD 转二维图工具、实际场地建图/重定位验证、实车路径跟踪与停车调参。
