# Scout workspace

Scout Mini + Livox MID360 的 ROS 2 Humble 工作空间。先复用现有驱动、FASTLIO2 建图和三维地图定位，再接入标准 Nav2。第一版使用静态二维地图规划和路径跟踪，不接实时障碍观测。

当前已引入源码、顺序构建脚本和本车 `scout_mini_description`；尚未接入 Nav2、二维地图转换或统一实车启动。编译通过与实车验收分开记录，不将已有算法实现视为已完成实车建图、定位和导航验证。

## 1. 下载

```bash
git clone --recurse-submodules https://github.com/HomeworldL/scout_ws.git
cd scout_ws
```

已经克隆主仓库时：

```bash
git submodule update --init --recursive
```

外部源码位于 `src/third_party/`，主仓库固定子模块 commit：

| 目录 | 上游分支 | 功能 |
| --- | --- | --- |
| [FASTLIO2_ROS2](https://github.com/liangheming/FASTLIO2_ROS2) | `main` | `fastlio2` 里程计、`pgo` 回环和保存、`localizer` 三维定位、`hba` 离线优化、`interface` 服务定义 |
| [Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2) | `master` | Livox 底层 C++ SDK |
| [livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2) | `master` | 雷达和 IMU 的 ROS 2 驱动 |
| [ugv_sdk](https://github.com/agilexrobotics/ugv_sdk) | `main` | 松灵底盘 C++ SDK |
| [scout_ros2](https://github.com/agilexrobotics/scout_ros2) | `humble` | 底盘驱动、消息和机器人模型 |

`git submodule update --init --recursive` 恢复记录的版本，不自动追随分支最新提交。

本车描述包位于 `src/scout_mini_description/`，直接由主仓库管理。官方模型来源和本车安装计算见[包说明](src/scout_mini_description/README.md)。

## 2. 依赖

平台：Ubuntu 22.04、ROS 2 Humble。当前开发机为 x86_64，车载 ARM64 环境单独构建。

### 2.1 建图、定位与底盘

安装 ROS 2 Humble 后，补齐基础依赖：

```bash
sudo apt update
sudo apt install \
  build-essential cmake git python3-colcon-common-extensions \
  libpcl-dev libeigen3-dev libyaml-cpp-dev libasio-dev libapr1-dev \
  ros-humble-sophus ros-humble-gtsam \
  ros-humble-pcl-conversions ros-humble-ament-cmake-auto \
  ros-humble-rosidl-default-generators ros-humble-ament-lint-auto \
  ros-humble-ament-lint-common ros-humble-rviz2
```

PCL、Eigen 使用系统库；Sophus、GTSAM 使用 ROS 软件源的二进制包，不重复克隆。当前已安装 Sophus `1.22.9102`、GTSAM `4.2.0`。Sophus 的 ROS 编号对应上游 `1.22.10` 系列加维护修改，规则见[维护仓库](https://github.com/clalancette/sophus/blob/release/1.22.x/package.xml)。Livox-SDK2 与 UGV SDK 从子模块构建，不要求安装到 `/usr/local`。

### 2.2 机器人模型

当前模型显示和固定 TF 需要：

```bash
sudo apt install ros-humble-robot-state-publisher ros-humble-xacro
```

### 2.3 后续导航

导航阶段再安装，当前脚本不自动安装依赖：

```bash
sudo apt install \
  ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-nav2-regulated-pure-pursuit-controller
```

Nav2 优先使用二进制包，不克隆或修改 Navigation2 本体。

## 3. 编译

在工作空间根目录执行：

```bash
./build.sh
```

脚本顺序执行：

1. 编译 `livox_sdk2`，安装到 `install/livox_sdk2/`。
2. 从官方 `package_ROS2.xml` 准备 Livox 的 `package.xml`，已有文件保留。
3. 编译 Livox 驱动，传入 ROS2/Humble 参数和工作空间内的 SDK 库、头文件路径。
4. 应用 `patches/fastlio2-sophus.patch`；已经应用时跳过。
5. 加载工作空间环境，执行 `colcon build --symlink-install` 编译全部包，包括 FASTLIO2、PGO、定位、HBA 和底盘包。

补丁只为 `lio_node` 增加 `Sophus::Sophus` 目标链接，使 ROS 二进制 Sophus 的头文件路径正确传递，不改变算法。本机原全量构建日志曾报 `sophus/so3.hpp` 找不到，修复针对该实际错误。子模块显示本地修改是预期状态；主仓库保存补丁，重新克隆后由脚本恢复，不需要 fork。

为减少这项已知补丁产生的提示，`.gitmodules` 仅对 FASTLIO2 设置 `ignore = dirty`：父仓库不再将子模块工作区改动显示为脏状态，子模块 commit 变化仍然显示，进入子模块仍可正常检查 `git diff`。这不是删除修改，也不代替保存补丁；不要把整个目录加入 `.gitignore`。

2026-11-12：当前 x86_64 开发机已通过完整脚本构建，全部 11 个包完成，FASTLIO2 保留上游编译警告。验证时临时降低编译并行度，没有新增工作空间构建配置。未进行本轮实车运行验证。

首次构建完成后，日常增量编译可直接执行：

```bash
colcon build --symlink-install
```

不要在嵌套目录直接运行上游 Livox 的 `build.sh`：它假定固定目录层级并包含构建目录清理操作。当前不引入 `colcon.meta`、`colcon_defaults.yaml` 或额外构建子命令。

## 4. 建图

以下命令均在工作空间根目录执行。每个新终端先加载环境；按 zsh 编写，Bash 用户将 `.zsh` 换成 `.bash`：

```zsh
source /opt/ros/humble/setup.zsh
source install/local_setup.zsh
```

### 4.1 启动雷达

先核对 `src/third_party/livox_ros_driver2/config/MID360_config.json` 的主机和雷达 IP。样例是主机 `192.168.1.5`、雷达 `192.168.1.12`，按实际设备设置。

```bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

输出 `/livox/lidar`（CustomMsg）和 `/livox/imu`。FASTLIO2 当前订阅 CustomMsg，保持该入口的 `xfer_format=1`。

### 4.2 启动带回环的建图

雷达保持运行，另开终端：

```bash
ros2 launch pgo pgo_launch.py
```

该入口已启动 FASTLIO2 和 PGO，不要再同时启动另一个 LIO。配置位于：

- `src/third_party/FASTLIO2_ROS2/fastlio2/config/lio.yaml`：话题、滤波、LiDAR–IMU 外参、坐标系。
- `src/third_party/FASTLIO2_ROS2/pgo/config/pgo.yaml`：关键帧、回环和地图坐标系。

如果只调试里程计而不使用 PGO 保存地图，可改用 `ros2 launch fastlio2 lio_launch.py`。这两个入口选择其一。

主要输出：

| 输出 | 坐标与含义 |
| --- | --- |
| `/fastlio2/body_cloud` | `body`，IMU 参考中的逐帧点云 |
| `/fastlio2/world_cloud` | `lidar`，里程计世界坐标中的逐帧点云 |
| `/fastlio2/lio_odom` | `lidar → body` 位姿及 body 线速度；当前未填角速度 |
| `/fastlio2/lio_path` | `lidar` 中的轨迹 |
| TF `map → lidar → body` | PGO 全局修正和 LIO 连续位姿 |

## 5. 地图保存与路径

推荐每个场地用独立目录：

```text
maps/site/
├── map.pcd       三维定位地图，PGO 保存
├── poses.txt     PGO 保存；save_patches=false 时也会创建，可能为空
├── patches/      仅 save_patches=true 时保存关键帧
├── map.yaml      后续生成的二维地图元数据
├── map.pgm       后续生成的二维灰度栅格图
└── map.png       用于编辑或直接加载的二维图像，可选
```

其中 YAML/PGM/PNG 不是当前 PGO 保存服务的输出。

在工作空间根目录先创建输出目录，再保存：

```bash
mkdir -p maps/site
ros2 service call /pgo/save_maps interface/srv/SaveMaps \
  "{file_path: '$PWD/maps/site', save_patches: false}"
```

服务的 `file_path` 是目录，传绝对路径；当前上游要求目录已存在。重定位使用 `$PWD/maps/site/map.pcd`。

若以后需要 HBA 离线优化，保存时设 `save_patches: true`；当前最简导航不依赖 HBA。人工编辑二维图不会修改三维 PCD，二者分别供导航和点云匹配使用。

## 6. 重定位

停止 PGO/LIO 建图入口，保留雷达驱动，另开终端启动：

```bash
ros2 launch localizer localizer_launch.py
```

该入口也已启动 FASTLIO2。加载保存的 PCD 并给出接近实际位置的初值；角度单位为弧度：

```bash
ros2 service call /localizer/relocalize interface/srv/Relocalize \
  "{pcd_path: '$PWD/maps/site/map.pcd', x: 0.0, y: 0.0, z: 0.0, yaw: 0.0, pitch: 0.0, roll: 0.0}"

ros2 service call /localizer/relocalize_check interface/srv/IsValid '{code: 0}'
```

配置：`src/third_party/FASTLIO2_ROS2/localizer/config/localizer.yaml`。服务成功只表示请求已接受、PCD 已加载；`code: 0` 的结果反映是否完成过初始匹配，不是持续定位新鲜度保证。上游采用带初值的两阶段 ICP，不是无初值的任意位置全局搜索。

默认定位 TF 同样是 `map → lidar → body`。建图和定位选择其一，避免重复发布同一段全局修正。

## 7. PCD 到二维栅格图

**二维栅格图就是导航地图，不是再运行一套二维 SLAM。** `map.pcd` 保存三维点，图像保存二维格子的自由/障碍/未知状态，YAML 保存图像路径、米/像素分辨率、地图原点和阈值。

```text
map.pcd → 选取地面及车高范围、区分地面和障碍
        → 按 XY 栅格化 → map.yaml + map.pgm 或 map.png
        → 人工修整二维图 → Nav2 map_server
```

当前仓库尚无 PCD 转图脚本，本节先明确输出和编辑流程，不提供尚不存在的转换命令。`map_saver_cli` 保存的是已有 OccupancyGrid，不能直接把 PCD 转成二维图。

离线转换需要明确：

- 保持 PCD 的 map 坐标；图像左下原点写进 YAML，不能把地图平移后仍沿用旧 origin。
- 地面不能直接当作障碍投影；可通行地面支持、未知区域与真实障碍应区分。
- PCD 只有累计点，通常没有完整的传感器射线历史；无点格不能自动认定为自由。第一版可以结合地面观测和人工确认处理。
- 可以先用 `0.05 m/格`，实际值写入 YAML；不需要为此引入原工程的高程/地形系统。

通常使用 `mode: trinary`、`negate: 0`，颜色含义如下：

| 灰度 | HEX | 导航语义 |
| --- | --- | --- |
| 0 | `#000000` | 障碍 |
| 254 或 255 | `#FEFEFE` / `#FFFFFF` | 自由 |
| 205 | `#CDCDCD` | 未知，配合常见阈值 occupied_thresh=0.65、free_thresh=0.196 |

PGM 和 PNG 都能作为 Nav2 的地图图像，YAML 的 `image` 指向实际文件。PNG 更方便在 VS Code 编辑，没必要每次转回 PGM。读取和阈值处理见 [Nav2 Humble map_io](https://github.com/ros-navigation/navigation2/blob/humble/nav2_map_server/src/map_io.cpp)。

## 8. VS Code 编辑二维图

推荐安装 [Luna Paint — Image Editor](https://marketplace.visualstudio.com/items?itemName=Tyriar.luna-paint)，扩展 ID：`Tyriar.luna-paint`。它支持 PNG 编辑，不直接支持 PGM。

在 VS Code 的扩展面板 `Ctrl+Shift+X` 搜索上述名字，或在具有 VS Code CLI 的终端执行：

```bash
code --install-extension Tyriar.luna-paint
```

若已有 PGM，先无损转成 PNG。下面只转换图像格式，不从 PCD 生成地图：

```bash
sudo apt install python3-pil
python3 -c "from PIL import Image; Image.open('maps/site/map.pgm').convert('L').save('maps/site/map.png')"
```

编辑步骤：

1. 右键 `map.png`，选择 **Open With / Reopen Editor With → Luna Paint**。
2. 放大，使用铅笔或硬边画笔；把已确认可通行的假障碍涂成白色，固定障碍涂成黑色，未知使用 `#CDCDCD`。
3. 不用透明橡皮擦替代白色画笔；不改变画布尺寸，不缩放、裁剪或旋转，不加抗锯齿或透明度。
4. 保存 PNG；将 `map.yaml` 的 `image` 改为 `map.png`，保留生成时的 resolution、origin、mode、negate 和阈值。
5. 在 RViz 叠加定位点云和地图，确认墙面与路径位置一致。

仅预览 PGM 可安装 [PBM/PPM/PGM Viewer](https://marketplace.visualstudio.com/items?itemName=ngtystr.ppm-pgm-viewer-for-vscode)，ID 为 `ngtystr.ppm-pgm-viewer-for-vscode`。它是查看器，不是画图编辑器，也支持另存 PNG。

## 9. 当前坐标系与 URDF/link 架构

### 9.1 先分清世界坐标和传感器坐标

| 名称 | 当前用途 | 发布或定义位置 |
| --- | --- | --- |
| `map` | 保存地图的全局坐标 | PGO 或 localizer 发布全局修正 |
| `lidar` | **里程计世界坐标**，不是正在移动的雷达本体 | `fastlio2/config/lio.yaml` 的 world_frame |
| `body` | FASTLIO2 的 IMU 参考体坐标 | 同一配置的 body_frame，LIO 发布其三维位姿 |
| `livox_frame` | 原始雷达消息的 frame_id；驱动也用该字符串标 IMU 消息 | Livox 启动文件和驱动 |
| `lidar_link` | 物理雷达安装坐标 | 本车 description 的固定 link |
| `base_link` | 车辆参考坐标，X 前、Y 左、Z 上 | description 按确认的安装关系连接到 body；独立底盘默认也使用这个名字 |
| `base_footprint` | 旧模型的地面参考根节点 | 不属于当前 LIO 输出链 |

当前建图/定位链：

```text
map ──PGO 或 localizer──> lidar ──FASTLIO2──> body
```

当前独立底盘链：

```text
odom ──scout_base──> base_link
```

原始驱动把激光和 IMU 都标为 `livox_frame`，不意味着它们与 LIO 的 `body` 原点相同。FASTLIO2 内部用 `p_body = R_IL × p_lidar + t_IL` 变换点；当前 `R_IL=I`、`t_IL=[-0.011,-0.02329,0.04412] m`，描述 LiDAR 原点在 IMU/body 中的位姿。

后续可将 world_frame 从 `lidar` 改名为 `odom`，同时核对 PGO/localizer 和显示配置；改名字不改变算法，也不能替代安装外参。

### 9.2 三个模型来源不是同一件事

| 来源 | 判断 |
| --- | --- |
| 当前子模块 `scout_ros2/scout_description` | 官方提供的包；当前默认入口加载 Scout 2.0 的 `scout_v2.xacro`，不是 Mini 模型 |
| 官方 [ugv_gazebo_sim/humble/scout/scout_description](https://github.com/agilexrobotics/ugv_gazebo_sim/tree/humble/scout/scout_description) | 包含 `scout_mini.xacro`、`mini.xacro` 和 Mini mesh，是官方 Mini 模型参考来源；含仿真内容，不直接整仓叠加 |
| 旧 nav2 工程 `assets/robots/scout_mini/urdf/` | 实测尺寸整理的本车模型，包含仿真和装车内容，不是未经修改的官方原版；当前 scout_ws 中还没有这个目录 |
| 本仓库 `src/scout_mini_description` | 采用固定版本的官方 mesh 和本车实测尺寸，整理了 body、base_link、车轮及 MID360；去掉仿真和机械臂内容，不新增子模块 |

两个官方仓库中的模型包都叫 `scout_description`，未经筛选同时放进当前 src 会造成重名。

旧工程的 `scout_mini_base.urdf.xacro` 定义底盘，`tzb_lunar_sampler.urdf.xacro` 组合底盘、Piper 外观、相机、雷达、IMU 和仿真插件。旧模型的尺寸可作本车描述的基础，但需要整理后再迁移：去掉实车不需要的 Gazebo/轮胎柔顺内容，修正 `package://base_sim_explor`、`package://piper_description` 资源引用，补全与 FASTLIO2 body 的关系。

### 9.3 旧实测模型的 link 树

已展开旧 xacro 核对，根节点为 base_footprint，没有名为 body 的 link：

```text
base_footprint
└── base_link                       z=+0.18108 m
    ├── 四个 tire_carrier           径向柔顺仿真关节
    │   └── 对应 wheel_link         连续旋转关节
    ├── piper_base_link             固定安装，零位 mesh 外观
    ├── rgbd_camera_link
    ├── lidar_link                  xyz=[0.202,0,0.10150] m，yaw=-90°
    │   └── livox_frame              单位变换
    ├── imu_link                    xyz=[0,0,0.07550] m
    └── fill_light_link
```

其中 `imu_link` 的位置是旧模型定义，不能直接改名为 FASTLIO2 的 `body`：后者是装在雷达中的 IMU 参考点，需根据 LiDAR–IMU 外参和实际安装关系计算。

旧 URDF 写的是 `base_link → lidar_link` yaw=-90°；旧实车 `vehicle_mount.py` 写的是 `body → base_footprint` yaw=-90°，在当前 `R_IL=I` 条件下反推 `base_link → LiDAR` 为 +90°。本轮用户已确认 LiDAR 自身 +X 朝车身左侧，因此采用安装 yaw=+90°，新 description 与旧实车安装代码一致，不沿用旧仿真 URDF 的 -90°。

### 9.4 本车 description 的连接方式

当前本车 description 以 body 为根，直接接在现有 LIO 的动态 TF 下：

```text
map
└── odom / 当前的 lidar             PGO 或 localizer
    └── body                        FASTLIO2
        ├── lidar_link              LiDAR–IMU 固定外参
        │   └── livox_frame
        └── base_link               由安装关系计算的固定变换
            └── 四个固定车轮 link
```

body→base_link 为 `xyz=[-0.011,0.17871,-0.05738] m`、yaw=-90°；body→lidar_link 使用当前 LiDAR–IMU 外参。两段组合后，base_link→lidar_link 为 `xyz=[0.202,0,0.10150] m`、yaw=+90°。不需要为了导航把 LIO 的 roll/pitch 清零。base_footprint 第一版不引入。

构建后可独立显示模型，或与 LIO/定位一起使用：

```bash
ros2 launch scout_mini_description description.launch.py
# 只发布 robot_description 和固定 TF，不打开 RViz
ros2 launch scout_mini_description description.launch.py rviz:=false
```

默认 RViz Fixed Frame 为 body，用于独立模型预览；与定位点云一起显示时改为 map。该入口不发布动态位姿，也不启动底盘。启动 LIO 和本车 description 时，不要再加载旧 Scout 2.0 的 description。

RViz 自动加载本包 `description.rviz`，相机工具、Orbit 视角及面板沿用系统默认配置，仅增加模型/TF 显示并设置 Fixed Frame。工具栏选择 Move Camera 后拖动可旋转视角。

若要保留旧 URDF 的 `base_footprint → base_link → 传感器` 根结构，则需要将 LIO 位姿换算为 `odom → base_footprint` 或 `odom → base_link`，并隔离原始 `odom → body` TF；不能在保留原树的同时再给 base_link 或 body 添加第二个父节点。

当前实现选择 body 根结构，不同时加载旧根结构。模型和静态 TF 已完成软件验证；实际安装位置和导航表现仍需实车核对。Nav2 的二维 footprint 单独按车体包络配置，不从 mesh 自动推导。

## 10. 下一步导航范围

使用标准 map_server、NavFn、RPP、StaticLayer/InflationLayer、BT Navigator 和 lifecycle manager；local costmap 仍保留，但暂不接实时障碍点云。不启动 AMCL，沿用三维定位。先把二维地图、车身 TF 和底盘速度反馈接通，不迁移旧工程的高程地形、自定义规划控制和探索体系。
