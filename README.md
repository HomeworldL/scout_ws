# Scout workspace

面向 Scout Mini、Livox MID360 的 ROS 2 Humble 工作空间，逐步接入三维建图、地图定位和基于静态二维地图的 Nav2 路径跟踪。

当前仅引入第三方源码，尚未添加硬件配置、启动入口或统一构建适配。未修改上游源码。

## 第三方源码

外部仓库统一放在 `src/third_party/`，使用 Git 子模块管理。主仓库的 gitlink 固定具体 commit；`.gitmodules` 中的分支仅记录上游分支，不会在普通初始化时自动更新到分支最新版本。

| 目录 | 上游分支 | 用途 |
| --- | --- | --- |
| [FASTLIO2_ROS2](https://github.com/liangheming/FASTLIO2_ROS2) | `main` | 激光惯性里程计、回环、地图保存与重定位；HBA 暂不接入 |
| [Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2) | `master` | Livox 雷达底层 SDK |
| [livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2) | `master` | MID360 的 ROS 2 驱动 |
| [ugv_sdk](https://github.com/agilexrobotics/ugv_sdk) | `main` | 松灵底盘通信 SDK |
| [scout_ros2](https://github.com/agilexrobotics/scout_ros2) | `humble` | Scout 系列底盘的 ROS 2 驱动 |

首次获取工作空间及子模块：

```bash
git clone --recurse-submodules https://github.com/HomeworldL/scout_ws.git
cd scout_ws
```

已经获取主仓库时，初始化缺少的子模块：

```bash
git submodule update --init --recursive
```

查看锁定版本与本地改动：

```bash
git submodule status --recursive
git status --short
```

## 二进制依赖与构建边界

PCL、Eigen 使用系统软件包。Sophus、GTSAM 优先采用 `ros-humble-sophus`、`ros-humble-gtsam`，不重复引入源码；与所选 FASTLIO2 版本的兼容性留待首次构建验证。

后续目标是在完成必要的构建准备后，从工作空间根目录执行 `colcon build --symlink-install`。当前尚未处理 Livox 的 ROS 2 清单准备、SDK 的构建顺序与路径、以及第三方包的构建元数据，不能将本次源码拉取视为已经构建通过。

不要直接在嵌套目录调用 Livox 的 `build.sh`：该脚本假定固定目录层级，并包含构建目录清理操作。后续会在主仓库完成必要适配。

优先在本仓库维护车辆参数、启动文件和适配代码。若确实需要修改第三方源码，再单独记录可复现的补丁或引用已发布的 fork commit。
