# 上游来源

模型资源来源：[agilexrobotics/ugv_gazebo_sim](https://github.com/agilexrobotics/ugv_gazebo_sim)，分支 `humble`，固定提交 `9d7888affce35321c657c1c8528307f3115b9d41`。

| 本地文件 | 上游相对路径 | SHA256 |
| --- | --- | --- |
| meshes/scout_mini_base_link2.dae | scout/scout_description/meshes/scout_mini_base_link2.dae | c91ba00c5822e49443202ef9cc9b512e210cca0e1c75ce91db5a8a26821dc6ad |
| meshes/wheel.dae | scout/scout_description/meshes/wheel.dae | 71d7c2c16c03782d127b84fcddba7393f057566885a20c1257dcfbbf2e65439c |

两份 mesh 保持下载内容不变。URDF 参考上游 `urdf/mini.xacro` 和 `scout_mini_wheel_1.xacro` 至 `_4.xacro` 的模型与轮组结构，整理为一个本地 xacro；mesh 引用改为本包，轮子改为固定关节，移除 Gazebo、动力学和多余传感器。上游轮组文件的 BSD 版权与许可声明保留在 `LICENSE`。

车体与轮胎包络、mesh 居中/缩放方法、雷达平移来自用户提供的旧工程 `assets/robots/scout_mini/urdf/`，不使用官方仿真模型的尺寸代替本车测量。雷达安装 yaw=+90° 由用户在本轮明确确认。LiDAR–IMU 外参对应当前 FASTLIO2 配置。

雷达平移具体沿用 `tzb_lunar_sampler.urdf.xacro` 中 `lidar_joint` 的 `[0.202, 0, 0.10150] m`；旧实车 `vehicle_mount.py` 也采用相同位置。旧代码将其称为实测安装位置，本轮未重新测量，也未核实原始测量记录。旧 URDF 的 yaw=-90° 与旧实车变换存在差异，当前以用户确认的 +90° 为准。

## 许可范围

本包整理代码采用 BSD-3-Clause。上游轮组代码保留 Clearpath Robotics、Weston Robot 的 BSD 声明。上游 `scout_description/package.xml` 的许可字段为 `TODO: License declaration`，上述两个 DAE 未附可单独确认的许可声明；`LicenseRef-Upstream-Meshes` 表示这一资源来源和许可状态，不将它们额外声称为本包 BSD 许可下的作品。
