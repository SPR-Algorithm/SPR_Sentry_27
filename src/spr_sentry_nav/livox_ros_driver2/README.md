# Livox ROS Driver 2

本目录是深圳北理莫斯科大学北极熊战队基于 Livox 官方 `livox_ros_driver2` 修改的 ROS2 驱动。

当前包只保留驱动实现、消息定义和内置 Livox-SDK2，不再携带官方 HAP、MID360、混合雷达及 RViz 示例配置。实车不能从本包读取网络参数，也不应使用独立示例 launch。

本工程的实际启动与配置入口为：

- `spr_nav_bringup/launch/rm_navigation_reality_lio_launch.py`
- `spr_nav_bringup/config/reality/nav2_params.yaml`
- `spr_nav_bringup/config/reality/mid360_user_config.json`

驱动相对官方源码的改动、运行链路、已删除示例和 Odin1 + 单 MID360 改造说明见 [MODIFICATIONS_FROM_UPSTREAM.md](MODIFICATIONS_FROM_UPSTREAM.md)。

主要保留能力：

- ROS2 Humble 构建；
- Livox `CustomMsg` 与 `PointCloud2` 发布；
- 单话题双格式发布能力；
- IMU 安装角旋转；
- 基于 `rosbag2_cpp` 的 ROS2 bag2 点云/IMU录制；
- 内置 x86_64 与 ARM64 Livox-SDK2 动态库。

构建：

```bash
colcon build --symlink-install --packages-select livox_ros_driver2
```

## ROS2 bag 录制

驱动内部录包已经使用 `rosbag2_cpp`，输出是可由 `ros2 bag info` 和 `ros2 bag play` 读取的标准 ROS2 bag2。参数为：

```yaml
output_data_type: 1
rosbag_path: "livox_rosbag2"
enable_lidar_bag: true
enable_imu_bag: true
```

`rosbag_path` 是待创建的 bag 目录，启动前不能已经存在。`output_data_type=1` 为“只写 bag、不发布 DDS 话题”；正常导航仍使用 `output_data_type=0`。

如果需要在正常发布话题的同时录制整套系统数据，应保持 `output_data_type=0`，在另一个终端使用：

```bash
ros2 bag record /red_standard_robot1/livox/lidar_merged \
  /red_standard_robot1/livox/imu_192_168_1_194
```

停止录制时使用 `Ctrl+C`/SIGINT，让 rosbag2 正常关闭并写入 `metadata.yaml`。
