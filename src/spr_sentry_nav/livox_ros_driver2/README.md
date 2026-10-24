# Livox ROS Driver 2

本目录是深圳北理莫斯科大学北极熊战队基于 Livox 官方 `livox_ros_driver2` 修改的 ROS2 驱动。

当前包只保留驱动实现、消息定义和内置 Livox-SDK2，不再携带官方 HAP、MID360、混合雷达及 RViz 示例配置。实车不能从本包读取网络参数，也不应使用独立示例 launch。

本工程的实际启动与配置入口为：

- `spr_nav_bringup/launch/bringup_launch.py`
- `spr_nav_bringup/config/reality/nav2_params.yaml`
- `spr_nav_bringup/config/reality/mid360_user_config.json`

驱动相对官方源码的改动、运行链路、已删除示例和 Odin1 + 单 MID360 改造说明见 [MODIFICATIONS_FROM_UPSTREAM.md](MODIFICATIONS_FROM_UPSTREAM.md)。

主要保留能力：

- ROS2 Humble 构建；
- Livox `CustomMsg` 与 `PointCloud2` 发布；
- 单话题双格式发布能力；
- IMU 安装角旋转；
- 基于 `rosbag2_cpp` 的 ROS2 bag2 点云/IMU录制能力（整车新入口不使用）；
- 内置 x86_64 与 ARM64 Livox-SDK2 动态库。

构建：

```bash
colcon build --symlink-install --packages-select livox_ros_driver2
```

## ROS2 bag 录制

驱动保留内部录制能力，但新 bringup 固定使用 `output_data_type=0` 发布话题，不启用内部录制。
整车统一通过主 launch 的 `use_rosbag_record:=true rosbag_path:=rosbags` 录制，完整启动示例见 `script/record.sh`。
输出为 `rosbag_path/rosbag_时间_唯一后缀/` 下的 `.db3` 与 `metadata.yaml`。
按 Ctrl+C 正常结束 launch，等待 rosbag2 收尾；新入口不提供驱动内部或轻量录制模式。

以下为驱动自身保留的内部录制参数，不是主 launch 参数：

```yaml
output_data_type: 1
rosbag_path: "livox_rosbag2"
enable_lidar_bag: true
enable_imu_bag: true
```

内部 `output_data_type=1` 只写 bag、不发布 DDS 话题，不能用于当前导航/建图数据链。
这里的 `rosbag_path` 是待创建且尚不存在的 bag 目录，不同于主 launch 的录制根目录。
`lvx_file_path` 仅恢复原有占位参数，不代表支持 LVX 文件回放。
