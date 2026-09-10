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
- 内置 x86_64 与 ARM64 Livox-SDK2 动态库。

构建：

```bash
colcon build --symlink-install --packages-select livox_ros_driver2
```
