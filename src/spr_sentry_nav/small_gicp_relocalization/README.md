# small_gicp_relocalization

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![Build](https://github.com/SMBU-PolarBear-Robotics-Team/small_gicp_relocalization/actions/workflows/build_and_test.yml/badge.svg?branch=main)](https://github.com/SMBU-PolarBear-Robotics-Team/small_gicp_relocalization/actions/workflows/build_and_test.yml)

A simple example: Implementing point cloud alignment and localization using [small_gicp](https://github.com/koide3/small_gicp.git)

Given a registered pointcloud (based on the odom frame) and prior pointcloud (mapped using [pointlio](https://github.com/SMBU-PolarBear-Robotics-Team/Point-LIO) or similar tools), the node will calculate the transformation between the two point clouds and publish the correction from the `map` frame to the `odom` frame.

## Dependencies

- ROS2 Humble
- small_gicp
- pcl
- OpenMP

## Build

```zsh
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

git clone https://github.com/SMBU-PolarBear-Robotics-Team/small_gicp_relocalization.git

cd ..
```

1. Install dependencies

    ```zsh
    rosdepc install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
    ```

2. Build

    ```zsh
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=release
    ```

## Usage

1. Set prior pointcloud file in [launch file](launch/small_gicp_relocalization_launch.py)

2. Adjust the transformation between `base_frame` and `lidar_frame`

    The `global_pcd_map` output by algorithms such as `pointlio` and `fastlio` is strictly based on the `lidar_odom` frame. However, the initial position of the robot is typically defined by the `base_link` frame within the `odom` coordinate system. To address this discrepancy, the code listens for the coordinate transformation from `base_frame`(velocity_reference_frame) to `lidar_frame`, allowing the `global_pcd_map` to be converted into the `odom` coordinate system.

    The node waits for this transform before preparing the map; it does not use a fabricated identity transform.

3. Run

    ```zsh
    ros2 launch small_gicp_relocalization small_gicp_relocalization_launch.py
    ```


## 高度裁剪（2026-10-26）

使用现有导航主参数文件
`spr_nav_bringup/config/reality/nav2_params.yaml`：

```yaml
small_gicp_relocalization:
  ros__parameters:
    height_filter:
      max_z: 2.0
    max_iterations: 30
    min_inlier_ratio: 0.1
```

- `height_filter.max_z`：map 坐标 z 上限，单位 m；仅删除 `z > max_z`，等于上限保留。
  **设为 `0.0` 时不做高度裁剪**，用于完整点云对照；仍排除 NaN/Inf。
  导航 YAML 默认 2.0；未提供此参数的独立/旧 launch 默认 0.0，保留完整高度。
  参数在节点启动时读取，修改 YAML 后需重新加载节点。
- **完整建图，GICP 双端裁剪**：Point-LIO 内部匹配、发布点云与原始 PCD 不受影响。
  加载 PCD 后先沿用现有 `base_frame <- lidar_frame` 变换，再裁剪内存中的地图副本；
  实时输入必须在 `odom_frame`，用当前 `map <- odom` 初值计算高度，
  保留点的坐标仍在 odom 中，交给 GICP 求 map←odom。
  两端均在降采样、邻域协方差计算之前裁剪，不把 z 压成零，也不覆盖地图文件。
- 2m 相对 map 原点，不是雷达自身 z，也不是当地离地高度。
  地图格式仍须遵守本包已有的 lidar_odom PCD 约定；
  已经转换到 map 的外部地图不能再未经核对套用同一外参。
  斜坡、台阶、地图原点高度及雷达安装倾角会影响实际保留的区域。

### 为什么删点后仍可能更慢或不收敛

1. **坐标/重叠不一致**：在倾斜雷达 z 和 map z 上分别截断，
   相当于保留不同区域。现在统一按 map 高度判断。
   实时云使用当前位姿初值，初值的 z、roll/pitch 错误仍会导致边界附近误删；
   本功能不是全局重定位，仍需要合理初始位姿。
2. **几何约束减少**：高处梁柱、墙面拐角可能是稳定特征；
   删除后只剩地面或重复平行墙时，水平平移/偏航等方向可能约束很弱。
   内点多、优化器报告收敛，都不等于位姿正确。增加迭代不能创造缺失的结构。
   GICP 的局部平面协方差和最大对应距离机制见
   [原论文](https://www.robots.ox.ac.uk/~avsegal/resources/papers/Generalized_ICP.pdf)。
3. **迭代预算**：原实现每轮写死 10 次，未收敛的结果被丢弃，
   下轮仍从最后成功的位姿开始，可能反复达到上限。
   改为 `max_iterations`，导航默认 30。
   这是更充分且有界的局部优化预算，不是对收敛效果的保证；
   更高上限也可能增加失败帧耗时。
4. **计算量与邻域**：删除了协方差计算后额外构建、配准并未使用的源 KD 树，
   降采样沿用配置的线程数。现有源/地图叶长仍为 0.05/0.1m，
   未凭合成试验改动这些现场参数。
5. **对应距离单位**：`max_dist_sq` 是距离平方，单位 m²；
   当前 3.0 对应约 1.73m，不是 3m。太大易误匹配，太小会削弱初值捕获范围。

### 失败保护与诊断

过滤/降采样后少于 `num_neighbors` 个点时跳过匹配；
有效输出还要求优化器收敛、至少 `num_neighbors` 个内点、
内点比例达到 `min_inlier_ratio`、变换/信息矩阵/误差有限。
这仅是基础输出门槛，不能排除重复场景误匹配或所有几何退化。
新版 bringup 使用 `publish_tf=false`，失败时不发送新的定位修正，
交由 TF 管理器保持旧值；旧版直接 TF 模式仍可能广播最后的变换，
不能把持续 TF 当成新匹配成功。

启动日志记录地图过滤前后点数；debug 日志记录实时云裁剪前后与降采样点数、
内点比例、迭代次数、预处理/配准耗时。拒绝匹配时以节流 warn 输出原因。
不会在失败时自动改阈值、恢复高处点或接受未收敛的中间结果。

### 离线验证与后续实测

`test_height_filter` 覆盖阈值边界、0 不裁剪、无效点、不同坐标系下保留点的一致性、
已知位姿三维配准，以及完整节点的外参初始化、空云/错误帧/无重叠输入拒绝和有效结果发布。
节点测试仅使用合成点云及静态外参，不启动硬件、导航或控制节点；
测试自动使用独立 ROS domain 和 localhost；CTest 将日志写入构建目录。
手动运行测试二进制时可另外设置项目内 `ROS_LOG_DIR`。

可选真实地图几何测试：设置 `GICP_TEST_PCD` 为本机现有 PCD 的绝对路径，
运行 `test_height_filter --gtest_filter=HeightFilter.ExistingMapCropKnownPose`。
它从地图裁片合成已知位姿源云，在受控外参下对比 0/2m 和 10/30 次迭代，
**不是采集得到的真实扫描，不能证明实车收敛或动态环境定位精度**。

现场具备测试条件后，建议用同一原始 PCD、同一段真实 bag、相同初值对比
`max_z=0.0` 与 `2.0`，观察成功率、位姿误差/跳变、内点比例、
耗时和达到迭代上限的比例。阈值选取应依据稳定结构、动态物体和遮挡情况；
不要仅按点数更少或“converged”判断是否更好。


本次 WSL 离线结果：合成三维场景与完整节点测试通过；两份现有地图裁片的已知位姿测试通过。
`rmuc2026.pcd` 的 2m 裁剪案例在 10 次时未收敛，30 次预算下在第 12 次收敛；
不裁剪对照在第 13 次收敛。这说明旧迭代上限可能成为瓶颈，
不能据此认定前辈的现场故障就是同一原因，也不能代替后续实车/bag 对照。
