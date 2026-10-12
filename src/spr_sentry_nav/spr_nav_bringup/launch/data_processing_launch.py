# Copyright 2025 Lihan Chen
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#     http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""公共数据链，不受 use_navigation 控制。

节点定义来自原 rm_navigation_reality、navigation、localization、slam launch。
Livox → merge → Point-LIO → 点云/地形处理；SLAM 时额外转换 obstacle_scan。
回放入口通过内部参数 use_live_lidar=false 跳过 Livox 和 merge，直接消费包中的融合点云。
主入口负责命名空间、TF remap、YAML 预处理和组件容器；本模块不重复创建。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode, ParameterFile, ParameterValue
from nav2_common.launch import RewrittenYaml


def _setup(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    def enabled(name):
        return value(name).lower() == "true"

    namespace = value("namespace")
    use_live_lidar = enabled("use_live_lidar")
    driver_bag = use_live_lidar and value("livox_output_mode") == "bag"
    configured_params = ParameterFile(RewrittenYaml(
        source_file=value("params_file"), root_key=namespace,
        param_rewrites={"use_sim_time": value("use_sim_time")}, convert_types=True,
    ), allow_substs=True)
    options = dict(
        output="screen", respawn=enabled("use_respawn"), respawn_delay=2.0,
        arguments=["--ros-args", "--log-level", value("log_level")],
    )
    actions = []
    if use_live_lidar:
        actions.append(Node(
            package="livox_ros_driver2", executable="livox_ros_driver2_node", name="livox_ros_driver2",
            parameters=[configured_params, {
                "output_data_type": 1 if driver_bag else 0,
                "rosbag_path": ParameterValue(value("rosbag_path"), value_type=str),
                "enable_lidar_bag": enabled("enable_lidar_bag"),
                "enable_imu_bag": enabled("enable_imu_bag"),
            }], **options,
        ))
    # 驱动内部录制不发布 ROS 数据，不启动等待点云/IMU的后续节点。
    if driver_bag:
        return actions

    if use_live_lidar:
        actions.append(Node(
            package="dual_mid360_merge", executable="merge_cloud_node", name="merge_cloud_node",
            parameters=[configured_params], **options,
        ))
    # 对应旧 localization/slam 的 Point-LIO 参数差异，全程只启动一份。
    if value("mode") == "slam":
        lio_overrides = {"prior_pcd.enable": False, "pcd_save.pcd_save_en": True}
    else:
        lio_overrides = {
            "prior_pcd.prior_pcd_map_path": ParameterValue(value("prior_pcd_file"), value_type=str),
        }
    actions.append(Node(
        package="point_lio", executable="pointlio_mapping", name="point_lio",
        parameters=[configured_params, lio_overrides], **options,
    ))

    # sensor_scan_generation 实际发布 odom→base_footprint，不能随导航关闭。
    components = []
    for package, executable, plugin in [
        ("loam_interface", "loam_interface_node", "loam_interface::LoamInterfaceNode"),
        ("sensor_scan_generation", "sensor_scan_generation_node", "sensor_scan_generation::SensorScanGenerationNode"),
    ]:
        if enabled("use_composition"):
            components.append(ComposableNode(
                package=package, plugin=plugin, name=package, parameters=[configured_params],
            ))
        else:
            actions.append(Node(
                package=package, executable=executable, name=package,
                parameters=[configured_params], **options,
            ))
    if components:
        actions.append(LoadComposableNodes(
            target_container="/" + "/".join(filter(None, [namespace, value("container_name")])),
            composable_node_descriptions=components,
        ))
    for package, executable in [
        ("terrain_analysis", "terrainAnalysis"), ("terrain_analysis_ext", "terrainAnalysisExt"),
    ]:
        actions.append(Node(
            package=package, executable=executable, name=package,
            parameters=[configured_params], **options,
        ))
    if value("mode") == "slam":
        actions.append(Node(
            package="pointcloud_to_laserscan", executable="pointcloud_to_laserscan_node",
            name="pointcloud_to_laserscan", parameters=[configured_params],
            remappings=[("cloud_in", "terrain_map_ext"), ("scan", "obstacle_scan")],
            **options,
        ))
    return actions


def generate_launch_description():
    # 内部参数，由实时/回放主入口传入；params_file 已完成占位替换。
    arguments = [
        DeclareLaunchArgument("params_file", description="主入口处理后的参数YAML路径"),
        DeclareLaunchArgument("namespace", default_value="", description="YAML命名空间；节点命名空间由父launch设置"),
        DeclareLaunchArgument("mode", default_value="localization", choices=["slam", "localization"], description="决定Point-LIO配置及是否转换LaserScan"),
        DeclareLaunchArgument("prior_pcd_file", default_value="", description="定位时Point-LIO的先验PCD路径，是否加载仍由YAML决定"),
        DeclareLaunchArgument("container_name", default_value="nav2_container", description="主入口已创建的组件容器"),
        DeclareLaunchArgument("log_level", default_value="info", description="日志级别"),
        DeclareLaunchArgument("livox_output_mode", default_value="ros", choices=["ros", "bag"], description="发布ROS话题 / 仅驱动内部录制"),
        DeclareLaunchArgument("rosbag_path", default_value="livox_rosbag2", description="Livox内部录制目录"),
    ]
    for name, default, description in [
        ("use_live_lidar", "true", "内部开关：实时启动Livox与merge；回放入口固定为false"),
        ("use_sim_time", "false", "使用/clock"),
        ("use_composition", "true", "将可组件化的数据处理节点加载进共享容器"),
        ("use_respawn", "false", "独立节点异常退出后重启"),
        ("enable_lidar_bag", "true", "Livox内部录制点云"),
        ("enable_imu_bag", "true", "Livox内部录制IMU"),
    ]:
        arguments.append(DeclareLaunchArgument(
            name, default_value=default, choices=["true", "false", "True", "False"],
            description=description,
        ))
    return LaunchDescription([*arguments, OpaqueFunction(function=_setup)])
