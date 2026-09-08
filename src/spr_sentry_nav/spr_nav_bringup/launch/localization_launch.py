# Copyright 2025 Lihan Chen
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml


def _parse_bool(value: str) -> bool:
    return value.lower() in ("true", "1", "yes")


def _localization_setup(context, *args, **kwargs):
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    prior_pcd_file = LaunchConfiguration("prior_pcd_file")
    use_composition = LaunchConfiguration("use_composition")
    container_name = LaunchConfiguration("container_name")
    container_name_full = (namespace, "/", container_name)
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")

    use_gicp = _parse_bool(LaunchConfiguration("use_gicp").perform(context))
    map_to_odom_x = float(LaunchConfiguration("map_to_odom_x").perform(context))
    map_to_odom_y = float(LaunchConfiguration("map_to_odom_y").perform(context))
    map_to_odom_yaw = float(LaunchConfiguration("map_to_odom_yaw").perform(context))
    # GICP init_pose: [x, y, z, roll, pitch, yaw] == map->odom
    init_pose = [map_to_odom_x, map_to_odom_y, 0.0, 0.0, 0.0, map_to_odom_yaw]

    configured_params = kwargs["configured_params"]
    lifecycle_nodes = kwargs["lifecycle_nodes"]
    use_composition_enabled = _parse_bool(use_composition.perform(context))
    prior_pcd_path = prior_pcd_file.perform(context)

    actions = []

    # use_gicp=False: static map->odom (LIO-style). use_gicp=True: GICP only (no static TF).
    if not use_gicp:
        actions.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="static_transform_publisher_map2odom",
                output="screen",
                arguments=[
                    "--x",
                    str(map_to_odom_x),
                    "--y",
                    str(map_to_odom_y),
                    "--z",
                    "0.0",
                    "--roll",
                    "0.0",
                    "--pitch",
                    "0.0",
                    "--yaw",
                    str(map_to_odom_yaw),
                    "--frame-id",
                    "map",
                    "--child-frame-id",
                    "odom",
                ],
            )
        )

    if use_composition_enabled:
        composable_nodes = [
            ComposableNode(
                package="nav2_map_server",
                plugin="nav2_map_server::MapServer",
                name="map_server",
                parameters=[configured_params],
            ),
            ComposableNode(
                package="nav2_lifecycle_manager",
                plugin="nav2_lifecycle_manager::LifecycleManager",
                name="lifecycle_manager_localization",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "autostart": autostart,
                        "node_names": lifecycle_nodes,
                    }
                ],
            ),
        ]
        if use_gicp:
            composable_nodes.insert(
                1,
                ComposableNode(
                    package="small_gicp_relocalization",
                    plugin="small_gicp_relocalization::SmallGicpRelocalizationNode",
                    name="small_gicp_relocalization",
                    parameters=[
                        configured_params,
                        {"prior_pcd_file": prior_pcd_path},
                        {"init_pose": init_pose},
                    ],
                ),
            )
        actions.append(
            LoadComposableNodes(
                target_container=container_name_full,
                composable_node_descriptions=composable_nodes,
            )
        )
    else:
        load_nodes_actions = [
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_localization",
                output="screen",
                arguments=["--ros-args", "--log-level", log_level],
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"autostart": autostart},
                    {"node_names": lifecycle_nodes},
                ],
            ),
        ]
        if use_gicp:
            load_nodes_actions.insert(
                1,
                Node(
                    package="small_gicp_relocalization",
                    executable="small_gicp_relocalization_node",
                    name="small_gicp_relocalization",
                    output="screen",
                    respawn=use_respawn,
                    respawn_delay=2.0,
                    parameters=[
                        configured_params,
                        {"prior_pcd_file": prior_pcd_path},
                        {"init_pose": init_pose},
                    ],
                    arguments=["--ros-args", "--log-level", log_level],
                ),
            )
        actions.append(GroupAction(actions=load_nodes_actions))

    return actions


def generate_launch_description():
    bringup_dir = get_package_share_directory("spr_nav_bringup")

    namespace = LaunchConfiguration("namespace")
    map_yaml_file = LaunchConfiguration("map")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    prior_pcd_file = LaunchConfiguration("prior_pcd_file")
    params_file = LaunchConfiguration("params_file")
    use_composition = LaunchConfiguration("use_composition")
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")

    lifecycle_nodes = ["map_server"]

    param_substitutions = {"use_sim_time": use_sim_time, "yaml_filename": map_yaml_file}

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True,
        ),
        allow_substs=True,
    )

    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1"
    )

    colorized_output_envvar = SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1")

    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace", default_value="", description="Top-level namespace"
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        "map", description="Full path to map yaml file to load"
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_prior_pcd_file_cmd = DeclareLaunchArgument(
        "prior_pcd_file",
        default_value="",
        description="Full path to prior PCD file to load",
    )

    declare_use_gicp_cmd = DeclareLaunchArgument(
        "use_gicp",
        default_value="True",
        description=(
            "True: GICP publishes map->odom; init_pose from map_to_odom_*; "
            "RViz 2D Pose Estimate on /initialpose also works. "
            "False: static map->odom TF only (no GICP)."
        ),
    )

    declare_map_to_odom_x_cmd = DeclareLaunchArgument(
        "map_to_odom_x",
        default_value="0.0",
        description="Initial map->odom x (m); GICP init_pose[0] when use_gicp is True",
    )

    declare_map_to_odom_y_cmd = DeclareLaunchArgument(
        "map_to_odom_y",
        default_value="0.0",
        description="Initial map->odom y (m); GICP init_pose[1] when use_gicp is True",
    )

    declare_map_to_odom_yaw_cmd = DeclareLaunchArgument(
        "map_to_odom_yaw",
        default_value="0.0",
        description="Initial map->odom yaw (rad); GICP init_pose[5] when use_gicp is True",
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(bringup_dir, "params", "nav2_params.yaml"),
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically startup the nav2 stack",
    )

    declare_use_composition_cmd = DeclareLaunchArgument(
        "use_composition",
        default_value="False",
        description="Use composed bringup if True",
    )

    declare_container_name_cmd = DeclareLaunchArgument(
        "container_name",
        default_value="nav2_container",
        description="the name of container that nodes will load in if use composition",
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        "use_respawn",
        default_value="False",
        description="Whether to respawn if a node crashes. Applied when composition is disabled.",
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        "log_level", default_value="info", description="log level"
    )

    start_point_lio_node = Node(
        package="point_lio",
        executable="pointlio_mapping",
        name="point_lio",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[
            configured_params,
            {"prior_pcd.prior_pcd_map_path": prior_pcd_file},
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    localization_setup = OpaqueFunction(
        function=_localization_setup,
        kwargs={
            "configured_params": configured_params,
            "lifecycle_nodes": lifecycle_nodes,
        },
    )

    ld = LaunchDescription()

    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(colorized_output_envvar)

    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_prior_pcd_file_cmd)
    ld.add_action(declare_use_gicp_cmd)
    ld.add_action(declare_map_to_odom_x_cmd)
    ld.add_action(declare_map_to_odom_y_cmd)
    ld.add_action(declare_map_to_odom_yaw_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_container_name_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)

    ld.add_action(start_point_lio_node)
    ld.add_action(localization_setup)

    return ld
