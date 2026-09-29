"""Behavior-tree decision test bench. No Gazebo, SLAM, localization, or Nav2."""

from __future__ import annotations

from pathlib import Path
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def resolve_path(value: str, package_share: Path) -> Path:
    if value.startswith("package://"):
        package_path = value[len("package://") :]
        package_name, separator, relative_path = package_path.partition("/")
        if not separator or not package_name or not relative_path:
            raise RuntimeError(f"Invalid package URI: {value}")
        return Path(get_package_share_directory(package_name)) / relative_path
    path = Path(value).expanduser()
    return path if path.is_absolute() else package_share / path


def launch_setup(context: LaunchContext):
    package_share = Path(get_package_share_directory("spr_bt_sim"))
    config_file = Path(LaunchConfiguration("config_file").perform(context)).expanduser()
    document = yaml.safe_load(config_file.read_text(encoding="utf-8"))
    settings = document["decision_sim"]

    namespace = str(settings["namespace"]).strip("/")
    map_yaml = resolve_path(str(settings["map_yaml"]), package_share)
    rviz_config = resolve_path(str(settings["rviz_config"]), package_share)
    tree_xml = resolve_path(str(settings["decision"]["tree_xml_file"]), package_share)
    for label, path in (
        ("map_yaml", map_yaml),
        ("rviz_config", rviz_config),
        ("decision.tree_xml_file", tree_xml),
    ):
        if not path.is_file():
            raise RuntimeError(f"decision_sim.{label} does not exist: {path}")

    runtime_dir = Path(tempfile.gettempdir()) / "spr_decision_sim" / namespace
    runtime_dir.mkdir(parents=True, exist_ok=True)
    tree_model_path = runtime_dir / "tree_nodes.xml"

    result_parameters = dict(settings["result_mock"])
    referee_parameters = dict(settings["referee_mock"])
    referee_enabled = bool(referee_parameters.pop("enabled"))
    decision_parameters = dict(settings["decision"]["parameters"])
    decision_parameters.update(
        {
            "tree_xml_file": str(tree_xml),
            "tree_node_model_export_path": str(tree_model_path),
        }
    )

    tf_remaps = [("/tf", "tf"), ("/tf_static", "tf_static")]
    actions = [
        LogInfo(
            msg=(
                f"Decision-only simulation: namespace=/{namespace}, map={map_yaml}, "
                f"tree={tree_xml}, result_mode={result_parameters['result_mode']}"
            )
        ),
        Node(
            package="nav2_map_server",
            executable="map_server",
            namespace=namespace,
            name="map_server",
            output="screen",
            parameters=[{"yaml_filename": str(map_yaml), "use_sim_time": False}],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            namespace=namespace,
            name="lifecycle_manager_map",
            output="screen",
            parameters=[
                {
                    "autostart": True,
                    "use_sim_time": False,
                    "node_names": ["map_server"],
                }
            ],
        ),
        Node(
            package="spr_bt_sim",
            executable="decision_result_mock.py",
            namespace=namespace,
            name="decision_result_mock",
            output="screen",
            parameters=[result_parameters],
            remappings=tf_remaps,
        ),
    ]

    if referee_enabled:
        actions.append(
            Node(
                package="spr_referee_mock",
                executable="referee_mock_gui",
                namespace=namespace,
                name="referee_mock",
                output="screen",
                parameters=[referee_parameters],
            )
        )

    if bool(settings["use_rviz"]):
        actions.append(
            Node(
                package="rviz2",
                executable="rviz2",
                namespace=namespace,
                name="rviz2",
                output="screen",
                arguments=["-d", str(rviz_config)],
                parameters=[{"use_sim_time": False}],
                remappings=tf_remaps,
            )
        )

    if bool(settings["decision"]["enabled"]):
        actions.append(
            TimerAction(
                period=float(settings["decision"]["start_delay_sec"]),
                actions=[
                    Node(
                        package="spr_decision",
                        executable="tree_exec_node",
                        namespace=namespace,
                        name="tree_exec",
                        output="screen",
                        parameters=[decision_parameters],
                        remappings=tf_remaps,
                    )
                ],
            )
        )

    return actions


def generate_launch_description():
    package_share = Path(get_package_share_directory("spr_bt_sim"))
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=str(package_share / "config" / "simulation.yaml"),
                description="Unified decision simulation configuration",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
