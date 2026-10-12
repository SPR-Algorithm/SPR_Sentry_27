"""决策子模块，节点定义沿用 spr_decision/launch/run.launch.py。

主入口将 params_file_name（含.yaml）解析为完整路径传入；行为树仍由YAML选择。
命名空间和TF remap由主入口设置，不在此重复PushRosNamespace。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def _setup(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    namespace = value("namespace")
    sim_time = value("use_sim_time").lower() == "true"
    options = dict(
        output="screen", respawn=value("use_respawn").lower() == "true", respawn_delay=2.0,
        arguments=["--ros-args", "--log-level", value("log_level")],
    )
    configured_params = ParameterFile(RewrittenYaml(
        source_file=value("params_file"), root_key=namespace, param_rewrites={}, convert_types=True,
    ), allow_substs=True)
    actions = [Node(
        package="spr_decision", executable="tree_exec_node", name="tree_exec",
        parameters=[configured_params, {"use_sim_time": sim_time}], **options,
    )]
    if value("use_referee_mock").lower() == "true":
        mock_share = get_package_share_directory("spr_referee_mock")
        referee_mock_params = ParameterFile(RewrittenYaml(
            source_file=os.path.join(mock_share, "config", "referee_mock_params.yaml"),
            root_key=namespace, param_rewrites={}, convert_types=True,
        ), allow_substs=True)
        actions.append(Node(
            package="spr_referee_mock", executable="referee_mock_gui", name="referee_mock",
            parameters=[referee_mock_params, {"use_sim_time": sim_time}], **options,
        ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("params_file", description="主入口解析出的决策YAML完整路径"),
        DeclareLaunchArgument("namespace", default_value="", description="YAML命名空间，继承父launch的节点命名空间"),
        DeclareLaunchArgument("use_sim_time", default_value="false", description="使用/clock"),
        DeclareLaunchArgument("use_respawn", default_value="false", description="节点异常退出后重启"),
        DeclareLaunchArgument("log_level", default_value="info", description="日志级别"),
        DeclareLaunchArgument("use_referee_mock", default_value="false", description="同时启动裁判系统mock"),
        OpaqueFunction(function=_setup),
    ])
