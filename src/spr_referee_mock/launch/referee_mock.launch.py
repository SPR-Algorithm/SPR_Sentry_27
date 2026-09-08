import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from nav2_common.launch import RewrittenYaml
from launch_ros.descriptions import ParameterFile


def generate_launch_description():
    pkg_dir = get_package_share_directory("spr_referee_mock")

    namespace = LaunchConfiguration("namespace")
    params_file = LaunchConfiguration("params_file")
    log_level = LaunchConfiguration("log_level")

    declare_namespace = DeclareLaunchArgument(
        "namespace",
        default_value="red_standard_robot1",
        description="与 spr_decision 一致的顶层命名空间",
    )

    declare_params = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(pkg_dir, "config", "referee_mock_params.yaml"),
        description="裁判模拟节点参数文件",
    )

    declare_log_level = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="日志级别",
    )

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )

    mock_group = GroupAction(
        actions=[
            PushRosNamespace(namespace),
            Node(
                package="spr_referee_mock",
                executable="referee_mock_gui",
                name="referee_mock",
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
                output="screen",
            ),
        ]
    )

    ld = LaunchDescription()
    ld.add_action(declare_namespace)
    ld.add_action(declare_params)
    ld.add_action(declare_log_level)
    ld.add_action(mock_group)
    return ld
