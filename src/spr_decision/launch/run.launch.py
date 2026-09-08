import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    bringup_dir = get_package_share_directory("spr_decision")
    referee_mock_dir = get_package_share_directory("spr_referee_mock")

    namespace = LaunchConfiguration("namespace")
    param_file_name = LaunchConfiguration("params_file_name")
    log_level = LaunchConfiguration("log_level")
    use_referee_mock = LaunchConfiguration("use_referee_mock")




    declare_params_file_name_cmd = DeclareLaunchArgument(
        "params_file_name",
        default_value="node_params",  # 注意这里不加.yaml
        description="Filename (without .yaml) under config/",
    )

    param_file_path = os.path.join(bringup_dir, "config")
    full_param_path = [
        TextSubstitution(text=param_file_path + "/"),
        LaunchConfiguration("params_file_name"),
        TextSubstitution(text=".yaml"),
    ]

    param_substitutions = {}


    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=full_param_path,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True,
        ),
        allow_substs=True,
    )

    # 声明参数
    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace",
        default_value="red_standard_robot1",
        description="Top-level namespace",
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Logging level",
    )

    declare_use_referee_mock_cmd = DeclareLaunchArgument(
        "use_referee_mock",
        default_value="false",
        description="为 true 时启动 spr_referee_mock，用模拟 game_state 替代 spr_sentry_serial",
    )

    referee_mock_params = ParameterFile(
        RewrittenYaml(
            source_file=os.path.join(
                referee_mock_dir, "config", "referee_mock_params.yaml"
            ),
            root_key=namespace,
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )

    referee_mock_group = GroupAction(
        condition=IfCondition(use_referee_mock),
        actions=[
            PushRosNamespace(namespace),
            Node(
                package="spr_referee_mock",
                executable="referee_mock_gui",
                name="referee_mock",
                parameters=[referee_mock_params],
                arguments=["--ros-args", "--log-level", log_level],
                output="screen",
            ),
        ],
    )

    bringup_cmd_group = GroupAction(
        actions=[
            PushRosNamespace(namespace),
            SetRemap("/tf", "tf"),
            SetRemap("/tf_static", "tf_static"),
            Node(
                package='spr_decision',
                executable='tree_exec_node',
                name='tree_exec',
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                output='screen',
            )
        ]
    )

    ld = LaunchDescription()
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_params_file_name_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_use_referee_mock_cmd)
    ld.add_action(referee_mock_group)
    ld.add_action(bringup_cmd_group)

    return ld
