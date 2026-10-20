"""离线回放建图入口，替代 old/slam_launch_bag.py + old/slam_bag_launch.py。

只启动数据重处理和SLAM，不启动Livox、merge、Nav2、决策或录制。
包须含与YAML中 point_lio.common.lid_topic/imu_topic 一致的融合点云及IMU；
namespace须与包中话题一致。原始双雷达未融合的bag不属于这个入口的输入。
bag_dir不属于launch参数，由外部 ros2 bag play 使用。

启动命令（暂存于此，后期移植到sh；两个终端均需先source工作空间）：
# 终端 1：启动回放建图
ros2 launch spr_nav_bringup replay_bringup_launch.py

# 终端 2：播放数据（将 /path/to/bag 替换为实际bag路径）
ros2 bag play /path/to/bag --clock

use_robot_state_pub=false时，传感器/底盘/云台间所需TF应由bag或外部模型提供；
启用模型发布时需有相应joint_states输入，避免重复回放相同模型TF。
本入口会重新产生里程计等输出，不应同时回放同名处理结果或重复的TF边。
静态map→odom默认值和SLAM的transform_publish_period保持原设计，暂不重设计TF策略。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, GroupAction, IncludeLaunchDescription,
    OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import ReplaceString, RewrittenYaml


class _ClockYaml(RewrittenYaml):
    """仅改运行时临时文件；旧 YAML 未声明时也补齐时钟参数。"""

    def substitute_params(self, data, rewrites):
        super().substitute_params(data, rewrites)

        def insert_clock(mapping):
            for key, child in mapping.items():
                if not isinstance(child, dict):
                    continue
                if key == "ros__parameters":
                    child["use_sim_time"] = rewrites["use_sim_time"].lower() == "true"
                else:
                    insert_clock(child)

        insert_clock(data)


def _include(path, arguments):
    # 子 launch 参数单独作用域，避免 params_file / namespace 相互污染。
    return GroupAction(actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path), launch_arguments=arguments.items(),
    )])


def _require_file(path, parameter):
    if not os.path.isfile(path):
        raise RuntimeError(f"{parameter}: 文件不存在: {path}")
    return path


def _setup(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    def enabled(name):
        return value(name).lower() == "true"

    share = get_package_share_directory("spr_nav_bringup")
    launch_dir = os.path.join(share, "launch")
    namespace = value("namespace").strip("/")
    params_file = _require_file(value("params_file"), "params_file")
    # 与实时入口一致：源YAML不修改，补齐全部节点的时钟参数。
    substituted = ReplaceString(
        source_file=params_file,
        replacements={"<robot_namespace>": "/" + namespace if namespace else ""},
    )
    prepared_params = _ClockYaml(
        source_file=substituted, root_key="",
        param_rewrites={"use_sim_time": value("use_sim_time")}, convert_types=True,
    ).perform(context)
    common = {name: value(name) for name in ("use_sim_time", "use_respawn", "log_level")}
    common.update(namespace=namespace, params_file=prepared_params)
    nodes = [
        PushRosNamespace(namespace), SetRemap("/tf", "tf"), SetRemap("/tf_static", "tf_static"),
    ]
    if enabled("use_composition"):
        # 与实时入口一致，保留完整的进程级YAML，供组件内部创建的节点读取。
        container_params = ParameterFile(RewrittenYaml(
            source_file=prepared_params, root_key=namespace,
            param_rewrites={}, convert_types=True,
        ), allow_substs=True)
        nodes.append(Node(
            package="rclcpp_components", executable="component_container_isolated",
            name="nav2_container", output="screen",
            parameters=[container_params, {
                "use_sim_time": enabled("use_sim_time"), "autostart": enabled("autostart"),
            }],
            arguments=["--ros-args", "--log-level", value("log_level")],
        ))
    nodes.extend([
        _include(os.path.join(launch_dir, "data_processing_launch.py"), {
            **common, "mode": "slam", "prior_pcd_file": "", "use_live_lidar": "false",
            "livox_output_mode": "ros", "use_composition": value("use_composition"),
            "container_name": "nav2_container",
        }),
        _include(os.path.join(launch_dir, "slam_launch.py"), {
            **common, "autostart": value("autostart"),
            "publish_static_map_to_odom": value("publish_static_map_to_odom"),
        }),
    ])
    actions = [GroupAction(actions=nodes)]
    # 模型子 launch 自己设置 namespace，必须放在上述命名空间组之外。
    if enabled("use_robot_state_pub"):
        robot_share = get_package_share_directory("spr_robot_description")
        robot_file = _require_file(os.path.join(
            robot_share, "resource", "xmacro", value("robot_name") + ".sdf.xmacro",
        ), "robot_name")
        actions.append(_include(os.path.join(robot_share, "launch", "robot_description_launch.py"), {
            "namespace": namespace, "robot_name": value("robot_name"), "robot_xmacro_file": robot_file,
            "params_file": os.path.join(robot_share, "params", "robot_description.yaml"),
            "use_sim_time": value("use_sim_time"), "use_rviz": "false",
            "use_respawn": value("use_respawn"), "log_level": value("log_level"),
        }))
    if enabled("use_rviz"):
        _require_file(value("rviz_config_file"), "rviz_config_file")
        rviz = Node(
            package="rviz2", executable="rviz2", namespace=namespace, output="screen",
            parameters=[{"use_sim_time": enabled("use_sim_time")}],
            arguments=["-d", value("rviz_config_file"), "--ros-args", "--log-level", value("log_level")],
            remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        )
        # 与旧入口一致：关闭RViz同时结束本次启动。
        actions.extend([rviz, RegisterEventHandler(OnProcessExit(
            target_action=rviz, on_exit=EmitEvent(event=Shutdown(reason="rviz exited")),
        ))])
    return actions


def generate_launch_description():
    share = get_package_share_directory("spr_nav_bringup")
    arguments = [
        DeclareLaunchArgument("namespace", default_value="red_standard_robot1", description="必须与bag中的话题命名空间一致；替换YAML中的<robot_namespace>"),
        DeclareLaunchArgument("robot_name", default_value="spr2025_sentry_robot", description="机器人模型名；仅use_robot_state_pub=true时读取"),
        DeclareLaunchArgument("params_file", default_value=os.path.join(share, "config", "reality", "nav2_params.yaml"), description="回放建图参数YAML；输入点云/IMU话题由其中point_lio.common指定"),
        DeclareLaunchArgument("log_level", default_value="info", choices=["debug", "info", "warn", "error", "fatal"], description="日志级别"),
        DeclareLaunchArgument("rviz_config_file", default_value=os.path.join(share, "rviz", "nav2_default_view.rviz"), description="RViz配置路径；仅use_rviz=true时读取"),
    ]
    for name, default, description in [
        ("use_sim_time", "true", "默认使用bag的/clock；播放时加--clock，无auto"),
        ("autostart", "true", "自动激活地图保存服务的生命周期"),
        ("use_composition", "true", "将支持组件化的数据处理节点放入共享容器"),
        ("use_respawn", "false", "重启异常退出的独立节点"),
        ("use_robot_state_pub", "false", "发布模型TF；bag/外部已有相同TF时保持false"),
        ("use_rviz", "true", "启动RViz；关闭RViz会结束本次launch"),
        ("publish_static_map_to_odom", "true", "发布单位map→odom静态TF，保持旧默认；有其他发布者时需评估关闭"),
    ]:
        arguments.append(DeclareLaunchArgument(
            name, default_value=default, choices=["true", "false", "True", "False"], description=description,
        ))
    return LaunchDescription([
        *arguments,
        SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
        SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1"),
        OpaqueFunction(function=_setup),
    ])
