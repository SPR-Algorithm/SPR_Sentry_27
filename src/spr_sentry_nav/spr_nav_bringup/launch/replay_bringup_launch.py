"""离线回放建图入口，替代 old/slam_launch_bag.py + old/slam_bag_launch.py。

只启动数据重处理和SLAM，不启动Livox、merge、Nav2、决策或录制。
包须含与YAML中 point_lio.common.lid_topic/imu_topic 一致的融合点云及IMU；
namespace须与包中话题一致。原始双雷达未融合的bag不属于这个入口的输入。
bag_dir不属于launch参数，由外部 ros2 bag play 使用。

完整多终端启动及地图保存命令已移至 script/map_replay.sh。
以下是手动启动示例（两个终端均需先source工作空间，并另行提供模型TF）：
# 终端 1：启动回放建图
ros2 launch spr_nav_bringup replay_bringup_launch.py

# 终端 2：播放数据（仅播放输入，话题列表见脚本；不要回放重复的输出TF）
ros2 bag play /path/to/bag --clock --topics <输入点云话题> <IMU话题> <关节状态话题>

use_robot_state_pub=false时，传感器/底盘/云台间所需TF应由bag或外部模型提供；
启用模型发布时需有相应joint_states输入，避免重复回放相同模型TF。
本入口会重新产生里程计等输出，不应同时回放同名处理结果或重复的TF边。
map→odom由独立TF管理器统一发布；不可同时回放bag中的该TF边。
"""

import os
import importlib
import yaml

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


def _declare_arguments():
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
    ]:
        arguments.append(DeclareLaunchArgument(
            name, default_value=default, choices=["true", "false", "True", "False"], description=description,
        ))
    return arguments


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
    with open(params_file, encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    manager = config.get("map_odom_tf_manager", {}).get("ros__parameters", {})
    for node in ("slam_toolbox", "loam_interface", "sensor_scan_generation"):
        settings = config.get(node, {}).get("ros__parameters", {})
        for frame, default in {"map_frame": "map", "odom_frame": "odom", "base_frame": "base_footprint"}.items():
            if frame in settings and settings[frame] != manager.get(frame, default):
                raise RuntimeError(f"{node}.{frame} 与 map_odom_tf_manager 不一致，请统一YAML的frame配置")
    if config.get("slam_toolbox", {}).get("ros__parameters", {}).get("transform_publish_period") != 0.0:
        raise RuntimeError("使用TF管理器时，YAML的slam_toolbox.transform_publish_period必须显式为0.0")
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
    manager_params = ParameterFile(RewrittenYaml(
        source_file=prepared_params, root_key=namespace,
        param_rewrites={}, convert_types=True,
    ), allow_substs=True)
    manager_node = Node(
        package="spr_nav_bringup", executable="map_odom_tf_manager",
        name="map_odom_tf_manager", output="screen",
        parameters=[manager_params, {"source_mode": "slam", "initial_pose": [0.0, 0.0, 0.0],
                                     "use_sim_time": enabled("use_sim_time")}],
        arguments=["--ros-args", "--log-level", value("log_level")],
    )
    nodes.extend([manager_node, RegisterEventHandler(OnProcessExit(
        target_action=manager_node,
        on_exit=lambda event, context: [] if context.is_shutdown else [
            EmitEvent(event=Shutdown(reason="TF manager exited; stopping replay"))],
    ))])
    if enabled("use_composition"):
        # 与实时入口一致，保留完整的进程级YAML，供组件内部创建的节点读取。
        container_params = ParameterFile(RewrittenYaml(
            source_file=prepared_params, root_key=namespace,
            param_rewrites={}, convert_types=True,
        ), allow_substs=True)
        nodes.append(Node(
            package="spr_nav_bringup", executable="component_container_isolated",
            name="nav2_container", output="screen",
            parameters=[container_params, {
                "use_sim_time": enabled("use_sim_time"), "autostart": enabled("autostart"),
            }],
            arguments=["--ros-args", "--log-level", value("log_level")],
        ))
    nodes.extend([
        _include(os.path.join(launch_dir, "data_processing_launch.py"), {
            **common, "mode": "slam", "prior_pcd_file": "", "use_live_lidar": "false",
            "use_composition": value("use_composition"),
            "container_name": "nav2_container",
        }),
        _include(os.path.join(launch_dir, "slam_launch.py"), {
            **common, "autostart": value("autostart"),
        }),
    ])
    actions = [GroupAction(actions=nodes)]
    # 模型子 launch 自己设置 namespace，必须放在上述命名空间组之外。
    if enabled("use_robot_state_pub"):
        try:
            importlib.import_module("sdformat_tools.urdf_generator")
            importlib.import_module("xmacro.xmacro4sdf")
        except ImportError as error:
            raise RuntimeError("模型发布依赖不完整，请安装sdformat_tools及其xmacro/transforms3d依赖并source工作空间") from error
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
    return LaunchDescription([
        *_declare_arguments(),
        SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
        SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1"),
        OpaqueFunction(function=_setup),
    ])
