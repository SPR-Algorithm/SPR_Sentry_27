"""实时主入口：SLAM / 已有地图定位，导航与决策可独立关闭。

旧入口归档在 launch/old/，不随新版安装。本文件的参数对应关系：
  slam:=True -> mode:=slam（手动建图再加 use_navigation:=false）
  slam:=False use_gicp:=True/False -> mode:=localization localization_mode:=gicp/static
  world/map/prior_pcd_file -> map_profile（同名 YAML + PCD，不再独立选择）
  localization_launch_file/slam_launch_file -> 内部按模式选择，不对外暴露
  container_name -> 内部固定 nav2_container；robot_name 仅选择机器人模型
子模块与本入口同处 launch/，新子模块不调用旧 bringup 子 launch。
主入口只管理参数、命名空间、组件容器、RViz和模块选择。
回放、串口启动及地图保存命令不在这个实时入口的范围内。
"""

import math
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
    mode = value("mode")
    driver_bag = value("livox_output_mode") == "bag"
    params_file = _require_file(value("params_file"), "params_file")

    if driver_bag:
        if not (enabled("enable_lidar_bag") or enabled("enable_imu_bag")):
            raise RuntimeError("livox_output_mode=bag 至少需要启用一种 enable_*_bag")
        if any(enabled(k) for k in ("use_navigation", "use_decision", "use_rosbag_record")):
            raise RuntimeError(
                "Livox 内部 bag 模式不发布点云/IMU；请设置 use_navigation=false、"
                "use_decision=false、use_rosbag_record=false，或使用 ros 模式外部录制"
            )

    map_yaml, pcd = "", ""
    if not driver_bag and mode == "localization":
        profile = value("map_profile")
        if not profile or os.path.basename(profile) != profile or profile in (".", ".."):
            raise RuntimeError("定位模式须指定 map_profile（地图名称，不含目录）")
        map_yaml = _require_file(
            os.path.join(share, "map", "reality", profile + ".yaml"), "map_profile",
        )
        pcd = os.path.join(share, "pcd", "reality", profile + ".pcd")
        if value("localization_mode") == "gicp":
            _require_file(pcd, "map_profile (GICP PCD)")
        for name in ("map_to_odom_x", "map_to_odom_y", "map_to_odom_yaw"):
            if not math.isfinite(float(value(name))):
                raise RuntimeError(f"{name} 必须为有限数值；yaw 单位为弧度")

    # YAML 保留话题/插件/硬件/算法配置；Launch 仅生成运行时覆盖文件。
    substituted = ReplaceString(
        source_file=params_file,
        replacements={"<robot_namespace>": "/" + namespace if namespace else ""},
    )
    prepared_params = _ClockYaml(
        source_file=substituted, root_key="",
        param_rewrites={"use_sim_time": value("use_sim_time")}, convert_types=True,
    ).perform(context)
    # 可选决策文件先校验，避免错误配置直到硬件启动后才被发现。
    decision_file = ""
    if enabled("use_decision"):
        decision_share = get_package_share_directory("spr_decision")
        decision_file = _require_file(os.path.join(
            decision_share, "config", value("params_file_name"),
        ), "params_file_name")

    common = {name: value(name) for name in ("use_sim_time", "use_respawn", "log_level")}
    common.update(namespace=namespace, params_file=prepared_params)
    component_args = {
        **common, "use_composition": value("use_composition"), "container_name": "nav2_container",
    }
    nodes = [
        PushRosNamespace(namespace), SetRemap("/tf", "tf"), SetRemap("/tf_static", "tf_static"),
    ]
    # 只创建一个容器，数据处理、定位和导航模块共用；驱动内部录制不需要容器。
    if not driver_bag and enabled("use_composition"):
        # 必须把完整、带命名空间的YAML交给容器进程。组件加载请求只提取组件
        # 自身参数，controller/planner内部创建的costmap须从进程参数中读取配置。
        container_params = ParameterFile(RewrittenYaml(
            source_file=prepared_params, root_key=namespace,
            param_rewrites={"yaml_filename": map_yaml}, convert_types=True,
        ), allow_substs=True)
        nodes.append(Node(
            package="rclcpp_components", executable="component_container_isolated",
            name="nav2_container", output="screen",
            parameters=[container_params, {
                "use_sim_time": enabled("use_sim_time"), "autostart": enabled("autostart"),
            }],
            arguments=["--ros-args", "--log-level", value("log_level")],
        ))
    nodes.append(_include(os.path.join(launch_dir, "data_processing_launch.py"), {
        **component_args, "mode": mode, "prior_pcd_file": pcd, "use_live_lidar": "true",
        **{name: value(name) for name in (
            "livox_output_mode", "rosbag_path", "enable_lidar_bag", "enable_imu_bag",
        )},
    }))
    if not driver_bag:
        if mode == "slam":
            nodes.append(_include(os.path.join(launch_dir, "slam_launch.py"), {
                **common, "autostart": value("autostart"),
                "publish_static_map_to_odom": value("publish_static_map_to_odom"),
            }))
        else:
            nodes.append(_include(os.path.join(launch_dir, "localization_launch.py"), {
                **component_args, "autostart": value("autostart"),
                "map": map_yaml, "prior_pcd_file": pcd,
                "localization_mode": value("localization_mode"),
                **{name: value(name) for name in ("map_to_odom_x", "map_to_odom_y", "map_to_odom_yaw")},
            }))
        if enabled("use_navigation"):
            nodes.append(_include(os.path.join(launch_dir, "navigation_launch.py"), {
                **component_args, "autostart": value("autostart"),
            }))
    if enabled("use_decision"):
        nodes.append(_include(os.path.join(launch_dir, "decision_launch.py"), {
            **common, "params_file": decision_file, "use_referee_mock": value("use_referee_mock"),
        }))

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
    if enabled("use_rosbag_record"):
        actions.append(_include(os.path.join(launch_dir, "recording_launch.py"), {
            "namespace": namespace, "params_file": prepared_params,
        }))
    return actions


def generate_launch_description():
    share = get_package_share_directory("spr_nav_bringup")
    # 可选参数的默认值均在此声明；仅 localization 模式要求填写 map_profile。
    arguments = [
        DeclareLaunchArgument("mode", default_value="localization", choices=["slam", "localization"], description="slam=建图；localization=已有地图定位"),
        DeclareLaunchArgument("localization_mode", default_value="gicp", choices=["gicp", "static"], description="仅定位模式：GICP匹配PCD / 固定map→odom"),
        DeclareLaunchArgument("map_profile", default_value="", description="定位模式必填：map/reality与pcd/reality内同名地图，不带扩展名；SLAM忽略"),
        DeclareLaunchArgument("namespace", default_value="red_standard_robot1", description="命名空间；同时替换YAML中的<robot_namespace>"),
        DeclareLaunchArgument("robot_name", default_value="spr2025_sentry_robot", description="机器人模型名，不带.sdf.xmacro；仅启用模型发布时有效"),
        DeclareLaunchArgument("params_file", default_value=os.path.join(share, "config", "reality", "nav2_params.yaml"), description="主参数YAML完整路径；硬件、话题、算法调参保留其中"),
        DeclareLaunchArgument("log_level", default_value="info", choices=["debug", "info", "warn", "error", "fatal"], description="日志级别"),
        DeclareLaunchArgument("rviz_config_file", default_value=os.path.join(share, "rviz", "nav2_default_view.rviz"), description="RViz配置路径；仅use_rviz=true时读取"),
        DeclareLaunchArgument("map_to_odom_x", default_value="0.0", description="定位初始X，米；GICP初值或静态TF"),
        DeclareLaunchArgument("map_to_odom_y", default_value="0.0", description="定位初始Y，米；GICP初值或静态TF"),
        DeclareLaunchArgument("map_to_odom_yaw", default_value="0.0", description="定位初始偏航角，弧度；SLAM忽略初值"),
        DeclareLaunchArgument("livox_output_mode", default_value="ros", choices=["ros", "bag"], description="ros=发布话题；bag=仅驱动内部录制，须关闭导航/决策/外部录制"),
        DeclareLaunchArgument("rosbag_path", default_value="livox_rosbag2", description="仅Livox内部bag输出目录，目录须不存在；不是外部录制路径"),
        DeclareLaunchArgument("params_file_name", default_value="node_params.yaml", description="决策config下文件名，包含.yaml，不自动补后缀；启用决策时须选择存在的文件，如node_params1.yaml"),
        # 当前驱动未实现LVX读取；这是回放输入路径，不是录制开关，暂不启用：
        # DeclareLaunchArgument("lvx_file_path", default_value="", description="LVX输入文件路径"),
    ]
    for name, default, description in [
        ("use_navigation", "true", "启动Nav2规划控制；false仍保留定位/建图所需处理节点"),
        ("use_sim_time", "false", "使用/clock；仅true/false，无auto"),
        ("autostart", "true", "自动配置并激活生命周期节点"),
        ("use_composition", "true", "支持组件化的节点放入同一容器"),
        ("use_respawn", "false", "重启异常退出的独立节点；容器内组件不单独重启"),
        ("use_robot_state_pub", "false", "启动机器人模型发布；外部已启动时保持false"),
        ("use_rviz", "true", "启动RViz"),
        ("publish_static_map_to_odom", "true", "仅SLAM：发布单位map→odom静态TF，保持原默认；定位分支由localization_mode决定"),
        ("enable_lidar_bag", "true", "仅livox_output_mode=bag：记录点云"),
        ("enable_imu_bag", "true", "仅livox_output_mode=bag：记录IMU"),
        ("use_rosbag_record", "false", "独立ros2 bag record，可在正常发布话题时录制"),
        # 独立录制命令（先保留在这里，后续移到sh；namespace按实际修改）：
        # ros2 bag record /red_standard_robot1/livox/lidar_merged /red_standard_robot1/livox/imu_192_168_1_194 /red_standard_robot1/tf /red_standard_robot1/tf_static /red_standard_robot1/serial/gimbal_joint_state
        ("use_decision", "false", "启动spr_decision；否则忽略其配置和mock参数"),
        ("use_referee_mock", "false", "仅use_decision=true：同时启动裁判系统mock"),
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
