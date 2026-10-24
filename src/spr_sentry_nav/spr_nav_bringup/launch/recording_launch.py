"""统一话题录制，由主入口 use_rosbag_record 控制；不启动回放。"""

import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def _declare_arguments():
    return [
        DeclareLaunchArgument("params_file", description="主入口处理后的参数YAML路径"),
        DeclareLaunchArgument("namespace", default_value="", description="相对话题所属命名空间"),
        DeclareLaunchArgument("output_dir", description="主入口生成的本次bag完整目录，必须尚不存在；不是根目录"),
    ]


def _setup(context):
    namespace = LaunchConfiguration("namespace").perform(context).strip("/")
    params_file = LaunchConfiguration("params_file").perform(context)
    output_dir = LaunchConfiguration("output_dir").perform(context)
    if not output_dir.strip() or os.path.lexists(output_dir):
        raise RuntimeError("output_dir 必须是尚不存在的非空bag目录")
    # 话题仍取自YAML，不额外增加主入口的话题参数。
    with open(params_file, encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    topics = config.get("rosbag_record", {}).get("ros__parameters", {}).get("topics")
    if topics is None:
        robot_share = get_package_share_directory("spr_robot_description")
        with open(os.path.join(robot_share, "params", "robot_description.yaml"), encoding="utf-8") as stream:
            robot_config = yaml.safe_load(stream)
        lio_topics = config["point_lio"]["ros__parameters"]["common"]
        topics = [
            lio_topics["lid_topic"], lio_topics["imu_topic"], "tf", "tf_static",
            *robot_config["joint_state_publisher"]["ros__parameters"]["source_list"],
        ]
    if not isinstance(topics, list) or not topics or not all(isinstance(t, str) and t for t in topics):
        raise RuntimeError("rosbag_record.ros__parameters.topics 必须是非空话题列表")
    topics = [t if t.startswith("/") else "/" + "/".join(filter(None, [namespace, t])) for t in topics]
    # 明确使用sqlite3，数据文件为.db3；Ctrl+C正常退出时完成metadata.yaml收尾。
    return [ExecuteProcess(
        cmd=["ros2", "bag", "record", "--storage", "sqlite3", "--output", output_dir, *topics],
        output="screen",
    )]


def generate_launch_description():
    return LaunchDescription([*_declare_arguments(), OpaqueFunction(function=_setup)])
