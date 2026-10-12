"""外部话题录制，由主入口 use_rosbag_record 控制；不负责Livox内部录制或回放。"""

import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def _setup(context):
    namespace = LaunchConfiguration("namespace").perform(context).strip("/")
    params_file = LaunchConfiguration("params_file").perform(context)
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
    # 外部录制目录由rosbag2自动命名，不使用驱动内部的rosbag_path。
    return [ExecuteProcess(cmd=["ros2", "bag", "record", *topics], output="screen")]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("params_file", description="主入口处理后的参数YAML路径"),
        DeclareLaunchArgument("namespace", default_value="", description="相对话题所属命名空间"),
        OpaqueFunction(function=_setup),
    ])
