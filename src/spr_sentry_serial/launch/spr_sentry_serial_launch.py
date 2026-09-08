import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory("spr_sentry_serial"),
        "config",
        "serial_params.yaml",
    )

    namespace = LaunchConfiguration("namespace")
    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace",
        default_value="red_standard_robot1",
        description="Top-level namespace",
    )

    serial_node = Node(
        package='spr_sentry_serial',
        executable='serial_node',
        name='serial_node',
        namespace=namespace,
        output='screen',
        parameters=[config_file],
    )

    return LaunchDescription(
        [declare_namespace_cmd, serial_node]
    )