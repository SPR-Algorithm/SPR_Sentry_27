"""无硬件启动图测试：实际展开ROS launch，拦截进程和组件加载，不启动ROS节点。

运行：source /opt/ros/humble/setup.bash
      python3 -m pytest src/spr_sentry_nav/spr_nav_bringup/test/test_system_launch.py
"""

import importlib.util
from pathlib import Path
from unittest.mock import patch

import pytest
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.utilities import (
    normalize_to_list_of_substitutions, perform_substitutions,
    visit_all_entities_and_collect_futures,
)
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.utilities import evaluate_parameters, to_parameters_list


BRINGUP = Path(__file__).resolve().parents[1]
SRC = BRINGUP.parents[1]
PACKAGES = {
    "spr_nav_bringup": BRINGUP,
    "spr_decision": SRC / "spr_decision",
    "spr_referee_mock": SRC / "spr_referee_mock",
    "spr_robot_description": SRC / "spr_robot_description",
}


def source_share(name):
    return str(PACKAGES[name]) if name in PACKAGES else get_package_share_directory(name)


def load_main(entry="system_bringup_launch.py"):
    spec = importlib.util.spec_from_file_location("system_launch_under_test", BRINGUP / "launch" / entry)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def expand(_entry="system_bringup_launch.py", _trace=None, **overrides):
    context = LaunchContext()
    context.launch_configurations.update({"map_profile": "rmuc2026", "use_rviz": "false", **overrides})
    nodes, commands, model_arguments = [], [], []

    def resolve(value):
        return perform_substitutions(context, normalize_to_list_of_substitutions(value))

    def capture_node(node, ctx):
        node._perform_substitutions(ctx)
        executable = resolve(node.node_executable)
        name = node.node_name.rsplit("/", 1)[-1]
        if name == Node.UNSPECIFIED_NODE_NAME:
            name = "map_saver" if executable == "map_saver_server" else executable
        namespace = node.expanded_node_namespace
        evaluated = evaluate_parameters(ctx, node._Node__parameters or [])
        params = {p.name: p.value for p in to_parameters_list(ctx, name, namespace, evaluated)}
        nodes.append((name, namespace, params, "process"))
        if _trace is not None:
            _trace.setdefault("implementations", {})[name] = (resolve(node.node_package), executable)
            _trace.setdefault("remaps", {})[name] = node.expanded_remapping_rules or []

    def capture_components(action, ctx):
        if _trace is not None:
            _trace.setdefault("containers", []).append(resolve(action._LoadComposableNodes__target_container))
        for node in action._LoadComposableNodes__composable_node_descriptions:
            name = resolve(node.node_name)
            namespace = ctx.launch_configurations.get("ros_namespace", "/")
            evaluated = evaluate_parameters(ctx, node.parameters or [])
            params = {p.name: p.value for p in to_parameters_list(ctx, name, namespace, evaluated)}
            nodes.append((name, namespace, params, "component"))
            if _trace is not None:
                _trace.setdefault("implementations", {})[name] = (resolve(node.package), resolve(node.node_plugin))
                remaps = [*ctx.launch_configurations.get("ros_remaps", []), *(node.remappings or [])]
                _trace.setdefault("remaps", {})[name] = [(resolve(a), resolve(b)) for a, b in remaps]

    def capture_process(action, ctx):
        commands.append([resolve(part) for part in action.cmd])

    original_include = IncludeLaunchDescription.execute

    def include(action, ctx):
        # 当前环境未安装xmacro；只验证模型入口路径和参数，不模拟模型生成。
        path = resolve(action.launch_description_source._LaunchDescriptionSource__location)
        # 旧文件仅归档、不安装，且内容原样保留。基线测试重定向旧文件的内部包含路径。
        if _entry.startswith("old/") and Path(path).parent == BRINGUP / "launch":
            path = str(BRINGUP / "launch/old" / Path(path).name)
            action = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(path), launch_arguments=action.launch_arguments,
            )
        if _trace is not None:
            _trace.setdefault("includes", []).append(Path(path))
        if path.endswith("robot_description_launch.py"):
            model_arguments.append({resolve(k): resolve(v) for k, v in action.launch_arguments})
            return None
        return original_include(action, ctx)

    with patch("ament_index_python.packages.get_package_share_directory", source_share), \
            patch("launch_ros.substitutions.find_package.get_package_share_directory", source_share), \
            patch.object(Node, "execute", capture_node), \
            patch.object(LoadComposableNodes, "execute", capture_components), \
            patch.object(ExecuteProcess, "execute", capture_process), \
            patch.object(IncludeLaunchDescription, "execute", include):
        module = load_main(_entry)
        visit_all_entities_and_collect_futures(module.generate_launch_description(), context)
    return nodes, commands, model_arguments


@pytest.mark.parametrize("mode,localization", [("slam", "gicp"), ("localization", "gicp"), ("localization", "static")])
@pytest.mark.parametrize("navigation", ["true", "false"])
@pytest.mark.parametrize("composition", ["true", "false"])
def test_mode_matrix(mode, localization, navigation, composition):
    nodes, _, _ = expand(mode=mode, localization_mode=localization,
                         use_navigation=navigation, use_composition=composition)
    names = [n[0] for n in nodes]
    assert len(names) == len(set(names)), "重复启动节点"
    assert {"point_lio", "loam_interface", "sensor_scan_generation", "terrain_analysis", "terrain_analysis_ext"} <= set(names)
    assert ("controller_server" in names) == (navigation == "true")
    assert ("fake_vel_transform" in names) == (navigation == "true")
    assert ("nav2_container" in names) == (composition == "true")
    assert ("slam_toolbox" in names) == (mode == "slam")
    assert ("map_server" in names) == (mode == "localization")
    assert ("small_gicp_relocalization" in names) == (mode == "localization" and localization == "gicp")
    assert ("static_transform_publisher_map2odom" in names) == (mode == "slam" or localization == "static")
    assert all(n[1] == "/red_standard_robot1" for n in nodes)


def test_public_arguments():
    with patch("ament_index_python.packages.get_package_share_directory", source_share):
        declarations = [a for a in load_main().generate_launch_description().entities if isinstance(a, DeclareLaunchArgument)]
    names = {a.name for a in declarations}
    assert len(names) == 27
    assert all(a.description for a in declarations)
    assert not names & {"input_source", "transform_publish_period", "output_data_type", "slam", "use_gicp", "map", "prior_pcd_file", "lvx_file_path"}


@pytest.mark.parametrize("namespace", ["", "robot2", "fleet/robot2"])
@pytest.mark.parametrize("composition", ["true", "false"])
def test_namespace_time_and_initial_pose(namespace, composition):
    nodes, _, _ = expand(namespace=namespace, use_sim_time="true", use_composition=composition,
                         map_to_odom_x="3.68", map_to_odom_y="7.98", map_to_odom_yaw="0.5", use_rviz="true")
    by_name = {n[0]: n for n in nodes}
    for name in ("point_lio", "loam_interface", "sensor_scan_generation", "terrain_analysis", "terrain_analysis_ext", "map_server", "small_gicp_relocalization", "controller_server", "rviz2"):
        assert by_name[name][1] == ("/" + namespace if namespace else "/")
        assert by_name[name][2]["use_sim_time"] is True, name
    assert list(by_name["small_gicp_relocalization"][2]["init_pose"]) == [3.68, 7.98, 0.0, 0.0, 0.0, 0.5]
    assert by_name["map_server"][2]["yaml_filename"].endswith("/rmuc2026.yaml")


def test_slam_without_map_or_navigation():
    nodes, _, _ = expand(mode="slam", map_profile="", use_navigation="false", use_sim_time="true", publish_static_map_to_odom="false")
    by_name = {n[0]: n for n in nodes}
    assert "static_transform_publisher_map2odom" not in by_name
    assert by_name["slam_toolbox"][2]["transform_publish_period"] == 0.05
    assert by_name["slam_toolbox"][2]["use_sim_time"] is True
    assert by_name["point_lio"][2]["prior_pcd.enable"] is False
    assert by_name["point_lio"][2]["pcd_save.pcd_save_en"] is True


def test_decision_mock_and_robot_arguments():
    nodes, _, models = expand(use_decision="true", params_file_name="node_params2.yaml", use_referee_mock="true", use_robot_state_pub="true", use_sim_time="true")
    by_name = {n[0]: n for n in nodes}
    assert by_name["tree_exec"][2]["tree_xml_file"].endswith("RMUC_adaptation_2026_B.xml")
    assert by_name["tree_exec"][2]["use_sim_time"] is True
    assert by_name["referee_mock"][2]["use_sim_time"] is True
    assert models[0]["namespace"] == "red_standard_robot1"
    assert models[0]["params_file"].endswith("spr_robot_description/params/robot_description.yaml")
    assert models[0]["use_sim_time"] == "true"


def test_external_recording_topics_and_driver_output():
    nodes, commands, _ = expand(namespace="robot2", use_rosbag_record="true", rosbag_path="driver-output")
    params = next(n[2] for n in nodes if n[0] == "livox_ros_driver2")
    assert params["output_data_type"] == 0
    assert params["rosbag_path"] == "driver-output"
    assert commands[0][:3] == ["ros2", "bag", "record"]
    assert "/robot2/livox/lidar_merged" in commands[0]
    assert not any("red_standard_robot1" in part for part in commands[0])


def test_driver_internal_recording():
    nodes, commands, _ = expand(map_profile="", livox_output_mode="bag", use_navigation="false", enable_imu_bag="false")
    assert [n[0] for n in nodes] == ["livox_ros_driver2"]
    assert nodes[0][2]["output_data_type"] == 1
    assert nodes[0][2]["enable_lidar_bag"] is True
    assert nodes[0][2]["enable_imu_bag"] is False
    assert not commands


def test_numeric_recording_path_remains_a_string():
    nodes, _, _ = expand(livox_output_mode="bag", use_navigation="false", rosbag_path="20261012")
    assert nodes[0][2]["rosbag_path"] == "20261012"


@pytest.mark.parametrize("kwargs,match", [
    ({"map_profile": ""}, "map_profile"),
    ({"map_profile": "does_not_exist"}, "文件不存在"),
    ({"use_decision": "true", "params_file_name": "does_not_exist.yaml"}, "params_file_name"),
    ({"use_decision": "true", "params_file_name": "node_params2"}, "params_file_name"),
    ({"livox_output_mode": "bag"}, "不发布点云"),
    ({"livox_output_mode": "bag", "enable_lidar_bag": "false", "enable_imu_bag": "false"}, "至少"),
    ({"use_sim_time": "auto"}, "not valid"),
    ({"map_to_odom_yaw": "nan"}, "有限数值"),
    ({"map_profile": "../rmuc2026"}, "map_profile"),
])
def test_invalid_combinations(kwargs, match):
    with pytest.raises(RuntimeError, match=match):
        expand(**kwargs)


@pytest.mark.parametrize("composition", ["True", "False"])
@pytest.mark.parametrize("entry,mode,localization", [
    ("rm_navigation_reality_launch.py", "localization", "gicp"),
    ("rm_navigation_reality_lio_launch.py", "localization", "static"),
    ("rm_navigation_reality_launch.py", "slam", "gicp"),
])
def test_covers_original_realtime_node_graph(entry, mode, localization, composition):
    old_trace, new_trace = {}, {}
    old, _, _ = expand(_entry="old/" + entry, world="rmuc2026", slam=str(mode == "slam"),
                        use_gicp=str(localization == "gicp"), use_composition=composition, _trace=old_trace)
    new, _, _ = expand(mode=mode, localization_mode=localization, use_composition=composition, _trace=new_trace)
    # 对比实际展开的旧入口，确保没有漏掉进程/组件，也不额外重复启动节点。
    assert sorted((n[0], n[1], n[3]) for n in old) == sorted((n[0], n[1], n[3]) for n in new)
    assert old_trace["implementations"] == new_trace["implementations"]
    for name in old_trace["remaps"]:
        # 旧硬件节点不在TF remap组内；新入口统一作用域，但它们不发布TF。
        if name not in ("livox_ros_driver2", "merge_cloud_node"):
            assert old_trace["remaps"][name] == new_trace["remaps"][name], name
    old_params = {n[0]: n[2] for n in old}
    for name, _, params, _ in new:
        # 新入口补齐时钟参数；其余算法/硬件/话题配置与旧入口相同。
        if name == "nav2_container":
            continue
        assert {k: v for k, v in params.items() if k != "use_sim_time"} == {
            k: v for k, v in old_params[name].items() if k != "use_sim_time"
        }, name


def test_disabled_optional_configs_are_not_read():
    nodes, _, _ = expand(mode="slam", map_profile="does_not_exist", use_navigation="false",
                         params_file_name="does_not_exist.yaml", use_referee_mock="true",
                         rviz_config_file="does_not_exist", robot_name="does_not_exist")
    assert not {"tree_exec", "referee_mock", "rviz2", "controller_server"} & {n[0] for n in nodes}


@pytest.mark.parametrize("mode", ["slam", "localization"])
@pytest.mark.parametrize("composition", ["true", "false"])
def test_autostart_false(mode, composition):
    nodes, _, _ = expand(mode=mode, use_composition=composition, autostart="false")
    lifecycle = [n for n in nodes if n[0].startswith("lifecycle_manager_")]
    assert len(lifecycle) == 2
    assert all(n[2]["autostart"] is False for n in lifecycle)


@pytest.mark.parametrize("mode", ["slam", "localization"])
@pytest.mark.parametrize("namespace", ["", "fleet/robot2"])
def test_module_includes_and_shared_container(mode, namespace):
    trace = {}
    expand(mode=mode, namespace=namespace, use_decision="true", params_file_name="node_params1.yaml",
           use_rosbag_record="true", _trace=trace)
    assert {p.name for p in trace["includes"]} == {
        "data_processing_launch.py", mode + "_launch.py", "navigation_launch.py",
        "decision_launch.py", "recording_launch.py",
    }
    assert all(p.parent == BRINGUP / "launch" for p in trace["includes"])
    assert len(trace["containers"]) == (2 if mode == "slam" else 3)
    expected = "/" + "/".join(filter(None, [namespace, "nav2_container"]))
    assert {"/" + c.strip("/") for c in trace["containers"]} == {expected}


def test_launch_basenames_are_unique():
    # old/不安装，仅检查新版命名；安装后的递归唯一性另用真实ROS CLI验证。
    names = [p.name for p in (BRINGUP / "launch").glob("*.py")]
    assert len(names) == len(set(names))
    assert len(names) == 8
    assert all(name.endswith("_launch.py") for name in names)
