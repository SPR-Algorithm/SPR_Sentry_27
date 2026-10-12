"""回放入口干运行测试：只展开启动图，不播放bag或启动节点。"""

from unittest.mock import patch

import pytest
from launch.actions import DeclareLaunchArgument

from test_system_launch import BRINGUP, expand, load_main, source_share


def replay(**overrides):
    return expand(_entry="replay_bringup_launch.py", **overrides)


def test_public_arguments_and_defaults():
    with patch("ament_index_python.packages.get_package_share_directory", source_share):
        declarations = [
            a for a in load_main("replay_bringup_launch.py").generate_launch_description().entities
            if isinstance(a, DeclareLaunchArgument)
        ]
    assert {a.name for a in declarations} == {
        "namespace", "robot_name", "params_file", "log_level", "rviz_config_file",
        "use_sim_time", "autostart", "use_composition", "use_respawn", "use_robot_state_pub",
        "use_rviz", "publish_static_map_to_odom",
    }
    assert all(a.description for a in declarations)
    nodes, commands, _ = replay()
    by_name = {n[0]: n for n in nodes}
    assert by_name["point_lio"][2]["use_sim_time"] is True
    assert by_name["slam_toolbox"][2]["use_sim_time"] is True
    assert by_name["slam_toolbox"][2]["transform_publish_period"] == 0.05
    assert "static_transform_publisher_map2odom" in by_name
    assert "nav2_container" in by_name
    assert not commands, "回放入口不能自行启动bag play或bag record"


@pytest.mark.parametrize("composition", ["true", "false"])
@pytest.mark.parametrize("clock", ["true", "false"])
@pytest.mark.parametrize("namespace", ["", "robot2", "fleet/robot2"])
def test_replay_graph_namespace_and_clock(composition, clock, namespace):
    trace = {}
    nodes, commands, models = replay(
        use_composition=composition, use_sim_time=clock, namespace=namespace,
        map_profile="does_not_exist", _trace=trace,
    )
    expected = {
        "point_lio", "loam_interface", "sensor_scan_generation", "terrain_analysis",
        "terrain_analysis_ext", "pointcloud_to_laserscan", "slam_toolbox", "map_saver",
        "lifecycle_manager_slam", "static_transform_publisher_map2odom",
    }
    if composition == "true":
        expected.add("nav2_container")
    names = [n[0] for n in nodes]
    assert len(names) == len(set(names))
    assert set(names) == expected
    for name, node_ns, params, _ in nodes:
        assert node_ns == ("/" + namespace if namespace else "/")
        if name != "static_transform_publisher_map2odom":
            assert params["use_sim_time"] is (clock == "true"), name
    by_name = {n[0]: n for n in nodes}
    assert by_name["point_lio"][2]["prior_pcd.enable"] is False
    assert by_name["point_lio"][2]["pcd_save.pcd_save_en"] is True
    assert by_name["point_lio"][2]["common.lid_topic"] == "livox/lidar_merged"
    assert not commands and not models
    assert set(trace["includes"]) == {
        BRINGUP / "launch/data_processing_launch.py", BRINGUP / "launch/slam_launch.py",
    }
    if composition == "true":
        container = "/" + "/".join(filter(None, [namespace, "nav2_container"]))
        assert trace["containers"] == [container]
    else:
        assert not trace.get("containers")


@pytest.mark.parametrize("composition", ["True", "False"])
def test_matches_old_replay_without_navigation(composition):
    old_trace, new_trace = {}, {}
    old, _, _ = expand(
        _entry="old/slam_launch_bag.py", use_composition=composition, _trace=old_trace,
    )
    new, _, _ = replay(use_composition=composition, _trace=new_trace)
    nav_nodes = {
        "fake_vel_transform", "controller_server", "smoother_server", "planner_server",
        "behavior_server", "bt_navigator", "waypoint_follower", "velocity_smoother",
        "lifecycle_manager_navigation",
    }
    old = [n for n in old if n[0] not in nav_nodes]
    assert sorted((n[0], n[1], n[3]) for n in new) == sorted((n[0], n[1], n[3]) for n in old)
    old_params = {n[0]: n[2] for n in old}
    for name, _, params, _ in new:
        assert new_trace["implementations"][name] == old_trace["implementations"][name]
        assert new_trace["remaps"][name] == old_trace["remaps"][name]
        if name != "nav2_container":
            assert {k: v for k, v in params.items() if k != "use_sim_time"} == {
                k: v for k, v in old_params[name].items() if k != "use_sim_time"
            }, name


def test_model_rviz_and_optional_switches():
    nodes, commands, models = replay(
        namespace="robot2", use_robot_state_pub="true", use_rviz="true",
        publish_static_map_to_odom="false", autostart="false",
    )
    by_name = {n[0]: n for n in nodes}
    assert "static_transform_publisher_map2odom" not in by_name
    assert by_name["lifecycle_manager_slam"][2]["autostart"] is False
    assert by_name["rviz2"][1] == "/robot2"
    assert by_name["rviz2"][2]["use_sim_time"] is True
    assert models[0]["namespace"] == "robot2"
    assert models[0]["use_sim_time"] == "true"
    assert models[0]["params_file"].endswith("spr_robot_description/params/robot_description.yaml")
    assert not commands


def test_live_and_replay_choose_hardware_internally():
    live, _, _ = expand(use_live_lidar="false")
    offline, _, _ = replay(use_live_lidar="true", livox_output_mode="bag")
    hardware = {"livox_ros_driver2", "merge_cloud_node"}
    assert hardware <= {n[0] for n in live}
    assert not hardware & {n[0] for n in offline}
    assert "point_lio" in {n[0] for n in offline}


@pytest.mark.parametrize("kwargs,match", [
    ({"params_file": "does_not_exist.yaml"}, "params_file"),
    ({"use_sim_time": "auto"}, "not valid"),
    ({"use_rviz": "true", "rviz_config_file": "does_not_exist.rviz"}, "rviz_config_file"),
    ({"use_robot_state_pub": "true", "robot_name": "does_not_exist"}, "robot_name"),
])
def test_invalid_replay_parameters(kwargs, match):
    with pytest.raises(RuntimeError, match=match):
        replay(**kwargs)
