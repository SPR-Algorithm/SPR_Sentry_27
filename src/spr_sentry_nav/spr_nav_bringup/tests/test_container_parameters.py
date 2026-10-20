"""Regression: nested Nav2 costmaps must receive process-level YAML.

Run after sourcing ROS:
  pytest -q src/spr_sentry_nav/spr_nav_bringup/tests
No ROS processes, hardware, or recording are started by these tests.
"""
import importlib.util
from pathlib import Path

import pytest
import yaml
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.utilities import evaluate_parameters, to_parameters_list

BRINGUP = Path(__file__).resolve().parents[1]


@pytest.fixture
def source_share(monkeypatch, tmp_path):
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "logs"))
    share = tmp_path / "share"
    share.mkdir()
    for folder in ("config", "launch", "rviz", "behavior_trees"):
        (share / folder).symlink_to(BRINGUP / folder, target_is_directory=True)
    maps = share / "map/reality"
    maps.mkdir(parents=True)
    (maps / "fixture.yaml").write_text("image: fixture.pgm\nresolution: 0.05\n")
    import ament_index_python.packages as packages
    original = packages.get_package_share_directory

    def lookup(name):
        return str(share) if name == "spr_nav_bringup" else original(name)

    monkeypatch.setattr(packages, "get_package_share_directory", lookup)
    monkeypatch.setattr(FindPackageShare, "find", lambda self, name: lookup(name))
    return share


def main_actions(entry, overrides):
    spec = importlib.util.spec_from_file_location("container_test_entry", BRINGUP / "launch" / entry)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    context.launch_configurations.update(overrides)
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    # Inspect only the entry's own actions; do not execute child launches or nodes.
    return context, module._setup(context)


def containers(actions, context):
    found = []
    for action in actions:
        if isinstance(action, GroupAction):
            found.extend(containers(action.get_sub_entities(), context))
        elif isinstance(action, Node):
            name = perform_substitutions(
                context, normalize_to_list_of_substitutions(action._Node__node_name or ""))
            if name == "nav2_container":
                found.append(action)
    return found


@pytest.mark.parametrize("entry,mode", [
    ("bringup_launch.py", "slam"),
    ("bringup_launch.py", "localization"),
    ("replay_bringup_launch.py", "slam"),
])
@pytest.mark.parametrize("namespace", ["", "robot1", "/fleet/robot1/"])
@pytest.mark.parametrize("sim_time", ["false", "true"])
@pytest.mark.parametrize("autostart", ["false", "true"])
def test_full_yaml_reaches_nested_nodes(source_share, entry, mode, namespace, sim_time, autostart):
    context, actions = main_actions(entry, {
        "mode": mode, "localization_mode": "static", "map_profile": "fixture",
        "namespace": namespace, "use_sim_time": sim_time, "autostart": autostart,
        "use_composition": "true", "use_rviz": "false", "use_robot_state_pub": "false",
    })
    container, = containers(actions, context)
    parameters = evaluate_parameters(context, container._Node__parameters)
    files = [value for value in parameters if isinstance(value, Path)]
    assert len(files) == 1, "Container must receive full YAML, not just a use_sim_time dictionary"
    tree = yaml.safe_load(files[0].read_text())
    normalized_ns = namespace.strip("/")
    if normalized_ns:
        tree = tree[normalized_ns]
    for section in ("local_costmap", "global_costmap"):
        nested_namespace = "/".join(filter(None, [normalized_ns, section]))
        actual = {p.name: p.value for p in to_parameters_list(
            context, section, nested_namespace, files)}
        assert actual["robot_base_frame"] == "gimbal_yaw_fake"
        assert actual["global_frame"] == ("odom" if section == "local_costmap" else "map")
        assert actual["use_sim_time"] is (sim_time == "true")
        assert "intensity_voxel_layer" in actual["plugins"]
        assert "obstacle_layer" not in actual["plugins"]
        if section == "local_costmap":
            assert actual["width"] == 10
        assert actual["intensity_voxel_layer.terrain_map" +
                      ("_ext" if section == "global_costmap" else "") + ".topic"] == (
            ("/" + normalized_ns if normalized_ns else "") +
            ("/terrain_map" if section == "local_costmap" else "/terrain_map_ext"))
    if entry == "bringup_launch.py" and mode == "localization":
        assert tree["map_server"]["ros__parameters"]["yaml_filename"] == str(
            source_share / "map/reality/fixture.yaml")
    assert parameters[-1]["use_sim_time"] is (sim_time == "true")
    assert parameters[-1]["autostart"] is (autostart == "true")
    assert "<robot_namespace>" not in files[0].read_text()
    assert "$(find-pkg-share" not in files[0].read_text()


@pytest.mark.parametrize("entry", ["bringup_launch.py", "replay_bringup_launch.py"])
def test_no_container_for_standalone(source_share, entry):
    context, actions = main_actions(entry, {
        "mode": "slam", "use_composition": "false", "use_rviz": "false",
    })
    assert not containers(actions, context)


def test_driver_only_recording_has_no_container(source_share):
    context, actions = main_actions("bringup_launch.py", {
        "livox_output_mode": "bag", "use_navigation": "false", "use_decision": "false",
        "use_rosbag_record": "false", "use_rviz": "false",
    })
    assert not containers(actions, context)
