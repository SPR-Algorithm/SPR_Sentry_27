#!/usr/bin/env bash

set -euo pipefail

workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$workspace"

source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

open_terminal() {
  local title="$1"
  shift

  local command workspace_command
  printf -v command '%q ' "$@"
  printf -v workspace_command '%q' "$workspace"

  gnome-terminal --title="$title" -- bash -lc \
    "cd $workspace_command && source /opt/ros/humble/setup.bash && source install/setup.bash && $command; exec bash"
}

# 串口不属于 bringup_launch.py，仍需单独启动。
open_terminal "SPR serial" \
  ros2 launch spr_sentry_serial spr_sentry_serial_launch.py

sleep 0.5

# 建图只启动 SLAM 所需链路；robot_description 由统一入口一并启动。
open_terminal "SPR mapping" \
  ros2 launch spr_nav_bringup bringup_launch.py \
  mode:=slam \
  use_navigation:=false \
  use_robot_state_pub:=true \
  "$@"

# 地图保存示例（另开终端执行）：
# ros2 run nav2_map_server map_saver_cli -f <map_name> --ros-args -r __ns:=/red_standard_robot1
