#!/usr/bin/env bash

set -euo pipefail

workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$workspace"

map_profile="${1:-map_0521}"
if (($# > 0)); then
  shift
fi

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

# 用法：./script/nav.sh [map_profile] [其他 launch 参数...]
# map_profile 必须同时存在对应的 map/reality/*.yaml 和 pcd/reality/*.pcd。
open_terminal "SPR navigation" \
  ros2 launch spr_nav_bringup bringup_launch.py \
  mode:=localization \
  localization_mode:=gicp \
  "map_profile:=$map_profile" \
  use_robot_state_pub:=true \
  "$@"
