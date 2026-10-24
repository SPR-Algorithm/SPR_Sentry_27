#!/usr/bin/env bash
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." || exit 1
source /opt/ros/humble/setup.bash || exit 1
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release || exit 1

# 仅 GICP 定位：已有地图 + 数据处理 + map_server，不启动 Nav2 规划控制/决策/SLAM。
# map_profile 对应同名 YAML/PGM 和 PCD；rmuc2026 是仓库示例，必须与现场一致。
cmds=(
  "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch spr_sentry_serial spr_sentry_serial_launch.py"
  "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch spr_robot_description robot_description_launch.py use_sim_time:=false use_rviz:=false"
  "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch spr_nav_bringup bringup_launch.py \
      map_profile:=rmuc2026 mode:=localization localization_mode:=gicp \
      use_navigation:=false use_decision:=false use_robot_state_pub:=false use_sim_time:=false"
)

for cmd in "${cmds[@]}";
do
  echo "Current CMD : $cmd"
  gnome-terminal -- bash -c "cd \"$PWD\" && $cmd; exec bash"
  sleep 0.5
done
