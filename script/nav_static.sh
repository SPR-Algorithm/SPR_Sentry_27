#!/usr/bin/env bash
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." || exit 1
source /opt/ros/humble/setup.bash || exit 1
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release || exit 1

# 已有地图 + 固定 map→odom + 导航：保留 Point-LIO，不启动 GICP/SLAM Toolbox。
# 必须确认地图与实际初始位姿一致；固定 TF 不是自动重定位或 GICP 失败时的保底。
# map_profile 为当前仓库示例；map_to_odom_x/y 单位米，yaw 单位弧度，请按现场修改。
cmds=(
  "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch spr_sentry_serial spr_sentry_serial_launch.py"
  "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch spr_robot_description robot_description_launch.py use_sim_time:=false use_rviz:=false"
  "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch spr_nav_bringup bringup_launch.py map_profile:=rmuc2026 mode:=localization localization_mode:=static map_to_odom_x:=0.0 map_to_odom_y:=0.0 map_to_odom_yaw:=0.0 use_navigation:=true use_decision:=false use_robot_state_pub:=false use_sim_time:=false"
)

for cmd in "${cmds[@]}";
do
  echo "Current CMD : $cmd"
  gnome-terminal -- bash -c "cd \"$PWD\" && $cmd; exec bash"
  sleep 0.5
done
