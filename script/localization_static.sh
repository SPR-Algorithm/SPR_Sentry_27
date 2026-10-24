#!/usr/bin/env bash
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." || exit 1
source /opt/ros/humble/setup.bash || exit 1
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release || exit 1

# 仅固定变换定位：TF管理器持续发布固定map→odom + Point-LIO + map_server，不启动导航/决策/GICP/SLAM。
# map_profile 为当前仓库示例；固定初始位姿必须按现场修改（x/y 米，yaw 弧度）。
cmds=(
  "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch spr_sentry_serial spr_sentry_serial_launch.py"
  "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch spr_robot_description robot_description_launch.py use_sim_time:=false use_rviz:=false"
  "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch spr_nav_bringup bringup_launch.py \
        map_profile:=rmuc2026 mode:=localization localization_mode:=static map_to_odom_x:=0.0 map_to_odom_y:=0.0 map_to_odom_yaw:=0.0 \
        use_navigation:=false use_decision:=false use_robot_state_pub:=false use_sim_time:=false"
)

for cmd in "${cmds[@]}";
do
  echo "Current CMD : $cmd"
  gnome-terminal --working-directory="$PWD" -- bash -c "$cmd; exec bash"
  sleep 0.5
done
