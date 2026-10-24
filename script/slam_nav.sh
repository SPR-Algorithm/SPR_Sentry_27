#!/usr/bin/env bash
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." || exit 1
source /opt/ros/humble/setup.bash || exit 1
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release || exit 1

# 边建图边导航：SLAM 提供实时二维地图，Nav2 规划控制；不读取已有地图。
# 默认不启动决策；确认定位/地图正常后再下发导航目标。
cmds=(
  "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch spr_sentry_serial spr_sentry_serial_launch.py"
  "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch spr_robot_description robot_description_launch.py use_sim_time:=false use_rviz:=false"
  "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch spr_nav_bringup bringup_launch.py mode:=slam use_navigation:=true use_decision:=false use_robot_state_pub:=false use_sim_time:=false"
)

for cmd in "${cmds[@]}";
do
  echo "Current CMD : $cmd"
  gnome-terminal -- bash -c "cd \"$PWD\" && $cmd; exec bash"
  sleep 0.5
done

# 建图结束前，在已 source 工作空间的终端保存二维地图（文件名按需修改）：
# ros2 run nav2_map_server map_saver_cli -f map_0711 --ros-args -r __ns:=/red_standard_robot1
# 可选：保存 SLAM Toolbox 位姿图，用于后续继续建图：
# ros2 service call /red_standard_robot1/slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '/absolute/path/map_0711'}"
# 保存二维地图后，对主 launch 按 Ctrl+C 正常退出，Point-LIO 将保存 PCD/scans.pcd。
# PCD 位于 src/spr_sentry_nav/point_lio/PCD/，会覆盖同名旧文件，建图前注意备份。
