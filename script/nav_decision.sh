#!/usr/bin/env bash
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." || exit 1
source /opt/ros/humble/setup.bash || exit 1
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release || exit 1

# 在 nav.sh 的 GICP + 导航基础上启用决策；一次只运行一个整套启动脚本，勿重复启动串口/导航。
# map_profile 按现场地图修改；决策文件直接写含 .yaml 的文件名，行为树仍由该 YAML 选择。
# node_params1.yaml=A树，node_params2.yaml=B树，node_params_b_demo.yaml=B演示树。
# 默认使用真实裁判消息（use_referee_mock=false）；决策可能主动发导航目标和底盘姿态/旋转指令。
# 如不使用 GICP，将 localization_mode 改为 static，并按现场设置 map_to_odom_x/y/yaw。
cmds=(
  "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch spr_sentry_serial spr_sentry_serial_launch.py"
  "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch spr_robot_description robot_description_launch.py use_sim_time:=false use_rviz:=false"
  "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch spr_nav_bringup bringup_launch.py map_profile:=rmuc2026 mode:=localization localization_mode:=gicp use_navigation:=true use_decision:=true params_file_name:=node_params1.yaml use_referee_mock:=false use_robot_state_pub:=false use_sim_time:=false"
)

for cmd in "${cmds[@]}";
do
  echo "Current CMD : $cmd"
  gnome-terminal -- bash -c "cd \"$PWD\" && $cmd; exec bash"
  sleep 0.5
done
