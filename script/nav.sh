#!/usr/bin/env bash
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." || exit 1
source /opt/ros/humble/setup.bash || exit 1
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release || exit 1

# 已有地图 GICP 定位 + 导航：串口 → 模型 → 主入口。
# map_profile 对应同名 YAML/PGM 和 PCD，必须与现场一致；rmuc2026 是当前仓库可用示例。
# map→odom 仅由TF管理器发布，GICP提交匹配结果；模型TF与Livox数据齐备后才能完成定位。
# 启用决策请使用 nav_decision.sh；不使用 GICP 请使用 nav_static.sh。
cmds=(
  "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch spr_sentry_serial spr_sentry_serial_launch.py"
  "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch spr_robot_description robot_description_launch.py use_sim_time:=false use_rviz:=false"
  "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch spr_nav_bringup bringup_launch.py map_profile:=rmuc2026 mode:=localization localization_mode:=gicp use_navigation:=true use_decision:=false use_robot_state_pub:=false use_sim_time:=false"
)

for cmd in "${cmds[@]}";
do
  echo "Current CMD : $cmd"
  gnome-terminal -- bash -c "cd \"$PWD\" && $cmd; exec bash"
  sleep 0.5
done
