#!/usr/bin/env bash
# 用法：bash script/map_replay.sh /path/to/bag（支持相对路径/含空格路径）
# 输入应为 record.sh 录制的融合点云 + IMU + 云台关节状态，不适用于驱动原始双雷达 bag。
# 回放前可用 ros2 bag info /path/to/bag 检查这三个输入话题是否存在且有消息。
if [[ $# -ne 1 ]]; then
  echo "用法：bash $0 /path/to/bag" >&2
  exit 1
fi
bag_dir=$(realpath -e -- "$1") || exit 1
if [[ ! -f "$bag_dir/metadata.yaml" ]]; then
  echo "bag 目录缺少 metadata.yaml：$bag_dir" >&2
  exit 1
fi
printf -v bag_dir_quoted '%q' "$bag_dir"

cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." || exit 1
source /opt/ros/humble/setup.bash || exit 1
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release || exit 1

# 离线建图：不启动串口/Livox/merge/导航/决策；模型、数据处理及 SLAM 统一使用 /clock。
# 只播放输入话题，模型与 LIO 重算 TF；禁止同时回放旧 /tf、/tf_static 或处理结果。
# 话题对应当前 YAML；修改 namespace 或输入话题时，此处也须同步修改。
# 播放器先暂停，待其他终端启动后在播放器终端按空格开始；bag_dir 只传给播放器。
# 同时重建二维地图和三维 PCD；处理跟不上时可在播放命令中添加 --rate 0.5。
cmds=(
  "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch spr_robot_description robot_description_launch.py use_sim_time:=true use_rviz:=false"
  "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch spr_nav_bringup replay_bringup_launch.py use_robot_state_pub:=false use_sim_time:=true"
  "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 bag play $bag_dir_quoted --clock --start-paused --topics /red_standard_robot1/livox/lidar_merged /red_standard_robot1/livox/imu_192_168_1_194 /red_standard_robot1/serial/gimbal_joint_state"
)

for cmd in "${cmds[@]}";
do
  echo "Current CMD : $cmd"
  gnome-terminal -- bash -c "cd \"$PWD\" && $cmd; exec bash"
  sleep 0.5
done

# SLAM 仍在运行时，另开已 source 工作空间的终端保存二维地图：
# ros2 run nav2_map_server map_saver_cli -f map_replay --ros-args -r __ns:=/red_standard_robot1 -p use_sim_time:=true
# 可选：保存 SLAM Toolbox 位姿图：
# ros2 service call /red_standard_robot1/slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '/absolute/path/map_replay'}"
# 保存二维地图后，对回放 launch 按 Ctrl+C 正常退出，Point-LIO 将保存 PCD/scans.pcd。
# PCD 位于 src/spr_sentry_nav/point_lio/PCD/，会覆盖同名旧文件，回放前注意备份。
