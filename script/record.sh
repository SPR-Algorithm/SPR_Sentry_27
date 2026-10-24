#!/usr/bin/env bash
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." || exit 1
source /opt/ros/humble/setup.bash || exit 1
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release || exit 1

# 外部话题录制 + 手动建图：雷达正常发布，记录融合点云/IMU/TF/云台关节状态。
# 不要求已有地图；可供 map_replay.sh 回放。也可在导航主命令上加 use_rosbag_record:=true。
# 录制由 recording_launch.py 启动；rosbag_path 指定根目录，每次自动建立带时间的bag子目录。
# 默认保存到工作空间下 rosbags/rosbag_时间_唯一后缀/，按 Ctrl+C 完成录制收尾。
# 与 map.sh 的建图配置相同，仅额外开启 rosbag2；增加序列化、内存缓冲和磁盘写入开销。
# Point-LIO/SLAM 单独退出时，只要驱动、merge 和录制进程仍在，输入还会继续录制。
# 整个主 launch 退出（包括关闭 RViz）会结束录制；不是独立于整套启动的故障保护。
# 输入从建图开始完整录下、未丢帧且 bag 正常收尾时，可用 map_replay.sh 重新建二维/三维地图。
# 仅录融合点云，不录双雷达原始点云；可重跑建图，不能离线重做双雷达融合。
cmds=(
  "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch spr_sentry_serial spr_sentry_serial_launch.py"
  "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch spr_robot_description robot_description_launch.py use_sim_time:=false use_rviz:=false"
  "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch spr_nav_bringup bringup_launch.py mode:=slam use_navigation:=false use_decision:=false use_robot_state_pub:=false use_sim_time:=false use_rosbag_record:=true rosbag_path:=rosbags"
)

for cmd in "${cmds[@]}";
do
  echo "Current CMD : $cmd"
  gnome-terminal -- bash -c "cd \"$PWD\" && $cmd; exec bash"
  sleep 0.5
done

# 若需独立终端录制，先将上面的 use_rosbag_record 改为 false，避免双重录制：
# mkdir -p rosbags
# ros2 bag record --storage sqlite3 --output "rosbags/rosbag_$(date +%Y-%m-%d_%H-%M-%S_%N)" /red_standard_robot1/livox/lidar_merged /red_standard_robot1/livox/imu_192_168_1_194 /red_standard_robot1/tf /red_standard_robot1/tf_static /red_standard_robot1/serial/gimbal_joint_state
# 结束时 Ctrl+C 正常退出，等待 rosbag2 完成写入后再关闭终端。
# 输出为 rosbags/rosbag_时间_唯一后缀/ 内的 .db3 数据文件及 metadata.yaml，不是单个独立文件。
# 如需同时保留本次二维地图，在停止前执行：
# ros2 run nav2_map_server map_saver_cli -f map_record --ros-args -r __ns:=/red_standard_robot1
# 本模式同时建图，退出时会保存 src/spr_sentry_nav/point_lio/PCD/scans.pcd，注意备份旧文件。
