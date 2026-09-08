cd /home/spr/new_ws/
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# bag 目录（需含 metadata.yaml；若只有 .db3，脚本会自动 reindex）
BAG_DIR="rosbag2_2026_05_20-11_18_03"

if [ ! -f "${BAG_DIR}/metadata.yaml" ]; then
  echo "未找到 ${BAG_DIR}/metadata.yaml，正在 reindex..."
  source /opt/ros/humble/setup.bash
  ros2 bag reindex "${BAG_DIR}" -s sqlite3
fi

cmds=(
  # 离线 SLAM：不启 Livox 驱动与 merge_cloud_node
  "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch spr_nav_bringup slam_launch_bag.py use_robot_state_pub:=False"
  # 必须先 source install，否则 rosbag2 找不到 livox_ros_driver2 会跳过 lidar_merged
  "source /opt/ros/humble/setup.bash && source install/setup.bash && sleep 8 && ros2 bag play ${BAG_DIR} --clock"
)

for cmd in "${cmds[@]}"; do
  echo "Current CMD : $cmd"
  gnome-terminal -- bash -c "cd $(pwd); source install/setup.bash; $cmd; exec bash"
  sleep 0.5
done

# 回放结束后保存地图：
# ros2 run nav2_map_server map_saver_cli -f map_名字 --ros-args -r __ns:=/red_standard_robot1
# ros2 service call /red_standard_robot1/slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '/home/spr/new_ws/map_名字.pbstream'}"
