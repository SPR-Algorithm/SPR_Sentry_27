cd /home/spr/new_ws/
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
cmds=( 
  "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch livox_ros_driver2 rviz_MID360_launch.py"
  #"source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 bag record   /livox/imu_192_168_1_194 /livox/lidar_192_168_1_194"
  #"source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 bag record   /livox/imu_192_168_1_104 /livox/lidar_192_168_1_104"
	)

for cmd in "${cmds[@]}";
do
  echo "Current CMD : $cmd"
  gnome-terminal -- bash -c "cd $(pwd); $cmd; exec bash"
  sleep 0.5
done
#    ros2 run nav2_map_server map_saver_cli -f ground2
# ros2 run nav2_map_server map_saver_cli -f map_0711 --ros-args -r __ns:=/red_standard_robot1




#   ros2 service call /red_standard_robot1/slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '/home/spr/nav2_ws_slam/map_0711.pbstream'}"






