cd /home/spr/new_ws/
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
cmds=( 
  "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch spr_sentry_serial spr_sentry_serial_launch.py"
  "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch spr_robot_description robot_description_launch.py"
  "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch spr_nav_bringup rm_navigation_reality_launch.py slam:=True use_robot_state_pub:=False"
  #"ros2 bag play rosbag2_2025_05_27-19_02_42" #6.23
  #"source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 bag record   /red_standard_robot1/tf  /red_standard_robot1/tf_static  /red_standard_robot1/livox/imu   /red_standard_robot1/livox/lidar"
 # "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 bag record   /red_standard_robot1/serial/gimbal_joint_state   /red_standard_robot1/livox/imu   /red_standard_robot1/livox/lidar"
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






