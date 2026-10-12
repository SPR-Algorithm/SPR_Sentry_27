cd /home/spr/new_ws/
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

cmds=( 
 "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch spr_sentry_serial spr_sentry_serial_launch.py"
  
 "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch spr_robot_description robot_description_launch.py"
 
 "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch spr_nav_bringup rm_navigation_reality_lio_launch.py world:=rmuc2026 slam:=False use_robot_state_pub:=False map_to_odom_x:=0.0 map_to_odom_y:=0.0 map_to_odom_yaw:=45.0 "
 
"source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch spr_decision run.launch.py params_file_name:=node_params_b_demo"

#"source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch spr_referee_mock referee_mock.launch.py"
 
#~/Groot2/bin/groot2

 #"source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch pb2025_nav_bringup rm_navigation_reality_lio_launch.py world:=0401map slam:=False use_robot_state_pub:=False map_to_odom_x:=0.0 map_to_odom_y:=0.38 map_to_odom_yaw:=0.0"
 
#"source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch commander commander_pass.launch.py polygons_json:=$json_string"
 # "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py world:=0401map slam:=False use_robot_state_pub:=False use_gicp:=False map_to_odom_x:=0.0 map_to_odom_y:=0.0 map_to_odom_yaw:=0.0"
  
#"ros2 launch rm_decision_cpp run.launch.py params_file_name:=node_params_rmul2026"
#"ros2 launch rm_decision_cpp run.launch.py params_file_name:=node_params_nav_test"
 
 #"ros2 bag play rosbag2_2025_05_27-02_00_08"

  #~/Groot2/bin/groot2


)

for cmd in "${cmds[@]}";
do
	echo Current CMD : "$cmd"
	gnome-terminal -- bash -c "cd $(pwd);source install/setup.bash;$cmd;exec bash;"
	sleep 0.2
done


