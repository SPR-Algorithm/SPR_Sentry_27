cd /home/spr/new_ws/
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

cmds=( 
 "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch spr_sentry_serial spr_sentry_serial_launch.py"
  
 "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch spr_robot_description robot_description_launch.py"
 
 "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch spr_nav_bringup rm_navigation_reality_lio_launch.py world:=rmuc2026 slam:=False use_robot_state_pub:=False use_rviz:=true map_to_odom_x:=3.68 map_to_odom_y:=7.98 map_to_odom_yaw:=0.0 "

"source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch spr_decision run.launch.py params_file_name:=node_params2"

#"source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch spr_referee_mock referee_mock.launch.py"


 #"source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 bag record   /red_standard_robot1/tf  /red_standard_robot1/tf_static  /red_standard_robot1/livox/imu_192_168_1_194   /red_standard_robot1/livox/lidar_merged"
 
#~/Groot2/bin/groot2
)

for cmd in "${cmds[@]}";
do
	echo Current CMD : "$cmd"
	gnome-terminal -- bash -c "cd $(pwd);source install/setup.bash;$cmd;exec bash;"
	sleep 0.2
done
























