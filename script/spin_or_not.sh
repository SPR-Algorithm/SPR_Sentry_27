cd /home/spr/new_ws/

cmds=( 
 "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch spr_sentry_serial spr_sentry_serial_launch.py"
 
 " source install/setup.bash &&  ros2 topic pub /red_standard_robot1/spin_or_not std_msgs/msg/UInt8 '{data: 1}' -r 10"
 
 #"source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch spr_nav_bringup rm_navigation_reality_launch.py world:=map_0521 slam:=False use_robot_state_pub:=False"
 
#"ros2 launch spr_decision run.launch.py params_file_name:=node_params"

#~/Groot2/bin/groot2

)

for cmd in "${cmds[@]}";
do
	echo Current CMD : "$cmd"
	gnome-terminal -- bash -c "cd $(pwd);source install/setup.bash;$cmd;exec bash;"
	sleep 0.2
done
























