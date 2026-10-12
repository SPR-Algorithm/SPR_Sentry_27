colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

json_string='"[[[1,2],[3,4],[5,6],[7,8]]]"'  

cmds=( 
 "ros2 run nav2_map_server map_saver_cli -f rmuc0727 --ros-args -r __ns:=/red_standard_robot1"
 
 

)

for cmd in "${cmds[@]}";
do
	echo Current CMD : "$cmd"
	gnome-terminal -- bash -c "cd $(pwd);source install/setup.bash;$cmd;exec bash;"
	sleep 0.2
done




