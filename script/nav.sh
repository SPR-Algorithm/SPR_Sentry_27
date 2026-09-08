cd /home/spr/new_ws/
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# 启动顺序：串口 → robot_description → 导航（需 Livox 有点云后 odom 帧才会出现）
# 先验地图与当前环境一致：默认 use_gicp:=True，由 small_gicp_relocalization 发布 map->odom（勿设静态 TF）
cmds=( 
 "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch spr_sentry_serial spr_sentry_serial_launch.py"
 
 "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch spr_robot_description robot_description_launch.py"
 
 "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch spr_nav_bringup rm_navigation_reality_launch.py world:=map_0521 slam:=False use_robot_state_pub:=False"
 
#"ros2 launch spr_decision run.launch.py params_file_name:=node_params"

#~/Groot2/bin/groot2

)

for cmd in "${cmds[@]}";
do
	echo Current CMD : "$cmd"
	gnome-terminal -- bash -c "cd $(pwd);source install/setup.bash;$cmd;exec bash;"
	sleep 0.2
done
























