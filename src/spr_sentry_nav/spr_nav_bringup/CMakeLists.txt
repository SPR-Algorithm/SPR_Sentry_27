cmake_minimum_required(VERSION 3.8)
project(spr_nav_bringup)

find_package(ament_cmake_auto REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

add_executable(map_odom_tf_manager src/map_odom_tf_manager.cpp)
target_compile_features(map_odom_tf_manager PUBLIC cxx_std_17)
ament_target_dependencies(map_odom_tf_manager
  rclcpp tf2 tf2_ros geometry_msgs diagnostic_msgs)
install(TARGETS map_odom_tf_manager DESTINATION lib/${PROJECT_NAME})

add_executable(component_container_isolated src/component_container_isolated.cpp)
target_compile_features(component_container_isolated PUBLIC cxx_std_17)
ament_target_dependencies(component_container_isolated rclcpp rclcpp_components)
target_link_libraries(component_container_isolated ${CMAKE_DL_LIBS})
install(TARGETS component_container_isolated DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_uncrustify
    ament_cmake_cpplint
    ament_cmake_flake8
  )
  ament_lint_auto_find_test_dependencies()
endif()

# old/只保留源码归档。ROS 2按文件名递归查找，不能将同名旧launch一起安装。
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.py"
  PATTERN "old" EXCLUDE
  PATTERN "__pycache__" EXCLUDE
)

ament_auto_package(
  INSTALL_TO_SHARE
  map
  pcd
  config
  behavior_trees
  rviz
)
