cmake_minimum_required(VERSION 3.8)
project(dual_mid360_merge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(merge_cloud_node src/dual_mid360_merge.cpp)
ament_target_dependencies(
  merge_cloud_node
  rclcpp
  sensor_msgs
  message_filters
  livox_ros_driver2
)
target_link_libraries(merge_cloud_node Eigen3::Eigen)

# 安装可执行文件
install(TARGETS
  merge_cloud_node
  DESTINATION lib/${PROJECT_NAME})

  if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

