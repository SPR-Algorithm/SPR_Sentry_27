cmake_minimum_required(VERSION 3.11)
project(spr_sentry_serial)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rm_interfaces REQUIRED)

include_directories(include)

add_executable(serial_node
  src/serial_node.cpp
  src/gimbal_yaw_update.cpp
  src/publish_game_state.cpp
)
ament_target_dependencies(serial_node
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  rm_interfaces
)

install(TARGETS serial_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
