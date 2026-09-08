cmake_minimum_required(VERSION 3.8)
project(rm_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)  # 将.msg/.srv/.action转换为c++/python等可使用的代码
find_package(std_msgs REQUIRED)  # ros2标准消息包
find_package(geometry_msgs REQUIRED)  # ros2几何消息包
find_package(nav_msgs REQUIRED)  # 导航相关消息

rosidl_generate_interfaces(${PROJECT_NAME}
  # "msg/referee/EventData.msg"
  # "msg/referee/GameResult.msg"
  # "msg/referee/GameRobotHp.msg"
  "msg/referee/GameState.msg"
  # "msg/referee/GameStatus.msg"
  # "msg/referee/RobotPos.msg"
  # "msg/referee/RobotStatus.msg"
  # "msg/referee/HurtData.msg"
  # "msg/referee/PowerHeatData.msg"
  # "msg/referee/RobotBuff.msg"
  # "msg/referee/RobotInteractiveData.msg"
  # "msg/referee/RobotPos.msg"
  # "msg/referee/RobotStatus.msg"
  "msg/referee/SentryCmd.msg"
  # "msg/referee/ShootData.msg"
  "msg/ChassisCmd.msg"
  "msg/Gimbal.msg"
  "msg/GimbalCmd.msg"
  "msg/ShootCmd.msg"
  "msg/Target.msg"

DEPENDENCIES  # 可能使用其他ros2包定义的消息，生成代码时一并处理
  std_msgs
  geometry_msgs
  nav_msgs
)

ament_export_dependencies(rosidl_default_runtime)  # 向其他依赖该包的ros2包声明:使用接口时还需要rosidl_default_runtime
ament_package()
