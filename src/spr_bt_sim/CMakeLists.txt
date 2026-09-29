cmake_minimum_required(VERSION 3.8)
project(spr_bt_sim)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/decision_result_mock.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  behavior_tree
  launch
  config
  map
  rviz
  DESTINATION share/${PROJECT_NAME}
)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
