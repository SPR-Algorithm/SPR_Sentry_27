cmake_minimum_required(VERSION 3.8)
project(spr_nav_bringup)

find_package(ament_cmake_auto REQUIRED)

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
