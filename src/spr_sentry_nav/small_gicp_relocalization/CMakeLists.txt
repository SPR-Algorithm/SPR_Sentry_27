cmake_minimum_required(VERSION 3.16)
project(small_gicp_relocalization)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Keep project warnings visible, but do not promote warnings from the
## third-party small_gicp headers to errors.
add_compile_options(-Wall)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#######################
## Find dependencies ##
#######################

find_package(ament_cmake_auto REQUIRED)
find_package(OpenMP REQUIRED)
find_package(small_gicp QUIET CONFIG)
ament_auto_find_build_dependencies()

# Older versions of this package assumed that small_gicp had been installed
# globally, but did not declare or discover it.  Match the upstream fix while
# pinning the dependency so clean workspace builds are reproducible.
if(NOT small_gicp_FOUND)
  include(FetchContent)
  message(STATUS "small_gicp not found; fetching pinned v1.0.0")
  FetchContent_Declare(
    small_gicp
    GIT_REPOSITORY https://github.com/koide3/small_gicp.git
    GIT_TAG v1.0.0
    GIT_SHALLOW TRUE
  )
  FetchContent_MakeAvailable(small_gicp)
  set(SMALL_GICP_TARGET small_gicp)
elseif(TARGET small_gicp::small_gicp)
  # The installed package exports a namespaced target, while FetchContent
  # exposes the in-tree target without a namespace.
  set(SMALL_GICP_TARGET small_gicp::small_gicp)
else()
  message(FATAL_ERROR "small_gicp was found but did not export a usable CMake target")
endif()

###########
## Build ##
###########

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)

target_link_libraries(${PROJECT_NAME}
  OpenMP::OpenMP_CXX
  ${SMALL_GICP_TARGET}
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN small_gicp_relocalization::SmallGicpRelocalizationNode
  EXECUTABLE ${PROJECT_NAME}_node
)

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_height_filter test/test_height_filter.cpp
    ENV "ROS_LOCALHOST_ONLY=1" "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/test_ros_logs"
  )
  target_link_libraries(test_height_filter ${PROJECT_NAME})
set(ament_cmake_clang_format_CONFIG_FILE "${CMAKE_SOURCE_DIR}/.clang-format")
find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_uncrustify
    ament_cmake_flake8
  )
  ament_lint_auto_find_test_dependencies()
endif()

#############
## Install ##
#############

ament_auto_package(
  INSTALL_TO_SHARE
  launch
)
