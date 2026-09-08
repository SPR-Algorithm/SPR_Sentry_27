cmake_minimum_required(VERSION 3.8)
project(spr_referee_mock)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rm_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(QT_NO_CREATE_VERSIONLESS_TARGETS ON)
find_package(Qt6 QUIET COMPONENTS Widgets)
if(Qt6_FOUND)
  set(QT_PACKAGE Qt6)
else()
  find_package(Qt5 REQUIRED COMPONENTS Widgets)
  set(QT_PACKAGE Qt5)
endif()

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(panel_headers include/spr_referee_mock/referee_mock_widget.hpp)
if(QT_PACKAGE STREQUAL "Qt6")
  qt6_wrap_cpp(referee_mock_moc ${panel_headers})
else()
  qt5_wrap_cpp(referee_mock_moc ${panel_headers})
endif()

add_executable(referee_mock_gui
  src/referee_mock_node.cpp
  src/referee_mock_widget.cpp
  src/referee_mock_gui_main.cpp
  ${referee_mock_moc}
)

target_include_directories(referee_mock_gui PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(referee_mock_gui ${QT_PACKAGE}::Widgets)
ament_target_dependencies(referee_mock_gui rclcpp rm_interfaces std_msgs geometry_msgs)

install(TARGETS referee_mock_gui
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
