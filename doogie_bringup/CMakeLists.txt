cmake_minimum_required(VERSION 2.8.3)
project(doogie_bringup)

add_compile_options(-std=c++11)

set(CATKIN_BUILD_DEPS
  controller_manager
  hardware_interface
  roscpp
  sensor_msgs
  doogie_drivers
)

find_package(catkin REQUIRED COMPONENTS ${CATKIN_BUILD_DEPS})

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS ${CATKIN_BUILD_DEPS}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(doogie_hardware
  src/doogie_hardware.cpp
)

target_link_libraries(doogie_hardware
  ${catkin_LIBRARIES}
)
add_executable(doogie_control_node 
  src/doogie_control_node.cpp
)

target_link_libraries(doogie_control_node
  ${catkin_LIBRARIES}
  doogie_hardware
)
