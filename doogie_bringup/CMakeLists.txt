cmake_minimum_required(VERSION 2.8.3)
project(doogie_bringup)

add_compile_options(-std=c++11)

set(CATKIN_BUILD_DEPS
  controller_manager
  hardware_interface
  roscpp
  sensor_msgs
)

find_package(catkin REQUIRED COMPONENTS ${CATKIN_BUILD_DEPS})

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS ${CATKIN_BUILD_DEPS}
#  DEPENDS wiringPi
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/doogie_bringup_node.cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )
