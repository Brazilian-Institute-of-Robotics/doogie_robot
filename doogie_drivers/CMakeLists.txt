cmake_minimum_required(VERSION 2.8.3)
project(doogie_drivers)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED)

set(DOOGIE_MOTORS_DRIVER_LIB doogie_motors_driver)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${DOOGIE_MOTORS_DRIVER_LIB}
)
  
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
    
add_library(${DOOGIE_MOTORS_DRIVER_LIB}
  src/${DOOGIE_MOTORS_DRIVER_LIB}.cpp
)
    
find_library(wiringPi_LIB wiringPi)
target_link_libraries(${DOOGIE_MOTORS_DRIVER_LIB}
  ${wiringPi_LIB}
)

install(TARGETS ${DOOGIE_MOTORS_DRIVER_LIB}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
  PATTERN "*~" EXCLUDE
)
