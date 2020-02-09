cmake_minimum_required(VERSION 2.8.3)
project(doogie_drivers)

add_compile_options(-std=c++11)

set(CATKIN_BUILD_DEPS
  roscpp
  std_msgs
  control_msgs
)

find_package(catkin REQUIRED COMPONENTS ${CATKIN_BUILD_DEPS})
find_package(Boost REQUIRED COMPONENTS)

set(DOOGIE_MOTORS_DRIVER_LIB doogie_motors_driver)
set(QUADRATURE_ENCODER_DRIVER_LIB quadrature_encoder_driver)
set(ADS1115_DRIVER_LIB ads1115_driver)

catkin_package(
  INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
  CATKIN_DEPENDS ${CATKIN_BUILD_DEPS}
  LIBRARIES 
    ${DOOGIE_MOTORS_DRIVER_LIB} 
    ${QUADRATURE_ENCODER_DRIVER_LIB}
  DEPENDS Boost
)
  
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
)
    
add_library(${DOOGIE_MOTORS_DRIVER_LIB}
  src/${DOOGIE_MOTORS_DRIVER_LIB}.cpp
)

add_library(${QUADRATURE_ENCODER_DRIVER_LIB}
  src/${QUADRATURE_ENCODER_DRIVER_LIB}.cpp
)

add_library(${ADS1115_DRIVER_LIB}
  src/${ADS1115_DRIVER_LIB}.cpp
)
    
find_library(wiringPi_LIB wiringPi)
target_link_libraries(${DOOGIE_MOTORS_DRIVER_LIB}
  ${wiringPi_LIB}
  ${Boost_LIBRARIES}
)

target_link_libraries(${QUADRATURE_ENCODER_DRIVER_LIB}
  ${wiringPi_LIB}
  ${Boost_LIBRARIES}
)

install(
  TARGETS 
    ${DOOGIE_MOTORS_DRIVER_LIB} 
    ${QUADRATURE_ENCODER_DRIVER_LIB}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
  PATTERN "*~" EXCLUDE
)

add_executable(ir_sensors_node src/ir_sensors_node.cpp)
target_link_libraries(ir_sensors_node ${ADS1115_DRIVER_LIB})

# add_executable(step_response_node 
#   test/step_response.cpp
# )

# target_link_libraries(step_response_node 
#   ${DOOGIE_MOTORS_DRIVER_LIB}
#   ${QUADRATURE_ENCODER_DRIVER_LIB}
#   ${catkin_LIBRARIES}
# )
