cmake_minimum_required(VERSION 2.8.3)
project(doogie_drivers)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES doogie_drivers
#  DEPENDS wiringPi
)

include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)

# find_library(wiringPi_LIB wiringPi)
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/doogie_drivers.cpp
# )

# target_link_libraries(doogie_motors
#   ${wiringPi_LIB}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.hpp"
#   PATTERN "~" EXCLUDE
# )
