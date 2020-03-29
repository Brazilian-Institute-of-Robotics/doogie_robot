#include <ros/ros.h>
#include <exception>
#include "doogie_drivers/ir_sensor_driver.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "ir_sensors_node");

  doogie_drivers::IRSensorDriver ir_sensor_driver;

  try {
    ir_sensor_driver.init();
    ir_sensor_driver.run();
  } catch (std::runtime_error error) {
    ROS_FATAL("%s", error.what());
    ros::shutdown();
  }

  return 0;
}