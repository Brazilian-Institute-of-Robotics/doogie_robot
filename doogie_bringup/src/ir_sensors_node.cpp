#include <ros/ros.h>
#include "doogie_drivers/ads1115_driver.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "doogie_control");

  doogie_drivers::ADS115Driver ads(0x48);
  ads.init(std::string("/dev/i2c-1"));

  ros::NodeHandle nh;

  ros::Rate rate(ros::Duration(1.0));
  while(ros::ok()) {
    ROS_INFO("Channel 0 = %.2f", ads.readVoltage(0));
    ROS_INFO("Channel 1 = %.2f", ads.readVoltage(1));
    ROS_INFO("Channel 2 = %.2f", ads.readVoltage(2));
    ROS_INFO("Channel 3 = %.2f", ads.readVoltage(3));
    rate.sleep();
  }

  return 0;
}