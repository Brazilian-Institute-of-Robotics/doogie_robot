#include <ros/ros.h>
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <stdexcept>
#include "doogie_drivers/doogie_motors_driver.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "doogie_control");
  ros::NodeHandle nh;
  
  doogie_drivers::DoogieMotorsDriver doogie_motors_ctrl;

  try {
    doogie_motors_ctrl.init();
    doogie_motors_ctrl.setMotorsVelocity(1.0);
    ros::Rate loop_rate(10);
 
    while (ros::ok()) {
      ROS_INFO("Loop test");
      loop_rate.sleep();
    }
  
    doogie_motors_ctrl.brakeMotors();
    return 0;
  } catch(std::runtime_error &error){
    ROS_ERROR("%s", error.what());
  }
    
}
