/*
 * Copyright (c) 2019, Brazilian Institute of Robotics
 */

#include <ros/ros.h>
#include "controller_manager/controller_manager.h"
#include "doogie_bringup/doogie_hardware.hpp"
#include "doogie_drivers/quadrature_encoder_driver.hpp"

using doogie_drivers::QuadratureEncoder;

int main(int argc, char **argv) {
  ros::init(argc, argv, "doogie_control");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle nh;
  doogie_bringup::DoogieHardware doogie_hardware;
  controller_manager::ControllerManager cm(&doogie_hardware, nh);

  doogie_drivers::QuadratureEncoder quadrature_encoder;

  ros::Duration period(0.1);
  ros::Time last_time;
  last_time = ros::Time::now();
  while (ros::ok()) {
    doogie_hardware.read(period);
    cm.update(ros::Time::now(), period);
    doogie_hardware.write();
    ROS_INFO("Control Loop");
    ROS_INFO("Encoder value = %d", quadrature_encoder.getPulses());
    period.sleep();
    ROS_INFO("Time = %f", (ros::Time::now() - last_time).toSec());
    last_time = ros::Time::now();
  }

  return 0;
}
