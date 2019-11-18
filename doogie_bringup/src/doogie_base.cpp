/*
 * Copyright (c) 2019, Brazilian Institute of Robotics
 */

#include <ros/ros.h>
#include "controller_manager/controller_manager.h"
#include "doogie_bringup/doogie_hardware.hpp"
#include "doogie_drivers/quadrature_encoder_driver.hpp"

using doogie_drivers::QuadratureEncoder;
using doogie_drivers::EncoderSide;

int main(int argc, char **argv) {
  ros::init(argc, argv, "doogie_control");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle nh;
  doogie_bringup::DoogieHardware doogie_hardware(nh);
  controller_manager::ControllerManager cm(&doogie_hardware, nh);

  ros::Duration period(0.1);
  ros::Time last_time = ros::Time::now();

  while (ros::ok()) {
    ros::Time time = ros::Time::now();

    doogie_hardware.read(time);
    cm.update(ros::Time::now(), time - last_time);
    doogie_hardware.write(time);

    period.sleep();
    last_time = time;
  }

  return 0;
}
