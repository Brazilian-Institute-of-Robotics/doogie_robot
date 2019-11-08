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

  QuadratureEncoder::channel_a_pin_ = 14;
  QuadratureEncoder::channel_b_pin_ = 15;
  QuadratureEncoder::index_ = -1;

  QuadratureEncoder::QuadratureEncoder::pulses_ = 0;
  QuadratureEncoder::revolutions_ = 0;
  QuadratureEncoder::QuadratureEncoder::pulses_per_rev_ = 360;
  QuadratureEncoder::encoding_ = doogie_drivers::QuadratureEncoder::X4_ENCODING;

  doogie_drivers::QuadratureEncoder quadrature_encoder(14, 15, -1, 360, doogie_drivers::QuadratureEncoder::X4_ENCODING);

  ros::Duration period(0.1);
  while (ros::ok()) {
    doogie_hardware.read(period);
    cm.update(ros::Time::now(), period);
    doogie_hardware.write();
    ROS_INFO("Control Loop");
    ROS_INFO("Encoder value = %d", quadrature_encoder.getPulses());
    period.sleep();
  }

  return 0;
}
