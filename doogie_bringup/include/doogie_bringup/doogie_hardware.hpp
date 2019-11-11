/*
 * Copyright (c) 2019, Brazilian Institute of Robotics
 */

#include <string>
#include "hardware_interface/joint_state_interface.h"
#include "hardware_interface/joint_command_interface.h"
#include "hardware_interface/robot_hw.h"
#include "doogie_drivers/doogie_motors_driver.hpp"
#include "doogie_drivers/quadrature_encoder_driver.hpp"
#include "ros/ros.h"

#ifndef DOOGIE_BRINGUP_DOOGIE_HARDWARE_HPP_
#define DOOGIE_BRINGUP_DOOGIE_HARDWARE_HPP_

namespace doogie_bringup {

class DoogieHardware : public hardware_interface::RobotHW {
 public:
  DoogieHardware();
  void write();
  void read(const double &dt);

 private:
  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::VelocityJointInterface velocity_joint_interface_;
  doogie_drivers::DoogieMotorsDriver doogie_motors_;
  doogie_drivers::QuadratureEncoder doogie_encoders_;
  std::string logger_name_;
  double joint_velocity_cmd_[2];
  double joint_position_[2];
  double joint_velocity_[2];
  double joint_effort_[2];

  enum WheelSide{
    LEFT_WHEEL,
    RIGHT_WHEEL
  };
};


}

#endif  // DOOGIE_BRINGUP_DOOGIE_HARDWARE_HPP_
