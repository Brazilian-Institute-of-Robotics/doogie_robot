/*
 * Copyright (c) 2019, Brazilian Institute of Robotics
 */

#include <cstddef>
#include <random>
#include "doogie_bringup/doogie_hardware.hpp"

using doogie_drivers::MotorSide;
using doogie_drivers::VelocityType;

namespace doogie_bringup {

DoogieHardware::DoogieHardware()
  : logger_name_("doogie_hardware_interface") {
  std::string joint_names[2] = {"wheel/left_joint", "wheel/right_joint"};

  for (size_t i = 0; i < 2; i++) {
    this->joint_velocity_cmd_[i] = 0;
    this->joint_position_[i] = 0;
    this->joint_velocity_[i] = 0;
    this->joint_effort_[i] = 0;
    hardware_interface::JointStateHandle joint_state_handle(joint_names[i],
                                                             &this->joint_position_[i],
                                                             &this->joint_velocity_[i],
                                                             &this->joint_effort_[i]);
    this->joint_state_interface_.registerHandle(joint_state_handle);

    hardware_interface::JointHandle joint_handle(joint_state_handle, &this->joint_velocity_cmd_[i]);
    this->velocity_joint_interface_.registerHandle(joint_handle);
  }

  registerInterface(&this->joint_state_interface_);
  registerInterface(&this->velocity_joint_interface_);

  this->doogie_motors_.init();
}

void DoogieHardware::write() {
  this->doogie_motors_.setMotorVelocity(joint_velocity_cmd_[LEFT_WHEEL], MotorSide::LEFT, VelocityType::ANGULAR);
  this->doogie_motors_.setMotorVelocity(joint_velocity_cmd_[RIGHT_WHEEL], MotorSide::RIGHT, VelocityType::ANGULAR);
  ROS_DEBUG_NAMED(logger_name_, "Left wheel angular velocity written = %lf", joint_velocity_cmd_[LEFT_WHEEL]);
  ROS_DEBUG_NAMED(logger_name_, "Right wheel angular velocity written = %lf", joint_velocity_cmd_[RIGHT_WHEEL]);
}

void DoogieHardware::read(const ros::Duration &period) {
  joint_velocity_[LEFT_WHEEL] = joint_velocity_cmd_[LEFT_WHEEL] + 0.1;
  joint_velocity_[RIGHT_WHEEL] = joint_velocity_cmd_[RIGHT_WHEEL] + 0.1;
  joint_position_[LEFT_WHEEL] += 0.1;
  joint_position_[RIGHT_WHEEL] += + 0.1;

  ROS_DEBUG_NAMED(logger_name_, "Left wheel angular velocity read = %lf", joint_velocity_[LEFT_WHEEL]);
  ROS_DEBUG_NAMED(logger_name_, "Right wheel angular velocity read = %lf", joint_velocity_[LEFT_WHEEL]);
  ROS_DEBUG_NAMED(logger_name_, "Period = %lf", period.toSec());
}

}  // namespace doogie_bringup
