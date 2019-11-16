/*
 * Copyright (c) 2019, Brazilian Institute of Robotics
 */

#include <cstddef>
#include <stdexcept>
#include "ros/ros.h"
#include "doogie_bringup/doogie_hardware.hpp"

using doogie_drivers::MotorSide;
using doogie_drivers::VelocityType;
using doogie_drivers::EncoderSide;

namespace doogie_bringup {

DoogieHardware::DoogieHardware(const ros::NodeHandle &nh)
  : doogie_encoders_(3, 0.016175, 30)
  , logger_name_("doogie_hardware_interface") {
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

  try {
    this->doogie_motors_.init();
    this->doogie_encoders_.init();
  } catch (std::runtime_error &error) {
    ROS_FATAL("%s", error.what());
    ros::shutdown();
  }

  pid_[LEFT_WHEEL].initPid(2.0, 1.0, 0, 10.0, -10.0, false, nh);
  pid_[LEFT_WHEEL].printValues();

  pid_[RIGHT_WHEEL].initPid(2.0, 1.0, 0, 10.0, -10.0, false, nh);
  pid_[RIGHT_WHEEL].printValues();

  this->write_last_time_ = this->read_last_time_ =  ros::Time::now();
}

void DoogieHardware::write(const ros::Time &time) {
  double ang_vel[2];
  ros::Duration dt = time - write_last_time_;

  ROS_DEBUG_NAMED(logger_name_, "Left wheel angular velocity requested = %lf", joint_velocity_cmd_[LEFT_WHEEL]);
  ROS_DEBUG_NAMED(logger_name_, "Right wheel angular velocity requested = %lf", joint_velocity_cmd_[RIGHT_WHEEL]);

  this->doogie_encoders_.updateVelocity(dt.toSec());

  ang_vel[LEFT_WHEEL] = pid_[LEFT_WHEEL].computeCommand(
    joint_velocity_cmd_[LEFT_WHEEL] - doogie_encoders_.getVelocity(EncoderSide::LEFT_ENC), dt);

  ang_vel[RIGHT_WHEEL] = pid_[RIGHT_WHEEL].computeCommand(
    joint_velocity_cmd_[RIGHT_WHEEL] - doogie_encoders_.getVelocity(EncoderSide::RIGHT_ENC), dt);

  this->doogie_motors_.setMotorVelocity(static_cast<int>(ang_vel[LEFT_WHEEL]), MotorSide::LEFT, VelocityType::ANGULAR);
  this->doogie_motors_.setMotorVelocity(static_cast<int>(ang_vel[RIGHT_WHEEL]), MotorSide::RIGHT, VelocityType::ANGULAR);

  ROS_DEBUG_NAMED(logger_name_, "Left wheel angular velocity pid = %lf", ang_vel[LEFT_WHEEL]);
  ROS_DEBUG_NAMED(logger_name_, "Right wheel angular velocity pid = %lf", ang_vel[RIGHT_WHEEL]);

  write_last_time_ = time;
}

void DoogieHardware::read(const ros::Time &time) {
  joint_position_[LEFT_WHEEL] = this->doogie_encoders_.getAngularPosition(EncoderSide::LEFT_ENC);
  joint_position_[RIGHT_WHEEL] = this->doogie_encoders_.getAngularPosition(EncoderSide::RIGHT_ENC);

  double dt = (time - read_last_time_).toSec();
  this->doogie_encoders_.updateVelocity(dt);

  joint_velocity_[LEFT_WHEEL] = this->doogie_encoders_.getVelocity(EncoderSide::LEFT_ENC);
  joint_velocity_[RIGHT_WHEEL] = this->doogie_encoders_.getVelocity(EncoderSide::RIGHT_ENC);

  read_last_time_ = time;
}

}  // namespace doogie_bringup
