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
  : doogie_encoders_(12, 0.016175, 30)
  , logger_name_("doogie_hardware_interface") {
  std::string joint_names[2] = {"wheel/left_joint", "wheel/right_joint"};

  for (size_t i = 0; i < 2; i++) {
    joint_velocity_cmd_[i] = 0;
    joint_position_[i] = 0;
    joint_velocity_[i] = 0;
    joint_effort_[i] = 0;
    hardware_interface::JointStateHandle joint_state_handle(joint_names[i],
                                                            &joint_position_[i],
                                                            &joint_velocity_[i],
                                                            &joint_effort_[i]);
    joint_state_interface_.registerHandle(joint_state_handle);

    hardware_interface::JointHandle joint_handle(joint_state_handle, &joint_velocity_cmd_[i]);
    velocity_joint_interface_.registerHandle(joint_handle);
  }

  registerInterface(&joint_state_interface_);
  registerInterface(&velocity_joint_interface_);

  try {
    doogie_motors_.init();
    doogie_encoders_.init();
  } catch (std::runtime_error &error) {
    ROS_FATAL("%s", error.what());
    ros::shutdown();
  }

  ros::NodeHandle left_wheel_nh("left_wheel_pid");
  pid_[LEFT_WHEEL].init(left_wheel_nh, false);

  ros::NodeHandle right_wheel_nh("right_wheel_pid");
  pid_[RIGHT_WHEEL].init(right_wheel_nh, false);

  write_last_time_  =  ros::Time::now();
}

void DoogieHardware::write(const ros::Time &time) {
  double ang_vel[2];
  double ang_vel_error[2];
  ros::Duration dt = time - write_last_time_;

  doogie_encoders_.updateVelocity(dt.toSec());

  ang_vel_error[LEFT_WHEEL] = joint_velocity_cmd_[LEFT_WHEEL] - doogie_encoders_.getVelocity(EncoderSide::LEFT_ENC);
  ang_vel[LEFT_WHEEL] = pid_[LEFT_WHEEL].computeCommand(ang_vel_error[LEFT_WHEEL], dt);
  
  ang_vel_error[RIGHT_WHEEL] = joint_velocity_cmd_[RIGHT_WHEEL] - doogie_encoders_.getVelocity(EncoderSide::RIGHT_ENC);
  ang_vel[RIGHT_WHEEL] = pid_[RIGHT_WHEEL].computeCommand(ang_vel_error[RIGHT_WHEEL], dt);

  doogie_motors_.setMotorVelocity(static_cast<int>(ang_vel[LEFT_WHEEL]), MotorSide::LEFT);
  doogie_motors_.setMotorVelocity(static_cast<int>(ang_vel[RIGHT_WHEEL]), MotorSide::RIGHT);

  write_last_time_ = time;
}

void DoogieHardware::read(const ros::Time &time) {
  joint_position_[LEFT_WHEEL] = doogie_encoders_.getAngularPosition(EncoderSide::LEFT_ENC);
  joint_position_[RIGHT_WHEEL] = doogie_encoders_.getAngularPosition(EncoderSide::RIGHT_ENC);

  joint_velocity_[LEFT_WHEEL] = doogie_encoders_.getVelocity(EncoderSide::LEFT_ENC);
  joint_velocity_[RIGHT_WHEEL] = doogie_encoders_.getVelocity(EncoderSide::RIGHT_ENC);
}

}  // namespace doogie_bringup
