/*
 * Copyright (c) 2019, Brazilian Institute of Robotics
 */

#include "ros/ros.h"
#include "control_msgs/JointControllerState.h"
#include "doogie_drivers/doogie_motors_driver.hpp"
#include "doogie_drivers/quadrature_encoder_driver.hpp"
#include "std_msgs/UInt8.h"
#include <cstddef>

using doogie_drivers::EncoderSide;

class StepResponse {
 public:
  StepResponse(const ros::NodeHandle &nh)
    : cmd_received(false)
    , nh_(nh) 
    , doogie_encoders_(12 , 0.016175, 30)
    , step_duration_(0) 
    , cmd_(0) {

    try {
      this->doogie_motors_.init();
      this->doogie_encoders_.init();
    } catch (std::runtime_error &error) {
      ROS_FATAL("%s", error.what());
      ros::shutdown();
    }

    cmd_subcriber_ = nh_.subscribe("step_response_cmd", 1, &StepResponse::stepResponseCallback, this);
    left_wheel_joint_controller_state_pub_ = nh_.advertise<control_msgs::JointControllerState>("left_wheel_joint_state", 1);
    right_wheel_joint_controller_state_pub_ = nh_.advertise<control_msgs::JointControllerState>("right_wheel_joint_state", 1);
  }

  void stepResponseCallback(const std_msgs::UInt8::ConstPtr& cmd) {
    this->cmd_ = cmd->data;
    cmd_received = true;
    ROS_INFO("Received step response command! Magnitude was %d", this->cmd_);
  }

  void sendCommandToMotors() {
    doogie_motors_.setMotorsVelocity(static_cast<int>(this->cmd_));
  }

  void pubWheelsJointControllerState(double dt) {
    step_duration_ += dt;

    if (step_duration_ > 3.0) {
      cmd_received = false;
      step_duration_ = 0;
      cmd_ = 0;
      doogie_motors_.brakeMotors();
      return;
    }
    
    doogie_encoders_.updateVelocity(dt);
    
    left_wheel_joint_controller_state_.command = cmd_;
    left_wheel_joint_controller_state_.time_step = dt;
    left_wheel_joint_controller_state_.process_value = doogie_encoders_.getVelocity(EncoderSide::LEFT_ENC);

    right_wheel_joint_controller_state_.command = cmd_;
    right_wheel_joint_controller_state_.time_step = dt;
    right_wheel_joint_controller_state_.process_value = doogie_encoders_.getVelocity(EncoderSide::RIGHT_ENC);

    left_wheel_joint_controller_state_pub_.publish(left_wheel_joint_controller_state_);
    right_wheel_joint_controller_state_pub_.publish(right_wheel_joint_controller_state_);
  }

  bool cmd_received;
 private:
  uint8_t cmd_;
  double step_duration_;
  ros::NodeHandle nh_;

  ros::Subscriber cmd_subcriber_;
  ros::Publisher left_wheel_joint_controller_state_pub_;
  ros::Publisher right_wheel_joint_controller_state_pub_;

  control_msgs::JointControllerState left_wheel_joint_controller_state_;
  control_msgs::JointControllerState right_wheel_joint_controller_state_;

  doogie_drivers::DoogieMotorsDriver doogie_motors_;
  doogie_drivers::QuadratureEncoder doogie_encoders_;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "step_response_node");
  
  ros::NodeHandle nh;
  StepResponse step_response(nh);  

  ros::Duration rate(0.1);
  ros::Time last_time = ros::Time::now();

  ROS_INFO("Step response node has started");

  while (ros::ok()) {
    ros::Time time = ros::Time::now();
    
    if (step_response.cmd_received) {
      step_response.sendCommandToMotors();
    }

    step_response.pubWheelsJointControllerState((time - last_time).toSec());
    last_time = time;
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}