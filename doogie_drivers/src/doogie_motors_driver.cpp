/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Brazilian Institute of Robotics.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the PAL Robotics nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*
 * Author: Mateus Menezes
 */

#include <wiringPi.h>
#include "doogie_drivers/doogie_motors_driver.hpp"

namespace doogie_drivers {

DoogieMotorsDriver::DoogieMotorsDriver(float wheel_diameter, float gear_ratio)
  : wheel_diameter_(wheel_diameter)
  , gear_ratio_(gear_ratio) {}

void DoogieMotorsDriver::init() {
  if (wiringPiSetupGpio() == -1) throw("Error on wiringPi setup");

  pinMode(MOTOR_RIGHT_EN1, OUTPUT);
  pinMode(MOTOR_RIGHT_EN2, OUTPUT);
  digitalWrite(MOTOR_RIGHT_EN1, LOW);
  digitalWrite(MOTOR_RIGHT_EN2, LOW);

  pinMode(MOTOR_LEFT_EN1, OUTPUT);
  pinMode(MOTOR_LEFT_EN2, OUTPUT);
  digitalWrite(MOTOR_LEFT_EN1, LOW);
  digitalWrite(MOTOR_LEFT_EN2, LOW);

  pinMode(HBRIDGE_STDBY, OUTPUT);
  digitalWrite(HBRIDGE_STDBY, HIGH);

  pinMode(MOTOR_LEFT_PWM, PWM_OUTPUT);
  pinMode(MOTOR_RIGHT_PWM, PWM_OUTPUT);
  pwmSetMode(PWM_MODE_MS);
  pwmSetClock(PWM_CLOCK);
  pwmSetRange(PWM_RANGE);
}

void DoogieMotorsDriver::setMotorVelocity(float velocity, MotorSide motor_side, VelocityType velocity_type) {
  if (velocity == 0.0) {
    switch (motor_side) {
      case LEFT:
        this->brakeMotor(LEFT);
        return;
      case RIGHT:
        this->brakeMotor(RIGHT);
        return;
    }
  }

  if (velocity > 0) {
    this->setMotorTurningDirection(motor_side, CLOCKWISE);
  } else {
    this->setMotorTurningDirection(motor_side, COUNTER_CLOCKWISE);
  }

  int pwm_value = 0;
  switch (velocity_type) {
    case LINEAR:
      pwm_value = velocity * 62.5f;
      break;
    case ANGULAR:
      pwm_value = velocity * 0.94f;
      break;
  }

  if (motor_side == LEFT) {
    pwmWrite(MOTOR_LEFT_PWM, pwm_value);
    return;
  }

  pwmWrite(MOTOR_RIGHT_PWM, pwm_value);
}

void DoogieMotorsDriver::setMotorsVelocity(float velocity, VelocityType velocity_type) {
  this->setMotorVelocity(velocity, LEFT, velocity_type);
  this->setMotorVelocity(velocity, RIGHT, velocity_type);
}

void DoogieMotorsDriver::brakeMotor(MotorSide motor_side) {
  switch (motor_side) {
    case LEFT:
      digitalWrite(MOTOR_LEFT_EN1, LOW);
      digitalWrite(MOTOR_LEFT_EN2, LOW);
      pwmWrite(MOTOR_LEFT_PWM, 0);
      return;
    case RIGHT:
      digitalWrite(MOTOR_RIGHT_EN1, LOW);
      digitalWrite(MOTOR_RIGHT_EN2, LOW);
      pwmWrite(MOTOR_RIGHT_PWM, 0);
      return;
  }
}

void DoogieMotorsDriver::brakeMotors() {
  digitalWrite(MOTOR_LEFT_EN1, LOW);
  digitalWrite(MOTOR_LEFT_EN2, LOW);
  pwmWrite(MOTOR_LEFT_PWM, 0);

  digitalWrite(MOTOR_RIGHT_EN1, LOW);
  digitalWrite(MOTOR_RIGHT_EN2, LOW);
  pwmWrite(MOTOR_RIGHT_PWM, 0);
}

float DoogieMotorsDriver::getLeftMotorVelocity() {
  return 0;
}

float DoogieMotorsDriver::getRightMotorVelocity() {
  return 0;
}

void DoogieMotorsDriver::setMotorTurningDirection(MotorSide motor_side, TurningDirection turning_direction) {
  switch (motor_side) {
    case LEFT:
      switch (turning_direction) {
        case CLOCKWISE:
          digitalWrite(MOTOR_LEFT_EN1, HIGH);
          digitalWrite(MOTOR_LEFT_EN2, LOW);
          return;
        case COUNTER_CLOCKWISE:
          digitalWrite(MOTOR_LEFT_EN1, LOW);
          digitalWrite(MOTOR_LEFT_EN2, HIGH);
          return;
      }

    case RIGHT:
      switch (turning_direction) {
        case CLOCKWISE:
            digitalWrite(MOTOR_RIGHT_EN1, LOW);
            digitalWrite(MOTOR_RIGHT_EN2, HIGH);
            return;
        case COUNTER_CLOCKWISE:
            digitalWrite(MOTOR_RIGHT_EN1, RIGHT);
            digitalWrite(MOTOR_RIGHT_EN2, LOW);
            return;
      }
  }
}

DoogieMotorsDriver::~DoogieMotorsDriver() {
  this->brakeMotors();
}

}  // namespace doogie_drivers
