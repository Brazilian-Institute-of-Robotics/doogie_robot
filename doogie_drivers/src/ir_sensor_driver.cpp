#include <string>

#include <time.h>
#include <wiringPi.h>

#include "doogie_drivers/ir_sensor_driver.hpp"
#include "doogie_msgs/DoogiePosition.h"
#include "doogie_msgs/WallDistances.h"

namespace doogie_drivers {

IRSensorDriver::IRSensorDriver() 
  : ads_(0x48) {}

void IRSensorDriver::init() {
  this->initHardware();
  ads_.init(std::string("/dev/i2c-1"));
}

void IRSensorDriver::initHardware() {
  if (wiringPiSetupGpio() == -1) throw("Error on wiringPi setup");

  pinMode(FRONT_EMITTERS_CTRL, OUTPUT);
  pinMode(LEFT_EMITTER_CTRL, OUTPUT);
  pinMode(RIGHT_EMITTER_CTRL, OUTPUT);
  pinMode(ADC_A1_CHANNEL_CTRL, OUTPUT);  // Pin to control analog multiplexer

  digitalWrite(FRONT_EMITTERS_CTRL, LOW);
  digitalWrite(LEFT_EMITTER_CTRL, LOW);
  digitalWrite(RIGHT_EMITTER_CTRL, LOW);
  digitalWrite(ADC_A1_CHANNEL_CTRL, LOW); // Enable channel 0 of analog multiplexer
}

void IRSensorDriver::run() {
  ros::Rate rate(ros::Duration(1.0));
  while(ros::ok()) {
    ROS_INFO("-----------------");
    ROS_INFO("Channel 0 = %.2f", this->computeDistance(LEFT_FRONT));
    ROS_INFO("Channel 1 = %.2f", this->computeDistance(LEFT));
    ROS_INFO("Channel 2 = %.2f", this->computeDistance(RIGHT_FRONT));
    ROS_INFO("Channel 3 = %.2f", this->computeDistance(RIGHT));
    ROS_INFO("-----------------");
    rate.sleep();
  }
}

void IRSensorDriver::turnOnEmitter(IRSensorSide ir_sensor_side) {
  switch (ir_sensor_side) {
    case LEFT_FRONT:
      digitalWrite(FRONT_EMITTERS_CTRL, HIGH);
      return;
    case LEFT:
      digitalWrite(LEFT_EMITTER_CTRL, HIGH);
      return;
    case RIGHT_FRONT:
      digitalWrite(FRONT_EMITTERS_CTRL, HIGH);
      return;
    case RIGHT:
      digitalWrite(RIGHT_EMITTER_CTRL, HIGH);
  }
}

void IRSensorDriver::turnOffEmitter(IRSensorSide ir_sensor_side) {
  switch (ir_sensor_side) {
    case LEFT_FRONT:
      digitalWrite(FRONT_EMITTERS_CTRL, LOW);
      return;
    case LEFT:
      digitalWrite(LEFT_EMITTER_CTRL, LOW);
      return;
    case RIGHT_FRONT:
      digitalWrite(FRONT_EMITTERS_CTRL, LOW);
      return;
    case RIGHT:
      digitalWrite(RIGHT_EMITTER_CTRL, LOW);
  }
}

float IRSensorDriver::computeDistance(IRSensorSide ir_sensor_side) {
  float distance;
  this->turnOnEmitter(ir_sensor_side);
  usleep(180);
  distance = ads_.readVoltage(ir_sensor_side);
  this->turnOffEmitter(ir_sensor_side);
  
  return distance;
}

IRSensorDriver::~IRSensorDriver() {
  digitalWrite(FRONT_EMITTERS_CTRL, LOW);
  digitalWrite(LEFT_EMITTER_CTRL, LOW);
  digitalWrite(FRONT_EMITTERS_CTRL, LOW);
}

}  // namespace doogie_drivers
