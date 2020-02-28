#include <string>

#include <time.h>
#include <wiringPi.h>

#include "doogie_drivers/ir_sensor_driver.hpp"
#include "doogie_msgs/DoogiePosition.h"
#include "doogie_msgs/WallDistances.h"
#include "doogie_core/utils.hpp"

using doogie_core::DoogieUtils;

namespace doogie_drivers {

IRSensorDriver::IRSensorDriver()
  : ads_(0x48)
  , ph_("~") {
  DoogieUtils::getParameterHelper<float>(ph_, "ir_driver", "front_left_threshold",
                                         &sensor_thresholds_[LEFT_FRONT], 0.5);
  DoogieUtils::getParameterHelper<float>(ph_, "ir_driver", "left_threshold",
                                         &sensor_thresholds_[LEFT], 0.5);
  DoogieUtils::getParameterHelper<float>(ph_, "ir_driver", "front_right_threshold",
                                         &sensor_thresholds_[RIGHT_FRONT], 0.5);
  DoogieUtils::getParameterHelper<float>(ph_, "ir_driver", "right_threshold",
                                         &sensor_thresholds_[RIGHT], 0.5);
}

void IRSensorDriver::init() {
  this->initHardware();
  ads_.init(std::string("/dev/i2c-1"));

  wall_distances_pub_ = nh_.advertise<doogie_msgs::WallDistances>("wall_distances", 1);
  doogie_position_sub_ = nh_.subscribe("doogie_position", 1, &IRSensorDriver::doogiePositionCallback, this);
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
  digitalWrite(ADC_A1_CHANNEL_CTRL, LOW);  // Enable channel 0 of analog multiplexer
}

void IRSensorDriver::run() {
  ROS_INFO("IR Sensors node has started!");
  ros::spin();
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
  float distance = ads_.readVoltage(ir_sensor_side);

  this->turnOnEmitter(ir_sensor_side);
  usleep(1000);
  distance = ads_.readVoltage(ir_sensor_side) - distance;
  this->turnOffEmitter(ir_sensor_side);

  if (distance > sensor_thresholds_[ir_sensor_side]) return 0;

  return 1.0;
}

void IRSensorDriver::doogiePositionCallback(const doogie_msgs::DoogiePositionConstPtr doogie_position) {
  doogie_msgs::WallDistances wall_distances;

  wall_distances.front_left_sensor.range = this->computeDistance(LEFT_FRONT);
  wall_distances.left_sensor.range = this->computeDistance(LEFT);
  wall_distances.front_right_sensor.range = this->computeDistance(RIGHT_FRONT);
  wall_distances.right_sensor.range = this->computeDistance(RIGHT);

  wall_distances_pub_.publish(wall_distances);
}

IRSensorDriver::~IRSensorDriver() {
  digitalWrite(FRONT_EMITTERS_CTRL, LOW);
  digitalWrite(LEFT_EMITTER_CTRL, LOW);
  digitalWrite(FRONT_EMITTERS_CTRL, LOW);
}

}  // namespace doogie_drivers
