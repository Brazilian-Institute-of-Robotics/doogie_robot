#ifndef DOOGIE_DRIVERS_IR_SENSOR_DRIVER_HPP_
#define DOOGIE_DRIVERS_IR_SENSOR_DRIVER_HPP_

#include <ros/ros.h>
#include "doogie_drivers/ads1115_driver.hpp"
#include "doogie_msgs/DoogiePosition.h"

#define FRONT_EMITTERS_CTRL 22
#define LEFT_EMITTER_CTRL 4
#define RIGHT_EMITTER_CTRL 13
#define ADC_A1_CHANNEL_CTRL 11

namespace doogie_drivers {

enum IRSensorSide {
  LEFT_FRONT,
  LEFT,
  RIGHT_FRONT,
  RIGHT,
};

class IRSensorDriver {
 public:
  IRSensorDriver();
  void run();
  void init();
  void turnOnEmitter(IRSensorSide ir_sensor_side);
  void turnOffEmitter(IRSensorSide ir_sensor_side);
  float computeDistance(IRSensorSide ir_sensor_side);
  ~IRSensorDriver();

 private:
  void initHardware();
  void doogiePositionCallback(const doogie_msgs::DoogiePositionConstPtr doogie_position);
  ros::NodeHandle nh_;
  ros::NodeHandle ph_;
  ros::Publisher wall_distances_pub_;
  ros::Subscriber doogie_position_sub_;

  ADS115Driver ads_;

  float sensor_thresholds_[4];
};

}  // namespace doogie_drivers

#endif  // DOOGIE_DRIVERS_IR_SENSOR_DRIVER_HPP_
