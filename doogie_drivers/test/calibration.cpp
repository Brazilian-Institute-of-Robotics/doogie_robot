#include <ros/ros.h>
#include <exception>
#include "doogie_drivers/ir_sensor_driver.hpp"

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/mean.hpp>

using namespace boost::accumulators;

class Calibration {
 public:
  Calibration() { ir_sensor_driver.init(); }
  void start() {
    ROS_INFO("Calibration has started!");
    timer_ = nh_.createTimer(ros::Duration(0.1), &Calibration::timerCallback, this);
    ros::spin();
  }

 private:
  ros::NodeHandle nh_;
  ros::Timer timer_;
  accumulator_set<float, stats<tag::mean>> left_front_acc_;
  accumulator_set<float, stats<tag::mean>> left_acc_;
  accumulator_set<float, stats<tag::mean>> right_front_acc_;
  accumulator_set<float, stats<tag::mean>> right_acc_;
  doogie_drivers::IRSensorDriver ir_sensor_driver;
  int num_samples = 0;

  void showResults() {
    ROS_INFO("----------------------------");
    ROS_INFO("Channel 0 mean value = %.2f", mean(left_front_acc_));
    ROS_INFO("Channel 1 mean value = %.2f", mean(left_acc_));
    ROS_INFO("Channel 2 mean value = %.2f", mean(right_front_acc_));
    ROS_INFO("Channel 3 mean value = %.2f", mean(right_acc_));
    ROS_INFO("----------------------------");
  }

  void timerCallback(const ros::TimerEvent& time) {
    int max_num_samples = 10;
    left_front_acc_(ir_sensor_driver.computeDistance(doogie_drivers::LEFT_FRONT, true));
    left_acc_(ir_sensor_driver.computeDistance(doogie_drivers::LEFT, true));
    right_front_acc_(ir_sensor_driver.computeDistance(doogie_drivers::RIGHT_FRONT, true));
    right_acc_(ir_sensor_driver.computeDistance(doogie_drivers::RIGHT, true));

    num_samples++;
    if (num_samples > max_num_samples) {
      this->showResults();
      ros::shutdown();
    }

    ROS_INFO("Sample %d/%d", num_samples, max_num_samples);
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "calibration_node");

  Calibration calibration;

  try {
    calibration.start();
  } catch (std::runtime_error error) {
    ROS_FATAL("%s", error.what());
    ros::shutdown();
  }

  return 0;
}
