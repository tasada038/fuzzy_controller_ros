

#ifndef FUZZY_CONTROLLER__TS_FUZZY_CONTROLLER_COMPONENT_HPP_
#define FUZZY_CONTROLLER__TS_FUZZY_CONTROLLER_COMPONENT_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/float32.hpp"

class FuzzyControllerComponent : public rclcpp::Node
{
  public:
    FuzzyControllerComponent(const rclcpp::NodeOptions & options);

  private:
    void timer_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);
    double short_distance(double d);
    double medium_distance(double d);
    double long_distance(double d);
    double apply_rule1(double short_distance);
    double apply_rule2(double short_distance);
    double apply_rule3(double short_distance);
    double defuzzify(double rule1_output, double rule2_output, double rule3_output);
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;

    const double INPUT_MAX = 1.0;
    const double INPUT_MIN = 0.0;
    double dist_range_;
    double dist_min_, dist_mid_, dist_max_;
    double weight_1_, weight_2_, weight_3_;
};

#endif // FUZZY_CONTROLLER__TS_FUZZY_CONTROLLER_COMPONENT_HPP_