#include <iostream>
#include <algorithm>
#include <vector>
#include <cmath>

#include "fuzzy_controller/ts_fuzzy_controller_component.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

FuzzyControllerComponent::FuzzyControllerComponent(const rclcpp::NodeOptions & options)
: Node("ts_fuzzy_node", options)
{

    declare_parameter("dist_range", 4.0);
    get_parameter("dist_range", dist_range_);
    declare_parameter("dist_min", dist_range_/4);
    get_parameter("dist_min", dist_min_);
    declare_parameter("dist_mid", dist_range_/2);
    get_parameter("dist_mid", dist_mid_);
    declare_parameter("dist_max", dist_range_*3/4);
    get_parameter("dist_max", dist_max_);
    declare_parameter("weight_1", 0.0);
    get_parameter("weight_1", weight_1_);
    declare_parameter("weight_2", 1.0);
    get_parameter("weight_2", weight_2_);
    declare_parameter("weight_3", 2.0);
    get_parameter("weight_3", weight_3_);

    auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());

    subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", 
    default_qos, 
    std::bind(&FuzzyControllerComponent::timer_callback, this, _1)\
    );

    publisher_ = this->create_publisher<std_msgs::msg::Float32>("/defuzzy_input", 10);
}

void FuzzyControllerComponent::timer_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
{
  double d = scan_msg->ranges[(scan_msg->ranges.size())/2];
  // debag
  RCLCPP_INFO(this->get_logger(), "distance: %f",d);

  // Call 3 membership functions
  double short_distance = this->short_distance(d);
  double medium_distance = this->medium_distance(d);
  double long_distance = this->long_distance(d);

  // Apply 3 fuzzy rules
  double rule1 = this->apply_rule1(short_distance);
  double rule2 = this->apply_rule2(medium_distance);
  double rule3 = this->apply_rule3(long_distance);

  // perform defuzzification
  double defuzzy = this->defuzzify(rule1, rule2, rule3);
  RCLCPP_INFO(this->get_logger(), "defuzzy_input: %f",defuzzy);

  std_msgs::msg::Float32 defuzzy_msg;
  defuzzy_msg.data = float(defuzzy);
  publisher_->publish(defuzzy_msg);
}

double FuzzyControllerComponent::short_distance(double d) {
  if (d < dist_min_) {
    return INPUT_MAX;
  } else if (d < dist_mid_) {
    return dist_mid_ - d;
  } else {
    return INPUT_MIN;
  }
}

double FuzzyControllerComponent::medium_distance(double d) {
  if (d < dist_min_ || d > dist_max_) {
    return INPUT_MIN;
  } if (d < dist_mid_) {
    return d - dist_min_;
  } else { // d < dist_max
    return dist_max_ - d;
  }
}

double FuzzyControllerComponent::long_distance(double d) {
  if (d < dist_mid_) {
    return INPUT_MIN;
  } else if (d < dist_max_) {
    return d - dist_mid_;
  } else {
    return INPUT_MAX;
  }
}

double FuzzyControllerComponent::apply_rule1(double short_distance) {
  return short_distance;
}

double FuzzyControllerComponent::apply_rule2(double medium_distance) {
  return medium_distance;
}

double FuzzyControllerComponent::apply_rule3(double long_distance) {
  return long_distance;
}

double FuzzyControllerComponent::defuzzify(double rule1_output, double rule2_output, double rule3_output) {
  // Define weight for each rule
  double w1 = weight_1_;
  double w2 = weight_2_;
  double w3 = weight_3_;

  double numerator = rule1_output * w1 + rule2_output * w2 + rule3_output * w3;
  double denominator = rule1_output + rule2_output + rule3_output;
  double defuzzify_output = numerator / denominator;

  return defuzzify_output/w3;
}