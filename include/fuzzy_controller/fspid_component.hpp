//-----------------------------------------------------------------------------------
// MIT License

// Copyright (c) 2025 Takumi Asada

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//-----------------------------------------------------------------------------------


#ifndef FUZZY_CONTROLLER__FSPID_COMPONENT_HPP__
#define FUZZY_CONTROLLER__FSPID_COMPONENT_HPP__

#include<iostream>
#include<cmath>
#include<string>

#include <Eigen/Core>

// headers in ros2
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/imu.hpp"

#define N_ 7

#define NB -3
#define NM -2
#define NS -1
#define ZO 0
#define PS 1
#define PM 2
#define PB 3


class FuzzyPIDController : public rclcpp::Node
{
public:
    enum MembershipType {
        TRIANGULAR,
        GAUSSIAN,
        TRAPEZOIDAL
    };
    FuzzyPIDController();

private:
    // ROS2 components
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr target_subscriber_;

    // Control variables
    double target_yaw_;
    double current_yaw_;
    double previous_error_;
    double integral_error_;
    double dt_;
    
    // Base and Current PID gains (initial values)
    double kp_base_, ki_base_, kd_base_;
    double kp_, ki_, kd_;
    double dkp_max_, dki_max_, dkd_max_;
    double kp_max_, ki_max_, kd_max_;

    // Membership function type
    MembershipType membership_type_;
    
    // Adaptive scaling parameters
    double max_expected_error_;
    double error_scale_factor_;
    double derror_scale_factor_;
    
    // Fuzzy system parameters
    std::vector<double> error_mf_normalized_, derror_mf_normalized_;
    std::vector<std::vector<double>> delta_kp_rules_, delta_ki_rules_, delta_kd_rules_;
    
    std::string getMembershipTypeName();

    void initializeFuzzySystem();
    std::vector<std::vector<double>> normalizeFuzzyRules(
      const std::vector<std::vector<double>>& fuzzy_rules,
      double range_limit,
      double max_fuzzy_value
    );
    // Triangular membership function
    double trimf(double x, double a, double b, double c);
    // Gaussian membership function
    double gaussmf(double x, double ave, double sigma);
    // Trapezoidal membership function
    double trapmf(double x, double a, double b, double c, double d);
    double getMembershipValue(double x, double center, MembershipType type);
    std::vector<double> fuzzyInference(double error, double derror);

    double normalizeAngle(double angle);
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu_msg);
    void target_callback(const std_msgs::msg::Float32::SharedPtr msg);
};

#endif //FUZZY_CONTROLLER__FSPID_COMPONENT_HPP__