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
// ros2 topic pub /fspid_target std_msgs/Float32 'data: 45'


#include"fuzzy_controller/fspid_component.hpp"


FuzzyPIDController::FuzzyPIDController() : Node("fuzzy_pid_controller")
{
    // Publishers and Subscribers
    publisher_ = this->create_publisher<std_msgs::msg::Float32>("/fuzzy/fspid", 10);
    subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu/data",
        10,
        std::bind(&FuzzyPIDController::imu_callback, this, std::placeholders::_1));
    target_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
        "/fspid_target",
        10,
        std::bind(&FuzzyPIDController::target_callback, this, std::placeholders::_1));

    // Initialize parameters
    target_yaw_ = 0.0;
    current_yaw_ = 0.0;
    previous_error_ = 0.0;
    integral_error_ = 0.0;
    dt_ = 0.01; // 10ms sampling time
    kp_base_ = 1.0;
    ki_base_ = 0.5;
    kd_base_ = 0.2;
    kp_ = kp_base_;
    ki_ = ki_base_;
    kd_ = kd_base_;
    dkp_max_ = 1.0;
    dki_max_ = 0.5;
    dkd_max_ = 0.2;
    kp_max_ = 5.0;
    ki_max_ = 1.0;
    kd_max_ = 0.5;
    // Fixed scaling factors for ±45° maximum error design
    error_scale_factor_ = 45.0;
    // Set membership function type
    membership_type_ = TRIANGULAR;
    // membership_type_ = GAUSSIAN;
    // membership_type = TRAPEZOIDAL;

    // Initialize adaptive scaling parameters
    max_expected_error_ = 180.0; // Maximum expected yaw error in degrees
    error_scale_factor_ = 1.0;
    derror_scale_factor_ = 1.0;

    // Initialize fuzzy system
    initializeFuzzySystem();
}
std::string FuzzyPIDController::getMembershipTypeName()
{
    switch(membership_type_) {
        case TRIANGULAR: return "Triangular";
        case GAUSSIAN: return "Gaussian";
        case TRAPEZOIDAL: return "Trapezoidal";
        default: return "Unknown";
    }
}

void FuzzyPIDController::initializeFuzzySystem()
{
    double sv_ = PB;  /* Scaling Value */
    // Define normalized membership function centers for error (NB, NM, NS, ZO, PS, PM, PB)
    error_mf_normalized_ = {NB/sv_, NM/sv_, NS/sv_, ZO/sv_, PS/sv_, PM/sv_, PB/sv_};
    // Define normalized membership function centers for error derivative
    derror_mf_normalized_ = {NB/sv_, NM/sv_, NS/sv_, ZO/sv_, PS/sv_, PM/sv_, PB/sv_};

    // ΔKp rules (adjustments to base Kp) - Maximum gain adjustment for ±45° error
    delta_kp_rules_ = {
        {PB,PB,PM,PM,PS,ZO,ZO},
        {PB,PB,PM,PS,PS,ZO,NS},
        {PM,PM,PM,PS,ZO,NS,NS},
        {PM,PM,PS,ZO,NS,NM,NM},
        {PS,PS,ZO,NS,NS,NM,NM},
        {PS,ZO,NS,NM,NM,NM,NB},
        {ZO,ZO,NM,NM,NM,NB,NB}
    };
    // ΔKi rules (adjustments to base Ki) - Conservative integral gain
    delta_ki_rules_ = {
        {NB,NB,NM,NM,NS,ZO,ZO},
        {NB,NB,NM,NS,NS,ZO,ZO},
        {NB,NM,NS,NS,ZO,PS,PS},
        {NM,NM,NS,ZO,PS,PM,PM},
        {NM,NS,ZO,PS,PS,PM,PB},
        {ZO,ZO,PS,PS,PM,PB,PB},
        {ZO,ZO,PS,PM,PM,PB,PB}
    };
    // ΔKd rules (adjustments to base Kd) - Strong derivative action for stability
    delta_kd_rules_ = {
        {PS,NS,NB,NB,NB,NM,PS},
        {PS,NS,NB,NM,NM,NS,ZO},
        {ZO,NS,NM,NM,NS,NS,ZO},
        {ZO,NS,NS,NS,NS,NS,ZO},
        {ZO,ZO,ZO,ZO,ZO,ZO,ZO},
        {PB,NS,PS,PS,PS,PS,PB},
        {PB,PM,PM,PM,PS,PS,PB}
    };

    delta_kp_rules_ = normalizeFuzzyRules(delta_kp_rules_, dkp_max_, sv_);
    delta_ki_rules_ = normalizeFuzzyRules(delta_ki_rules_, dki_max_, sv_);
    delta_kd_rules_ = normalizeFuzzyRules(delta_kd_rules_, dkd_max_, sv_);

}

std::vector<std::vector<double>> FuzzyPIDController::normalizeFuzzyRules(
    const std::vector<std::vector<double>>& fuzzy_rules,
    double range_limit,
    double max_fuzzy_value
) {
    std::vector<std::vector<double>> normalized;
    normalized.resize(fuzzy_rules.size());

    for (size_t i = 0; i < fuzzy_rules.size(); ++i) {
        normalized[i].resize(fuzzy_rules[i].size());
        for (size_t j = 0; j < fuzzy_rules[i].size(); ++j) {
            int fuzzy_value = static_cast<int>(fuzzy_rules[i][j]);
            normalized[i][j] = (fuzzy_value / max_fuzzy_value) * range_limit;
        }
    }
    return normalized;
}

// ----- Membership Function of Triangular -----
double FuzzyPIDController::trimf(double x, double a, double b, double c)
{
    double u;
    /* μ(x) = max(0, 1 - |x-c|/w) */
    if(x >= a && x <= b)
        u = (x - a) / (b - a);
    else if(x > b && x <= c)
        u = (c - x) / (c - b);
    else
        u = 0.0;
    return u;
}

// ----- Membership Function of Gaussian -----
double FuzzyPIDController::gaussmf(double x, double ave, double sigma)
{
    if(sigma <= 0) {
        RCLCPP_ERROR(this->get_logger(), "In gaussmf, sigma must be larger than 0");
        return 0.0;
    }
    /* μ(x) = exp(-0.5 * ((x-c)/σ)²) */
    return exp(-pow((x - ave) / sigma, 2));
}

// ----- Membership Function of Trapezoidal -----
double FuzzyPIDController::trapmf(double x, double a, double b, double c, double d)
{
    double u;
    if(x >= a && x < b)
        u = (x - a) / (b - a);
    else if(x >= b && x < c)
        u = 1.0;
    else if(x >= c && x <= d)
        u = (d - x) / (d - c);
    else
        u = 0.0;
    return u;
}

double FuzzyPIDController::getMembershipValue(double x, double center, MembershipType type)
{
    switch(type) {
        case TRIANGULAR:
            return trimf(x, center - 0.4, center, center + 0.4);
        case GAUSSIAN:
            return gaussmf(x, center, 0.3);
        case TRAPEZOIDAL:
            return trapmf(x, center - 0.4, center - 0.2, center + 0.2, center + 0.4);
        default:
            return 0.0;
    }
}

std::vector<double> FuzzyPIDController::fuzzyInference(double error, double derror)
{
    // Normalize inputs to [-1, 1] range using adaptive scaling
    double normalized_error = error / error_scale_factor_;
    double normalized_derror = derror / error_scale_factor_;

    // Clip to [-1, 1] range
    normalized_error = std::max(-1.0, std::min(1.0, normalized_error));
    normalized_derror = std::max(-1.0, std::min(1.0, normalized_derror));

    double total_weight = 0.0;
    double weighted_delta_kp = 0.0;
    double weighted_delta_ki = 0.0;
    double weighted_delta_kd = 0.0;

    // Fuzzification and Inference
    for (size_t i = 0; i < error_mf_normalized_.size(); ++i) {
        for (size_t j = 0; j < derror_mf_normalized_.size(); ++j) {
            double mu_error = getMembershipValue(normalized_error, error_mf_normalized_[i], membership_type_);
            double mu_derror = getMembershipValue(normalized_derror, derror_mf_normalized_[j], membership_type_);

            // Rule activation strength (Mamdani Inference)
            double weight = mu_error * mu_derror;

            if (weight > 0.0) {
                total_weight += weight;

                // Remove error magnitude scaling - use fixed scaling for ±45° design
                weighted_delta_kp += weight * delta_kp_rules_[i][j];
                weighted_delta_ki += weight * delta_ki_rules_[i][j];
                weighted_delta_kd += weight * delta_kd_rules_[i][j];
            }
        }
    }

    /* ---------- Defuzzification (Center of Gravity method) ---------- */
    std::vector<double> delta_gains(3);
    if (total_weight > 0.0) {
        delta_gains[0] = weighted_delta_kp / total_weight;  // ΔKp
        delta_gains[1] = weighted_delta_ki / total_weight;  // ΔKi
        delta_gains[2] = weighted_delta_kd / total_weight;  // ΔKd
    } else {
        delta_gains[0] = 0.0;
        delta_gains[1] = 0.0;
        delta_gains[2] = 0.0;
    }

    return delta_gains;
}

double FuzzyPIDController::normalizeAngle(double angle)
{
    while (angle > max_expected_error_) angle -= 2*max_expected_error_;
    while (angle < -max_expected_error_) angle += 2*max_expected_error_;
    return angle;
}

void FuzzyPIDController::imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
{
    double q0 = imu_msg->orientation.w;
    double q1 = imu_msg->orientation.x;
    double q2 = imu_msg->orientation.y;
    double q3 = imu_msg->orientation.z;
    double roll = atan2(2.0 * (q0*q1 + q2*q3), 1.0 - 2.0 * (q1*q1 + q2*q2))*180 / M_PI;
    double pitch = asin(2.0 * (q0*q2 - q3*q1))*180 / M_PI;
    double yaw = atan2(2.0 * (q0*q3 + q1*q2), 1.0 - 2.0 * (q2*q2 + q3*q3))*180 / M_PI;
    // std::cout << "Roll: " << roll << " Pitch: " << pitch << " Yaw: " << yaw << std::endl;

    current_pitch_ = pitch;
    current_yaw_ = yaw;

    // Calculate error (in degrees)
    double error = normalizeAngle(target_yaw_ - current_yaw_);
    double derror = (error - previous_error_) / dt_;

    // Fuzzy inference to adapt PID gains
    std::vector<double> delta_gains = fuzzyInference(error, derror);

    // Update adaptive gains:
    /* K_p = K_p^{\prime} + \Delta K_p */
    /* K_i = K_i^{\prime} + \Delta K_i */
    /* K_d = K_d^{\prime} + \Delta K_d */
    kp_ = kp_base_ + delta_gains[0];
    ki_ = ki_base_ + delta_gains[1];
    kd_ = kd_base_ + delta_gains[2];

    // Ensure gains remain positive
    // kp_ = std::max(kp_max_, kp_);
    // ki_ = std::max(ki_max_, ki_);
    // kd_ = std::max(kd_max_, kd_);

    // Update integral error
    integral_error_ += error * dt_;

    // Anti-windup with adaptive limits
    double integral_limit = std::max(1.0, std::abs(error) / 10.0);
    integral_error_ = std::max(-integral_limit, std::min(integral_limit, integral_error_));

    // Calculate PID output
    double proportional = kp_ * error;
    double integral = ki_ * integral_error_;
    double derivative = kd_ * derror;

    double control_output = proportional + integral + derivative;

    // Adaptive output saturation - maximum output at ±45° error
    double normalized_error_for_output = std::abs(error) / 45.0;
    double output_limit = std::min(1.0, normalized_error_for_output);
    control_output = std::max(-output_limit, std::min(output_limit, control_output));

    std::cout << "Target: " << target_yaw_ << " Current: " << current_yaw_
              << " Error: " << error << " Output: " << control_output
              << " Kp, Kd, Ki: " << kp_ << ", " << ki_ << ", " << kd_ << ", " << std::endl;

    // Publish control output
    auto output_msg = std_msgs::msg::Float32();
    output_msg.data = static_cast<float>(control_output);
    publisher_->publish(output_msg);

    // Update previous error
    previous_error_ = error;
}

void FuzzyPIDController::target_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    target_yaw_ = static_cast<double>(msg->data);
    integral_error_ = 0.0; // Reset integral when target changes
    previous_error_ = 0.0; // Reset previous error
}
