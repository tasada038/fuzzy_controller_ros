// headers in ros2
#include "rclcpp/rclcpp.hpp"
// headers in this package
#include"fuzzy_controller/fspid_component.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FuzzyPIDController>());
    rclcpp::shutdown();
    return 0;
}