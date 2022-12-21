

#include "rclcpp/rclcpp.hpp"
#include "fuzzy_controller/ts_fuzzy_controller_component.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    auto component = std::make_shared<FuzzyControllerComponent>(options);
    rclcpp::spin(component);
    rclcpp::shutdown();
    return 0;
}