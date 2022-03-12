#include <rclcpp/rclcpp.hpp>

#include "ManualControlNode.hpp"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ManualControlNode>("manual_control"));
    rclcpp::shutdown();
    return 0;
}
