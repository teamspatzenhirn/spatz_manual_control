/**
 * @file ManualControlNode.cpp
 * @author ottojo
 * @date 9.11.2021
 */

#include "ManualControlNode.hpp"

static constexpr double degToRad(double deg) {
    return deg * M_PI / 180;
}

ManualControlNode::ManualControlNode(const std::string &name) :
    rclcpp::Node(name),
    controlSetpointPublisher(
            create_publisher<spatz_interfaces::msg::ControlSetpoint>("/control_setpoint", rclcpp::QoS(1).reliable())),
    joystickSubscription(create_subscription<sensor_msgs::msg::Joy>(
            "/joy", rclcpp::QoS(1).best_effort(),
            [&](sensor_msgs::msg::Joy::ConstSharedPtr msg) { joystickCallback(msg); })) {
    declare_parameter<int>("joystickSteerAxis", 3);
    declare_parameter<int>("joystickAccelerateAxis", 5);
    declare_parameter<int>("joystickBrakeAxis", 2);
    declare_parameter<int>("joystickSteeringConfig", 0);
    declare_parameter<double>("maxVelms", 3);
    declare_parameter<double>("maxSteerDeg", 30);
}

void ManualControlNode::joystickCallback(const sensor_msgs::msg::Joy::ConstSharedPtr &joystickMessage) {
    double steer = get_parameter("maxSteerDeg").as_double() *
                   joystickMessage->axes.at(get_parameter("joystickSteerAxis").as_int());
    double accel = (joystickMessage->axes.at(get_parameter("joystickAccelerateAxis").as_int()) + 1) * 0.5;
    double brake = (joystickMessage->axes.at(get_parameter("joystickBrakeAxis").as_int()) + 1) * 0.5;
    double steerSetting = joystickMessage->axes.at(get_parameter("joystickSteeringConfig").as_int());
    double vel = (accel - brake) * get_parameter("maxVelms").as_double();

    std_msgs::msg::Header h;
    h.frame_id = "spatz";
    h.stamp = joystickMessage->header.stamp;
    spatz_interfaces::msg::ControlSetpoint msg = spatz_interfaces::build<spatz_interfaces::msg::ControlSetpoint>()
                                                         .header(h)
                                                         .vel(vel)
                                                         .delta_front(degToRad(steer))
                                                         .delta_rear(degToRad(steer * steerSetting));
    RCLCPP_INFO(get_logger(), "Setpoint: %f° front, %f° rear, %fm/s", msg.delta_front, msg.delta_rear, msg.vel);
    controlSetpointPublisher->publish(msg);
}
