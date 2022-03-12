/**
 * @file ManualControlNode.hpp
 * @author ottojo
 * @date 9.11.2021
 */

#ifndef MANUALCONTROL_MANUALCONTROLNODE_HPP
#define MANUALCONTROL_MANUALCONTROLNODE_HPP

#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <spatz_interfaces/msg/control_setpoint.hpp>

/**
 * @brief Node that publishes spatz_interfaces::msg::ControlSetpoint from joystick (gamepad) input for testing purposes
 * @ingroup ROSNodes
 * @details Subscribes to "/joy" with type sensor_msgs/Joy, as provided by the "joy" package.
 *          Publishes setpoints at "/control_setpoint".
 */
class ManualControlNode : public rclcpp::Node {
  public:
    explicit ManualControlNode(const std::string &name);

  private:
    rclcpp::Publisher<spatz_interfaces::msg::ControlSetpoint>::SharedPtr controlSetpointPublisher;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joystickSubscription;

    void joystickCallback(const sensor_msgs::msg::Joy::ConstSharedPtr &joystickMessage);
};

#endif // MANUALCONTROL_MANUALCONTROLNODE_HPP
