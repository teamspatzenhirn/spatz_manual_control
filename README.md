# Manual Control Node

This package provides a ROS node for controlling Spatz using a gamepad or similar "joystick-like" input device.

It subscribes to `/joy` with type
[`sensor_msgs/Joy`](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/Joy.msg) and publishes
[`spatz_interfaces/ControlSetpoint`](https://github.com/teamspatzenhirn/spatz_interfaces/blob/master/msg/ControlSetpoint.msg)
on `/control_setpoint`.

Joystick configuration and steering/velocity limits can be adjusted using parameters.

Use the `joy_node` from the [`joy` package](https://index.ros.org/p/joy/github-ros-drivers-joystick_drivers/) as a
driver for your joystick that publishes on `/joy`.
