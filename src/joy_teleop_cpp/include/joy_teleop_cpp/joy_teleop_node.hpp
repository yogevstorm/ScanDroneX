#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/empty.hpp>

class JoyTeleopNode : public rclcpp::Node
{
public:
  JoyTeleopNode();

private:
  static constexpr double MAX_LINEAR  = 2.0;
  static constexpr double MAX_ANGULAR = 3.0;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr      takeoff_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr      land_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr  joy_sub_;

  bool prev_btn_takeoff_;
  bool prev_btn_land_;

  static double scale(double axis, double max);
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
};
