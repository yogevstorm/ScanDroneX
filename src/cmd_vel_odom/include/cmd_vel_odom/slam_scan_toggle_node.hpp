#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>

class SlamScanToggleNode : public rclcpp::Node
{
public:
  SlamScanToggleNode();

private:
  void onUsingCmdVel(const std_msgs::msg::Bool::SharedPtr msg);
  void setScanMatching(bool enabled);

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_;
  rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr client_;
  bool last_using_cmd_vel_{false};
  bool initialized_{false};
};
