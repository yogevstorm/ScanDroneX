#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

class CmdVelOdomNode : public rclcpp::Node
{
public:
  CmdVelOdomNode();

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

  double x_, y_, z_, yaw_;
  rclcpp::Time last_time_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
};
