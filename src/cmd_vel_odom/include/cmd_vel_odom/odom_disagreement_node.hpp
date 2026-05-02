#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32.hpp>
#include <deque>

class OdomDisagreementNode : public rclcpp::Node
{
public:
  OdomDisagreementNode();

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void rf2oCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void compute();

  double cmd_vx_{0.0}, cmd_vy_{0.0}, cmd_wz_{0.0};
  double rf2o_vx_{0.0}, rf2o_vy_{0.0}, rf2o_wz_{0.0};
  bool cmd_received_{false}, rf2o_received_{false};
  double min_cmd_vx_{0.2};

  static constexpr size_t WINDOW = 20;
  std::deque<float> linear_window_;
  std::deque<float> angular_window_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr rf2o_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr linear_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr angular_pub_;
};
