#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/transform_broadcaster.h>

class OdomSwitchNode : public rclcpp::Node
{
public:
  OdomSwitchNode();

private:
  void rf2oCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void cmdVelOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void disagreementCallback(const std_msgs::msg::Float32::SharedPtr msg);
  void publish(const nav_msgs::msg::Odometry::SharedPtr & odom);

  enum class Source { RF2O, CMD_VEL };
  Source active_source_{Source::RF2O};

  nav_msgs::msg::Odometry::SharedPtr latest_rf2o_;
  nav_msgs::msg::Odometry::SharedPtr latest_cmd_vel_;

  double threshold_high_{0.8};
  double threshold_low_{0.2};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr rf2o_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr cmd_vel_odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr disagreement_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_switch_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr source_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};
