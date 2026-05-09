#pragma once

#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/transform_broadcaster.h>

class OdomSwitchNode : public rclcpp::Node
{
public:
  OdomSwitchNode();

private:
  enum class Source { RF2O, CMD_VEL };

  void rf2oCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void cmdVelOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void cornersCallback(const std_msgs::msg::Int32::SharedPtr msg);
  void switchTo(Source next, const nav_msgs::msg::Odometry::SharedPtr & incoming);
  void publish(const nav_msgs::msg::Odometry::SharedPtr & odom);

  Source active_source_{Source::RF2O};

  nav_msgs::msg::Odometry::SharedPtr latest_rf2o_;
  nav_msgs::msg::Odometry::SharedPtr latest_cmd_vel_;

  // Offset state for seamless switching
  double base_x_{0.0}, base_y_{0.0};   // last published position at switch time
  double ref_x_{0.0},  ref_y_{0.0};    // new source's position at switch time
  double last_pub_x_{0.0}, last_pub_y_{0.0};

  double threshold_high_{1.0};
  double threshold_low_{0.0};
  double switch_delay_sec_{5.0};

  std::optional<rclcpp::Time> high_since_;
  std::optional<rclcpp::Time> low_since_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr rf2o_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr cmd_vel_odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr corners_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_switch_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr source_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};
