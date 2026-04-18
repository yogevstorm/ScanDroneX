#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <Eigen/Dense>

class CmdVelEkfNode : public rclcpp::Node
{
public:
  CmdVelEkfNode();

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void rf2oCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void predictStep();
  void publishState();
  double normalizeAngle(double angle);

  // State: [x, y, yaw]
  Eigen::Vector3d x_;
  Eigen::Matrix3d P_;
  Eigen::Matrix3d Q_;
  Eigen::Matrix3d R_;

  double vx_cmd_, vy_cmd_, vyaw_cmd_;
  double mahal_threshold_;
  bool initialized_;
  rclcpp::Time last_predict_time_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr rf2o_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::TimerBase::SharedPtr predict_timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};
