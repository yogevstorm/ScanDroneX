#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <Eigen/Dense>

class KalmanOdomNode : public rclcpp::Node
{
public:
  KalmanOdomNode();

private:
  void cmdVelOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void rf2oOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void timerCallback();

  void predict(double dt);
  void update(double z_x, double z_y, double z_yaw);
  void publishState();
  static double normalizeAngle(double angle);

  // EKF state: [x, y, yaw]
  Eigen::Vector3d x_;
  Eigen::Matrix3d P_;   // state covariance
  Eigen::Matrix3d Q_;   // process noise
  Eigen::Matrix3d R_;   // LiDAR measurement noise (baseline)

  // Chi-squared threshold (3 DOF, 95th percentile) for adaptive NIS
  static constexpr double kChi2Threshold = 7.815;

  // Latest buffered messages
  nav_msgs::msg::Odometry::SharedPtr latest_cmd_vel_odom_;
  nav_msgs::msg::Odometry::SharedPtr latest_rf2o_odom_;
  bool new_rf2o_available_{false};

  rclcpp::Time last_predict_time_;
  bool initialized_{false};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr rf2o_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};
