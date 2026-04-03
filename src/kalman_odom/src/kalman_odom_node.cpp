#include "kalman_odom/kalman_odom_node.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>

KalmanOdomNode::KalmanOdomNode()
: Node("kalman_odom_node"),
  x_(Eigen::Vector3d::Zero()),
  P_(Eigen::Matrix3d::Identity())
{
  // Process noise: tuned for drone cmd_vel integration drift
  Q_ = Eigen::Matrix3d::Zero();
  Q_(0, 0) = 0.01;   // x
  Q_(1, 1) = 0.01;   // y
  Q_(2, 2) = 0.005;  // yaw

  // LiDAR measurement noise (trusted by default; scaled up adaptively in corridors)
  R_ = Eigen::Matrix3d::Zero();
  R_(0, 0) = 0.05;   // x
  R_(1, 1) = 0.05;   // y
  R_(2, 2) = 0.01;   // yaw

  cmd_vel_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom_cmd_vel", 10,
    std::bind(&KalmanOdomNode::cmdVelOdomCallback, this, std::placeholders::_1));

  rf2o_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom_rf2o", 10,
    std::bind(&KalmanOdomNode::rf2oOdomCallback, this, std::placeholders::_1));

  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom_kalman", 10);

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(20),  // 50 Hz
    std::bind(&KalmanOdomNode::timerCallback, this));
}

void KalmanOdomNode::cmdVelOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  latest_cmd_vel_odom_ = msg;
}

void KalmanOdomNode::rf2oOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  latest_rf2o_odom_ = msg;
  new_rf2o_available_ = true;
}

void KalmanOdomNode::timerCallback()
{
  // Wait until both sources have delivered at least one message
  if (!latest_cmd_vel_odom_ || !latest_rf2o_odom_) return;

  if (!initialized_) {
    // Seed the filter from the first LiDAR pose (more trusted source)
    const auto & pose = latest_rf2o_odom_->pose.pose;
    tf2::Quaternion q(pose.orientation.x, pose.orientation.y,
                      pose.orientation.z, pose.orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    x_(0) = pose.position.x;
    x_(1) = pose.position.y;
    x_(2) = yaw;
    P_    = Eigen::Matrix3d::Identity() * 0.1;

    last_predict_time_ = this->now();
    initialized_ = true;
    return;
  }

  const auto now = this->now();
  const double dt = (now - last_predict_time_).seconds();
  last_predict_time_ = now;

  // Guard against stale or zero dt (e.g. sim pause, clock jump)
  if (dt <= 0.0 || dt > 0.5) return;

  predict(dt);

  if (new_rf2o_available_) {
    const auto & pose = latest_rf2o_odom_->pose.pose;
    tf2::Quaternion q(pose.orientation.x, pose.orientation.y,
                      pose.orientation.z, pose.orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    update(pose.position.x, pose.position.y, yaw);
    new_rf2o_available_ = false;
  }

  publishState();
}

void KalmanOdomNode::predict(double dt)
{
  const double yaw  = x_(2);
  const double vx   = latest_cmd_vel_odom_->twist.twist.linear.x;
  const double vy   = latest_cmd_vel_odom_->twist.twist.linear.y;
  const double vyaw = latest_cmd_vel_odom_->twist.twist.angular.z;

  // Nonlinear process model (body-frame velocities → world-frame displacement)
  Eigen::Vector3d x_pred;
  x_pred(0) = x_(0) + dt * (vx * std::cos(yaw) - vy * std::sin(yaw));
  x_pred(1) = x_(1) + dt * (vx * std::sin(yaw) + vy * std::cos(yaw));
  x_pred(2) = normalizeAngle(x_(2) + dt * vyaw);

  // Jacobian F = ∂f/∂x  (identity + yaw-dependent columns)
  Eigen::Matrix3d F = Eigen::Matrix3d::Identity();
  F(0, 2) = dt * (-vx * std::sin(yaw) - vy * std::cos(yaw));
  F(1, 2) = dt * ( vx * std::cos(yaw) - vy * std::sin(yaw));

  x_ = x_pred;
  P_ = F * P_ * F.transpose() + Q_;
}

void KalmanOdomNode::update(double z_x, double z_y, double z_yaw)
{
  // Measurement model: H = I₃  (LiDAR gives pose directly in same frame)
  Eigen::Vector3d z(z_x, z_y, z_yaw);
  Eigen::Vector3d innovation = z - x_;
  innovation(2) = normalizeAngle(innovation(2));

  // Innovation covariance with baseline R
  Eigen::Matrix3d S = P_ + R_;

  // --- Adaptive NIS: scale R when LiDAR becomes inconsistent (e.g. corridor) ---
  // NIS follows χ²(3); 95th-percentile threshold = 7.815
  const double NIS = innovation.transpose() * S.inverse() * innovation;
  Eigen::Matrix3d R_adaptive = R_;
  if (NIS > kChi2Threshold) {
    // Proportionally reduce LiDAR trust; cmd_vel prediction dominates instead
    R_adaptive *= (NIS / kChi2Threshold);
    S = P_ + R_adaptive;
  }

  const Eigen::Matrix3d K = P_ * S.inverse();
  x_ += K * innovation;
  x_(2) = normalizeAngle(x_(2));
  P_ = (Eigen::Matrix3d::Identity() - K) * P_;
}

void KalmanOdomNode::publishState()
{
  const auto now = this->now();

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, x_(2));

  // --- Odometry message ---
  nav_msgs::msg::Odometry odom;
  odom.header.stamp    = now;
  odom.header.frame_id = "odom_kalman";
  odom.child_frame_id  = "base_link";

  odom.pose.pose.position.x    = x_(0);
  odom.pose.pose.position.y    = x_(1);
  // z is not part of the 2-D filter state; pass through from cmd_vel source
  odom.pose.pose.position.z    = latest_cmd_vel_odom_->pose.pose.position.z;
  odom.pose.pose.orientation.x = q.x();
  odom.pose.pose.orientation.y = q.y();
  odom.pose.pose.orientation.z = q.z();
  odom.pose.pose.orientation.w = q.w();

  // Twist comes from cmd_vel (body frame; consistent with prediction model)
  odom.twist.twist = latest_cmd_vel_odom_->twist.twist;

  odom_pub_->publish(odom);

  // --- TF broadcast: odom_kalman → base_link ---
  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp    = now;
  tf.header.frame_id = "odom_kalman";
  tf.child_frame_id  = "base_link";

  tf.transform.translation.x = x_(0);
  tf.transform.translation.y = x_(1);
  tf.transform.translation.z = latest_cmd_vel_odom_->pose.pose.position.z;
  tf.transform.rotation.x    = q.x();
  tf.transform.rotation.y    = q.y();
  tf.transform.rotation.z    = q.z();
  tf.transform.rotation.w    = q.w();

  tf_broadcaster_->sendTransform(tf);
}

double KalmanOdomNode::normalizeAngle(double angle)
{
  while (angle >  M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}
