#include "cmd_vel_ekf/cmd_vel_ekf_node.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <cmath>

CmdVelEkfNode::CmdVelEkfNode()
: Node("cmd_vel_ekf_node"),
  vx_cmd_(0.0), vy_cmd_(0.0), vyaw_cmd_(0.0),
  initialized_(false)
{
  // Process noise Q: uncertainty in cmd_vel prediction model
  double q_x   = this->declare_parameter("q_x",   0.05);
  double q_y   = this->declare_parameter("q_y",   0.05);
  double q_yaw = this->declare_parameter("q_yaw", 0.02);
  Q_ = Eigen::Matrix3d::Zero();
  Q_(0, 0) = q_x;
  Q_(1, 1) = q_y;
  Q_(2, 2) = q_yaw;

  // Measurement noise R: uncertainty in rf2o pose (fallback if covariance not in message)
  double r_x   = this->declare_parameter("r_x",   0.05);
  double r_y   = this->declare_parameter("r_y",   0.05);
  double r_yaw = this->declare_parameter("r_yaw", 0.02);
  R_ = Eigen::Matrix3d::Zero();
  R_(0, 0) = r_x;
  R_(1, 1) = r_y;
  R_(2, 2) = r_yaw;

  mahal_threshold_ = this->declare_parameter("mahal_threshold", 5.0);

  x_ = Eigen::Vector3d::Zero();
  P_ = Eigen::Matrix3d::Identity();

  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/simple_drone/cmd_vel", 10,
    std::bind(&CmdVelEkfNode::cmdVelCallback, this, std::placeholders::_1));

  rf2o_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom_rf2o", 10,
    std::bind(&CmdVelEkfNode::rf2oCallback, this, std::placeholders::_1));

  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odometry/filtered", 10);

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  double freq = this->declare_parameter("frequency", 30.0);
  predict_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / freq),
    std::bind(&CmdVelEkfNode::predictStep, this));

  last_predict_time_ = this->now();
}

void CmdVelEkfNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  vx_cmd_   = msg->linear.x;
  vy_cmd_   = msg->linear.y;
  vyaw_cmd_ = msg->angular.z;
}

void CmdVelEkfNode::predictStep()
{
  if (!initialized_) return;

  auto now = this->now();
  double dt = (now - last_predict_time_).seconds();
  last_predict_time_ = now;

  if (dt <= 0.0 || dt > 1.0) return;

  double yaw  = x_(2);
  double vx = vx_cmd_;
  double vy = vy_cmd_;

  // Predict: rotate body-frame cmd_vel into world frame and integrate.
  // Yaw is not integrated here — it is updated exclusively by the rf2o measurement.
  x_(0) += (vx * std::cos(yaw) - vy * std::sin(yaw)) * dt;
  x_(1) += (vx * std::sin(yaw) + vy * std::cos(yaw)) * dt;

  // Jacobian of prediction w.r.t. state (linearized around current yaw)
  Eigen::Matrix3d F = Eigen::Matrix3d::Identity();
  F(0, 2) = -(vx * std::sin(yaw) + vy * std::cos(yaw)) * dt;
  F(1, 2) =  (vx * std::cos(yaw) - vy * std::sin(yaw)) * dt;

  // Propagate covariance (Q scaled by dt: continuous noise discretization)
  P_ = F * P_ * F.transpose() + Q_ * dt;

  publishState();
}

void CmdVelEkfNode::rf2oCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  // Extract x, y, yaw from rf2o measurement
  double z_x = msg->pose.pose.position.x;
  double z_y = msg->pose.pose.position.y;

  tf2::Quaternion q(
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w);
  double roll, pitch, z_yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, z_yaw);

  // Initialize state from first rf2o measurement
  if (!initialized_) {
    x_(0) = z_x;
    x_(1) = z_y;
    x_(2) = z_yaw;
    P_ = Eigen::Matrix3d::Identity() * 0.1;
    initialized_ = true;
    last_predict_time_ = this->now();
    return;
  }

  // Use rf2o pose covariance from message if valid, otherwise fall back to R_
  Eigen::Matrix3d R_used = R_;
  const auto & cov = msg->pose.covariance;
  if (cov[0] > 0.0 && cov[35] > 0.0) {
    R_used(0, 0) = cov[0];
    R_used(1, 1) = cov[7];
    R_used(2, 2) = cov[35];
  }

  // Innovation: z - Hx  (H = I)
  Eigen::Vector3d innov(z_x - x_(0), z_y - x_(1), normalizeAngle(z_yaw - x_(2)));

  // Innovation covariance S = P + R
  Eigen::Matrix3d S = P_ + R_used;
  Eigen::Matrix3d S_inv = S.inverse();

  // Mahalanobis distance on x,y only — yaw is always updated regardless
  Eigen::Vector2d innov_xy(innov(0), innov(1));
  Eigen::Matrix2d S_xy = S.block<2, 2>(0, 0);
  double mahal_sq = innov_xy.transpose() * S_xy.inverse() * innov_xy;

  if (mahal_sq <= mahal_threshold_) {
    // rf2o x,y consistent with cmd_vel prediction — full 3D update
    Eigen::Matrix3d K = P_ * S_inv;
    x_ = x_ + K * innov;
    P_ = (Eigen::Matrix3d::Identity() - K) * P_;
  } else {
    // Corridor degeneracy detected — skip x,y correction, update yaw only
    RCLCPP_DEBUG(get_logger(), "rf2o gated (mahal=%.2f > %.2f): yaw-only update", mahal_sq, mahal_threshold_);
    double K_yaw = P_(2, 2) / (P_(2, 2) + R_used(2, 2));
    x_(2) += K_yaw * innov(2);
    P_(2, 2) *= (1.0 - K_yaw);
  }

  x_(2) = normalizeAngle(x_(2));

  publishState();
}

void CmdVelEkfNode::publishState()
{
  auto now = this->now();

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, x_(2));

  nav_msgs::msg::Odometry odom;
  odom.header.stamp    = now;
  odom.header.frame_id = "ekf_odom";
  odom.child_frame_id  = "simple_drone/base_footprint";

  odom.pose.pose.position.x    = x_(0);
  odom.pose.pose.position.y    = x_(1);
  odom.pose.pose.position.z    = 0.0;
  odom.pose.pose.orientation.x = q.x();
  odom.pose.pose.orientation.y = q.y();
  odom.pose.pose.orientation.z = q.z();
  odom.pose.pose.orientation.w = q.w();

  odom.pose.covariance[0]  = P_(0, 0);
  odom.pose.covariance[7]  = P_(1, 1);
  odom.pose.covariance[35] = P_(2, 2);

  odom.twist.twist.linear.x  = vx_cmd_;
  odom.twist.twist.linear.y  = vy_cmd_;
  odom.twist.twist.angular.z = vyaw_cmd_;

  odom_pub_->publish(odom);

  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp    = now;
  tf.header.frame_id = "ekf_odom";
  tf.child_frame_id  = "simple_drone/base_footprint";

  tf.transform.translation.x = x_(0);
  tf.transform.translation.y = x_(1);
  tf.transform.translation.z = 0.0;
  tf.transform.rotation.x    = q.x();
  tf.transform.rotation.y    = q.y();
  tf.transform.rotation.z    = q.z();
  tf.transform.rotation.w    = q.w();

  tf_broadcaster_->sendTransform(tf);
}

double CmdVelEkfNode::normalizeAngle(double angle)
{
  while (angle >  M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelEkfNode>());
  rclcpp::shutdown();
  return 0;
}
