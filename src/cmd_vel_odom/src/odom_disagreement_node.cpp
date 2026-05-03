#include "cmd_vel_odom/odom_disagreement_node.hpp"
#include <cmath>
#include <numeric>

static double yaw_from_quat(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

OdomDisagreementNode::OdomDisagreementNode()
: Node("odom_disagreement_node")
{
  min_cmd_vx_ = this->declare_parameter<double>("min_cmd_vx", 0.2);

  cmd_vel_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom_cmd_vel", 10,
    std::bind(&OdomDisagreementNode::cmdVelOdomCallback, this, std::placeholders::_1));

  rf2o_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom_rf2o", 10,
    std::bind(&OdomDisagreementNode::rf2oCallback, this, std::placeholders::_1));

  linear_pub_  = this->create_publisher<std_msgs::msg::Float32>("/odom_disagreement/linear",  10);
  angular_pub_ = this->create_publisher<std_msgs::msg::Float32>("/odom_disagreement/angular", 10);
}

void OdomDisagreementNode::cmdVelOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  const double x   = msg->pose.pose.position.x;
  const double y   = msg->pose.pose.position.y;
  const double yaw = yaw_from_quat(msg->pose.pose.orientation);
  const rclcpp::Time stamp = msg->header.stamp;

  if (!has_prev_cmd_) {
    prev_cmd_x_    = x;
    prev_cmd_y_    = y;
    prev_cmd_yaw_  = yaw;
    prev_cmd_stamp_ = stamp;
    has_prev_cmd_  = true;
    return;
  }

  const double dt = (stamp - prev_cmd_stamp_).seconds();
  if (dt <= 0.0) {
    return;
  }

  // World-frame velocity from finite difference
  const double vx_world = (x   - prev_cmd_x_)   / dt;
  const double vy_world = (y   - prev_cmd_y_)    / dt;

  // Yaw rate — normalize dyaw to [-pi, pi] before dividing
  double dyaw = yaw - prev_cmd_yaw_;
  if (dyaw >  M_PI) dyaw -= 2.0 * M_PI;
  if (dyaw < -M_PI) dyaw += 2.0 * M_PI;

  prev_cmd_x_    = x;
  prev_cmd_y_    = y;
  prev_cmd_yaw_  = yaw;
  prev_cmd_stamp_ = stamp;

  // Rotate world-frame velocity into body frame (transpose of rotation matrix)
  cmd_vx_ =  vx_world * std::cos(yaw) + vy_world * std::sin(yaw);
  cmd_vy_ = -vx_world * std::sin(yaw) + vy_world * std::cos(yaw);
  cmd_wz_ = dyaw / dt;

  cmd_received_ = true;
  compute();
}

void OdomDisagreementNode::rf2oCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  rf2o_vx_ = msg->twist.twist.linear.x;
  rf2o_vy_ = msg->twist.twist.linear.y;
  rf2o_wz_ = msg->twist.twist.angular.z;
  rf2o_received_ = true;
  compute();
}

static float window_average(std::deque<float> & window, float new_val, size_t max_size)
{
  window.push_back(new_val);
  if (window.size() > max_size) {
    window.pop_front();
  }
  return std::accumulate(window.begin(), window.end(), 0.0f) / static_cast<float>(window.size());
}

void OdomDisagreementNode::compute()
{
  if (!cmd_received_ || !rf2o_received_) {
    return;
  }

  if (std::abs(cmd_vx_) < min_cmd_vx_) {
    std_msgs::msg::Float32 zero;
    zero.data = 0.0f;
    linear_pub_->publish(zero);
    angular_pub_->publish(zero);
    return;
  }

  float linear_raw  = std::sqrt(std::pow(cmd_vx_ - rf2o_vx_, 2) + std::pow(cmd_vy_ - rf2o_vy_, 2));
  float angular_raw = std::abs(cmd_wz_ - rf2o_wz_);

  float linear_avg  = window_average(linear_window_,  linear_raw,  WINDOW);
  float angular_avg = window_average(angular_window_, angular_raw, WINDOW);

  std_msgs::msg::Float32 lin_msg, ang_msg;
  lin_msg.data = linear_avg;
  ang_msg.data = angular_avg;

  linear_pub_->publish(lin_msg);
  angular_pub_->publish(ang_msg);

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
    "[odom_disagreement] linear=%.4f m/s  angular=%.4f rad/s  (window=%zu)",
    linear_avg, angular_avg, linear_window_.size());
}
