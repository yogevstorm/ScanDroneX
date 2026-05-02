#include "cmd_vel_odom/odom_disagreement_node.hpp"
#include <cmath>
#include <numeric>

OdomDisagreementNode::OdomDisagreementNode()
: Node("odom_disagreement_node")
{
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/simple_drone/cmd_vel", 10,
    std::bind(&OdomDisagreementNode::cmdVelCallback, this, std::placeholders::_1));

  rf2o_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom_rf2o", 10,
    std::bind(&OdomDisagreementNode::rf2oCallback, this, std::placeholders::_1));

  min_cmd_vx_ = this->declare_parameter<double>("min_cmd_vx", 0.2);

  linear_pub_  = this->create_publisher<std_msgs::msg::Float32>("/odom_disagreement/linear",  10);
  angular_pub_ = this->create_publisher<std_msgs::msg::Float32>("/odom_disagreement/angular", 10);
}

void OdomDisagreementNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  cmd_vx_ = msg->linear.x;
  cmd_vy_ = msg->linear.y;
  cmd_wz_ = msg->angular.z;
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

  float linear_raw  = std::sqrt(std::pow(cmd_vx_ - rf2o_vx_, 2) +
                                 std::pow(cmd_vy_ - rf2o_vy_, 2));
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
