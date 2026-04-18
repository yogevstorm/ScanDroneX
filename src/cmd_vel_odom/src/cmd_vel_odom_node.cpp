#include "cmd_vel_odom/cmd_vel_odom_node.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <cmath>

CmdVelOdomNode::CmdVelOdomNode()
: Node("cmd_vel_odom_node"), x_(0.0), y_(0.0), z_(0.0), yaw_(0.0)
{
  last_time_ = this->now();

  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/simple_drone/cmd_vel", 10,
    std::bind(&CmdVelOdomNode::cmdVelCallback, this, std::placeholders::_1));

  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom_cmd_vel", 10);
}

void CmdVelOdomNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  auto current_time = this->now();
  double dt = (current_time - last_time_).seconds();
  last_time_ = current_time;

  // Rotate body-frame velocities into the world frame using current yaw,
  // then integrate to get world-frame position
  x_   += (msg->linear.x * std::cos(yaw_) - msg->linear.y * std::sin(yaw_)) * dt;
  y_   += (msg->linear.x * std::sin(yaw_) + msg->linear.y * std::cos(yaw_)) * dt;
  z_   += msg->linear.z * dt;
  yaw_ += msg->angular.z * dt;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw_);

  // Publish odometry message
  nav_msgs::msg::Odometry odom;
  odom.header.stamp    = current_time;
  odom.header.frame_id = "odom_cmd_vel";
  odom.child_frame_id  = "simple_drone/base_footprint";

  odom.pose.pose.position.x    = x_;
  odom.pose.pose.position.y    = y_;
  odom.pose.pose.position.z    = z_;
  odom.pose.pose.orientation.x = q.x();
  odom.pose.pose.orientation.y = q.y();
  odom.pose.pose.orientation.z = q.z();
  odom.pose.pose.orientation.w = q.w();

  odom.twist.twist = *msg;

  // Twist covariance: higher than RF2O (vx=0.1) so EKF trusts this source less.
  // Diagonal: vx=0.3, vy=0.3, vz=1e6(unused), vroll=1e6, vpitch=1e6, vyaw=1e6(not fused)
  odom.twist.covariance[0]  = 0.3;
  odom.twist.covariance[7]  = 0.3;
  odom.twist.covariance[14] = 1e6;
  odom.twist.covariance[21] = 1e6;
  odom.twist.covariance[28] = 1e6;
  odom.twist.covariance[35] = 1e6;

  odom_pub_->publish(odom);
}
