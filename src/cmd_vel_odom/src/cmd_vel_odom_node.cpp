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

  rf2o_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom_rf2o", 10,
    std::bind(&CmdVelOdomNode::rf2oCallback, this, std::placeholders::_1));

  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom_cmd_vel", 10);

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
}

void CmdVelOdomNode::rf2oCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  const auto & q = msg->pose.pose.orientation;
  rf2o_yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  rf2o_received_ = true;
}

void CmdVelOdomNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  auto current_time = this->now();
  double dt = (current_time - last_time_).seconds();
  last_time_ = current_time;

  yaw_ += msg->angular.z * dt;
  const double active_yaw = rf2o_received_ ? rf2o_yaw_ : yaw_;

  x_ += (msg->linear.x * std::cos(active_yaw) - msg->linear.y * std::sin(active_yaw)) * dt;
  y_ += (msg->linear.x * std::sin(active_yaw) + msg->linear.y * std::cos(active_yaw)) * dt;
  z_ += msg->linear.z * dt;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, active_yaw);

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

  odom_pub_->publish(odom);

  // Broadcast TF: odom_cmd_vel -> simple_drone/base_footprint
  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp    = current_time;
  tf.header.frame_id = "odom_cmd_vel";
  tf.child_frame_id  = "simple_drone/base_footprint";

  tf.transform.translation.x = x_;
  tf.transform.translation.y = y_;
  tf.transform.translation.z = z_;
  tf.transform.rotation.x    = q.x();
  tf.transform.rotation.y    = q.y();
  tf.transform.rotation.z    = q.z();
  tf.transform.rotation.w    = q.w();

  // tf_broadcaster_->sendTransform(tf);
}
