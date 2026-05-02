#include "cmd_vel_odom/odom_switch_node.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>

OdomSwitchNode::OdomSwitchNode()
: Node("odom_switch_node")
{
  threshold_high_ = this->declare_parameter<double>("threshold_high", 0.8);
  threshold_low_  = this->declare_parameter<double>("threshold_low",  0.2);

  rf2o_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom_rf2o", 10,
    std::bind(&OdomSwitchNode::rf2oCallback, this, std::placeholders::_1));

  cmd_vel_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom_cmd_vel", 10,
    std::bind(&OdomSwitchNode::cmdVelOdomCallback, this, std::placeholders::_1));

  disagreement_sub_ = this->create_subscription<std_msgs::msg::Float32>(
    "/odom_disagreement/linear", 10,
    std::bind(&OdomSwitchNode::disagreementCallback, this, std::placeholders::_1));

  odom_switch_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom_switch", 10);
  source_pub_      = this->create_publisher<std_msgs::msg::Bool>("/odom_switch/using_cmd_vel", 10);
  tf_broadcaster_  = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
}

void OdomSwitchNode::disagreementCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
  const float val = msg->data;
  if (val > static_cast<float>(threshold_high_)) {
    if (active_source_ != Source::CMD_VEL) {
      RCLCPP_INFO(this->get_logger(),
        "[odom_switch] disagreement=%.3f > %.2f → switching to odom_cmd_vel",
        val, threshold_high_);
      active_source_ = Source::CMD_VEL;
    }
  } else if (val < static_cast<float>(threshold_low_)) {
    if (active_source_ != Source::RF2O) {
      RCLCPP_INFO(this->get_logger(),
        "[odom_switch] disagreement=%.3f < %.2f → switching to odom_rf2o",
        val, threshold_low_);
      active_source_ = Source::RF2O;
    }
  }
  // between thresholds: hold current selection

  std_msgs::msg::Bool source_msg;
  source_msg.data = (active_source_ == Source::CMD_VEL);
  source_pub_->publish(source_msg);
}

void OdomSwitchNode::rf2oCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  latest_rf2o_ = msg;
  if (active_source_ == Source::RF2O) {
    publish(latest_rf2o_);
  }
}

void OdomSwitchNode::cmdVelOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  latest_cmd_vel_ = msg;
  if (active_source_ == Source::CMD_VEL) {
    publish(latest_cmd_vel_);
  }
}

void OdomSwitchNode::publish(const nav_msgs::msg::Odometry::SharedPtr & odom)
{
  nav_msgs::msg::Odometry out = *odom;
  out.header.frame_id = "odom_switch";

  odom_switch_pub_->publish(out);

  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp    = out.header.stamp;
  tf.header.frame_id = "odom_switch";
  tf.child_frame_id  = out.child_frame_id;

  tf.transform.translation.x = out.pose.pose.position.x;
  tf.transform.translation.y = out.pose.pose.position.y;
  tf.transform.translation.z = out.pose.pose.position.z;
  tf.transform.rotation      = out.pose.pose.orientation;

  tf_broadcaster_->sendTransform(tf);
}
