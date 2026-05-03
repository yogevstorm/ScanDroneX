#include "cmd_vel_odom/slam_scan_toggle_node.hpp"
#include <rcl_interfaces/msg/parameter.hpp>
#include <rcl_interfaces/msg/parameter_type.hpp>

SlamScanToggleNode::SlamScanToggleNode() : Node("slam_scan_toggle")
{
  sub_ = create_subscription<std_msgs::msg::Bool>(
    "/odom_switch/using_cmd_vel", 10,
    std::bind(&SlamScanToggleNode::onUsingCmdVel, this, std::placeholders::_1));

  client_ = create_client<rcl_interfaces::srv::SetParameters>(
    "/slam_toolbox/set_parameters");

  RCLCPP_INFO(get_logger(), "slam_scan_toggle ready");
}

void SlamScanToggleNode::onUsingCmdVel(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (initialized_ && msg->data == last_using_cmd_vel_) {
    return;
  }
  last_using_cmd_vel_ = msg->data;
  initialized_ = true;
  setScanMatching(!msg->data);
}

void SlamScanToggleNode::setScanMatching(bool enabled)
{
  if (!client_->service_is_ready()) {
    RCLCPP_WARN(get_logger(), "slam_toolbox set_parameters service not ready");
    return;
  }

  auto req = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();

  rcl_interfaces::msg::Parameter p;
  p.name = "use_scan_matching";
  p.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
  p.value.bool_value = enabled;

  req->parameters = {p};

  client_->async_send_request(req);
  RCLCPP_INFO(get_logger(), "use_scan_matching -> %s", enabled ? "true" : "false");
}
