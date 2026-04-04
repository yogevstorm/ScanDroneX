#include <algorithm>
#include "joy_teleop_cpp/joy_teleop_node.hpp"

JoyTeleopNode::JoyTeleopNode()
: Node("joy_teleop_node"),
  prev_btn_takeoff_(false),
  prev_btn_land_(false)
{
  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/simple_drone/cmd_vel", 10);
  takeoff_pub_ = create_publisher<std_msgs::msg::Empty>("/simple_drone/takeoff", 10);
  land_pub_    = create_publisher<std_msgs::msg::Empty>("/simple_drone/land", 10);

  joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
    "/joy", 10,
    std::bind(&JoyTeleopNode::joyCallback, this, std::placeholders::_1));
}

double JoyTeleopNode::scale(double axis, double max)
{
  return std::clamp(axis * max, -max, max);
}

void JoyTeleopNode::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  geometry_msgs::msg::Twist twist;
  if (msg->axes.size() >= 4) {
    twist.linear.x  = scale(msg->axes[1], MAX_LINEAR);
    twist.linear.y  = scale(msg->axes[0], MAX_LINEAR);
    twist.linear.z  = scale(msg->axes[4], MAX_LINEAR);
    twist.angular.z = scale(msg->axes[3], MAX_ANGULAR);
  }
  cmd_vel_pub_->publish(twist);

  if (msg->buttons.size() >= 2) {
    bool btn_takeoff = msg->buttons[0] == 1;
    bool btn_land    = msg->buttons[1] == 1;

    if (btn_takeoff && !prev_btn_takeoff_) {
      takeoff_pub_->publish(std_msgs::msg::Empty());
    }

    if (btn_land && !prev_btn_land_) {
      cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
      land_pub_->publish(std_msgs::msg::Empty());
    }

    prev_btn_takeoff_ = btn_takeoff;
    prev_btn_land_    = btn_land;
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyTeleopNode>());
  rclcpp::shutdown();
  return 0;
}
