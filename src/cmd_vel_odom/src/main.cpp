#include "cmd_vel_odom/cmd_vel_odom_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelOdomNode>());
  rclcpp::shutdown();
  return 0;
}
