#include "cmd_vel_odom/odom_disagreement_node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomDisagreementNode>());
  rclcpp::shutdown();
  return 0;
}
