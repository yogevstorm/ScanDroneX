#include "cmd_vel_odom/slam_scan_toggle_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SlamScanToggleNode>());
  rclcpp::shutdown();
  return 0;
}
