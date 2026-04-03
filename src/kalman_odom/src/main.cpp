#include "kalman_odom/kalman_odom_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KalmanOdomNode>());
  rclcpp::shutdown();
  return 0;
}
