#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <memory>
#include <ScanLayer.hpp>

class ScanNode
{
public:
  void Init();
  void Run();

  std::shared_ptr<rclcpp::Node> m_node;

private:
  void MapCallBack(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void IsDestinationCallBack(const std_msgs::msg::Bool::SharedPtr msg);
  void IsPathBlockedCallBack(const std_msgs::msg::Bool::SharedPtr msg);
  void PublishNewGoal();
  void PublishEstop(bool estop);

  ScanLayer m_scan_layer;
  nav_msgs::msg::OccupancyGrid m_map;
  geometry_msgs::msg::PoseStamped m_current_goal;

  bool m_is_map_received = false;
  bool m_has_goal        = false;
  bool m_is_path_blocked = false;

  rclcpp::Time m_last_blocked_pub{0, 0, RCL_ROS_TIME};

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr m_pub_goal_pose;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_pub_estop;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr m_sub_map;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_sub_is_destination;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_sub_is_path_blocked;

protected:
};
