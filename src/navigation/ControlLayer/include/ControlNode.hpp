#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/bool.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "navigation_msgs/msg/world_point.hpp"
#include "navigation_msgs/msg/dist_map_msg.hpp"
#include "navigation_msgs/msg/path_msg.hpp"
#include "navigation_msgs/msg/drone_state.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include "Control.hpp"
#include <memory>
#include <map>
#include <vector>
#include <cmath>

class ControlNode
{
public:
  void Init_Publishers_Subscribers();
  void InitParams();
  void UpdateParams();
  bool SafetyRequirements();
  void Estop();

  std::shared_ptr<rclcpp::Node> m_node;

  Control m_control;

  bool m_estop = false;

  float m_max_dis_drone_from_traj = 0.5;

private:
  void TrajectoryCallBack(const navigation_msgs::msg::PathMsg::SharedPtr msg);
  void StateCallBack(const navigation_msgs::msg::DroneState::SharedPtr msg);
  bool IsDroneFarFromTraj();
  void UpdateEstop();

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_pub_cmd;
  rclcpp::Subscription<navigation_msgs::msg::PathMsg>::SharedPtr m_sub_path;
  rclcpp::Subscription<navigation_msgs::msg::DroneState>::SharedPtr m_sub_state;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_sub_estop_scan;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_sub_estop_local_planner;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_sub_estop_mission;

  bool m_estop_scan           = false;
  bool m_estop_local_planner  = false;
  bool m_estop_mission        = false;

  navigation_msgs::msg::PathMsg m_trajectory;
  navigation_msgs::msg::DroneState m_drone_state;

  bool m_is_recieved_state_msg = false;

  rclcpp::Time m_traj_timer = rclcpp::Clock().now();

protected:
};
