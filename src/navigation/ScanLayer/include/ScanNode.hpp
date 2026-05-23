#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <navigation_msgs/msg/drone_state.hpp>
#include <memory>

class ScanNode
{
public:
  void Init();
  void Run();
  void InitParams();
  void UpdateParams();

  std::shared_ptr<rclcpp::Node> m_node;

private:
  void MapCallBack(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void IsDestinationCallBack(const std_msgs::msg::Bool::SharedPtr msg);
  void IsPathBlockedCallBack(const std_msgs::msg::Bool::SharedPtr msg);
  void GoalUnreachableCallBack(const std_msgs::msg::Bool::SharedPtr msg);
  void IsOutOfLaneCallBack(const std_msgs::msg::Bool::SharedPtr msg);
  void DroneStateCallBack(const navigation_msgs::msg::DroneState::SharedPtr msg);
  void OpeningGoalCallBack(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void PublishNewGoal();
  void PublishEstop(bool estop);

  nav_msgs::msg::OccupancyGrid       m_map;
  geometry_msgs::msg::PoseStamped    m_current_goal;
  geometry_msgs::msg::PoseStamped    m_latest_opening;

  bool  m_is_map_received     = false;
  bool  m_has_goal            = false;
  bool  m_is_path_blocked     = false;
  bool  m_is_goal_unreachable = false;
  bool  m_returning_home      = false;
  bool  m_has_opening         = false;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr m_pub_goal_pose;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr             m_pub_estop;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr m_sub_map;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr          m_sub_is_destination;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr          m_sub_lane_end;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr          m_sub_is_path_blocked;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr          m_sub_goal_unreachable;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr          m_sub_is_out_of_lane;
  rclcpp::Subscription<navigation_msgs::msg::DroneState>::SharedPtr        m_sub_drone_state;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr         m_sub_opening_goal;
};
