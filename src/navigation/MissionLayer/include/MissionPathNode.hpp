#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "navigation_msgs/msg/world_point.hpp"
#include "navigation_msgs/msg/dist_map_msg.hpp"
#include <memory>
#include <map>
#include <vector>
#include <cmath>
#include <DistMap.hpp>
#include <MissionPath.hpp>
#include <ControlUtils.hpp>

class MissionPathNode
{
public:
  void Init_Publishers_Subscribers();
  void Init();
  void InitMission();
  void RunMission();
  void GetDroneState();

  void GoalPoseCallBack(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void StateCallBack(const geometry_msgs::msg::Pose::SharedPtr msg);
  void ObstacleMapCallBack(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

  void UpdateParams();
  void CollisionRViz();
  bool IsDestination();
  void PubEstop(bool estop);
  void ESTOPCollisonR();

  nav_msgs::msg::OccupancyGrid m_map;
  navigation_msgs::msg::DistMapMsg m_dist_map;

  // std::vector<navigation_msgs::msg::WorldPoint> mpath_, rpath_;
  navigation_msgs::msg::PathMsg m_mpath;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_pub_estop;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr m_pub_drone_vel;
  rclcpp::Publisher<navigation_msgs::msg::PathMsg>::SharedPtr m_pub_mission_path;

  std::shared_ptr<rclcpp::Node> m_node;

  std::vector<double> m_collision_pnts = {0.0, 0.2};

  bool m_recieved_goal_pose_msg = false;
  bool m_is_recieved_state_msg = false;
  bool m_is_recieved_mpath = false;

  float m_end_tolerance = 0.2;
  float m_esp_collision_r = 0.15;

private:
  void CalcDroneVel(const geometry_msgs::msg::Pose current_drone_state);
  void InitParams();

  DistMap m_distmap;
  MissionPath m_mission_path;
  ControlUtils m_control_utils;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr m_sub_goal_pose;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr m_sub_state_pose;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr m_sub_obstacle_map;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_pub_collision_r_vis;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_pub_esp_collision_r_vis;
  rclcpp::Publisher<navigation_msgs::msg::DistMapMsg>::SharedPtr m_pub_dist_map;

  geometry_msgs::msg::PoseStamped m_goal_pose;
  geometry_msgs::msg::Pose m_drone_state;
  nav_msgs::msg::OccupancyGrid m_obs_map;

  float m_collision_r = 0.2;
  float m_drone_vel = 0.0;
  float m_localization_dt = 999.9;

  int64_t m_start_time = cv::getTickCount();

protected:
};
