#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/int8.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "navigation_msgs/msg/world_point.hpp"
#include "navigation_msgs/msg/dist_map_msg.hpp"
#include "navigation_msgs/msg/path_msg.hpp"
#include "navigation_msgs/msg/drone_state.hpp"
#include <memory>
#include <map>
#include <vector>
#include <cmath>
#include <ControlUtils.hpp>

class Control
{
public:
  std::vector<float> GetCmd(navigation_msgs::msg::DroneState drone_state, navigation_msgs::msg::PathMsg trajectory);

  float m_max_steer_angle = 0.523;
  float m_wheel_base = 0.26;
  float m_ds = 0.2;
  float m_lookahead_dis = 1.0;
  float m_v_max = 0.8;
  float m_v_min = 0.2;
  float m_k_gain = 1.0;

private:
  float PurePursuit(bool is_reverse);
  float AdaptiveSpeed();
  int FindClosestIdx();
  int FindLookaheadIdx(int closest_idx);
  std::tuple<float, float> RelPointToDrone(float x, float y);

  ControlUtils m_control_utils;

  navigation_msgs::msg::DroneState m_drone_state;
  navigation_msgs::msg::PathMsg m_trajectory;

  int m_closest_idx = 0;
  int m_lookahead_idx = 0;

protected:
};
