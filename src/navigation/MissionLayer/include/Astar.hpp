#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <std_msgs/msg/int8.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include "navigation_msgs/msg/world_point.hpp"
#include "navigation_msgs/msg/path_msg.hpp"
#include <chrono>
#include <memory>
#include <queue>
#include <unordered_map>
#include <vector>
#include <cmath>
#include <DistMap.hpp>
#include <Node.hpp>
#include <ControlUtils.hpp>

using std::placeholders::_1;

struct NodeCompare {
  bool operator()(const Node& a, const Node& b) const { return a.f > b.f; }
};

class Astar
{
public:
  std::shared_ptr<rclcpp::Node> m_node;

  navigation_msgs::msg::PathMsg Search(navigation_msgs::msg::DistMapMsg dist_map,
                                       geometry_msgs::msg::Pose start,
                                       geometry_msgs::msg::Pose end);

  float  m_collision_r        = 0.15f;
  int    m_step_size          = 3;      // cells per step (3 * 0.05m = 0.15m/step)
  float  m_heuristic_weight   = 2.0f;   // >1 trades optimality for speed
  double m_search_timeout_sec = 10.0;

private:
  float GetHeuristic(Node node);
  bool  IsNodeCollide(Node node, navigation_msgs::msg::DistMapMsg dist_map);
  bool  IsNodeOutOfBounds(Node node, navigation_msgs::msg::DistMapMsg dist_map);
  void  CreateNeighbors();

  navigation_msgs::msg::PathMsg BackTracking(std::unordered_map<int, Node>& closed_map,
                                             nav_msgs::msg::MapMetaData map_info);

  nav_msgs::msg::MapMetaData m_map_info;
  std::vector<std::tuple<int, int, float>> m_neighbors_related_to_node;

  ControlUtils m_control_utils;
  DistMap      m_dist_map_obj;
  Node         m_end_node;

protected:
};
