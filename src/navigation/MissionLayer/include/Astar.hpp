#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <std_msgs/msg/int8.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include "navigation_msgs/msg/world_point.hpp"
#include "navigation_msgs/msg/path_msg.hpp"
#include <memory>
#include <map>
#include <vector>
#include <cmath>
#include <DistMap.hpp>
#include <Node.hpp>
#include <ControlUtils.hpp>

using std::placeholders::_1;

class Astar
{
public:
  std::shared_ptr<rclcpp::Node> m_node;

  navigation_msgs::msg::PathMsg Search(navigation_msgs::msg::DistMapMsg dist_map,
                                       geometry_msgs::msg::Pose start,
                                       geometry_msgs::msg::Pose end);

  float m_collision_r = 0.15;

private:
  float GetHeuristic(Node node);

  std::map<int, Node> AppendToOpenList(std::map<int, Node> open_list,
                                       std::map<int, Node> closed_list,
                                       nav_msgs::msg::MapMetaData map_info,
                                       Node current_node);

  bool IsNodeCollide(Node node, navigation_msgs::msg::DistMapMsg dist_map);
  bool IsNodeOutOfBounds(Node node, navigation_msgs::msg::DistMapMsg dist_map);

  void CreateNeighbors();
  void NeighborNode(Node current_node, navigation_msgs::msg::DistMapMsg dist_map);

  navigation_msgs::msg::PathMsg BackTracking(std::map<int, Node> closed_list,
                                             nav_msgs::msg::MapMetaData map_info);

  float m_goal_tolerance = 1.1;
  std::vector<std::tuple<int, int, float>> m_neighbors_related_to_node;
  std::vector<Node> m_neighbors;
  nav_msgs::msg::MapMetaData m_map_info;

  ControlUtils m_control_utils;
  DistMap m_dist_map_obj;
  Node m_end_node;

protected:
};
