#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <boost/math/interpolators/barycentric_rational.hpp>
#include <std_msgs/msg/int8.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "navigation_msgs/msg/world_point.hpp"
#include "navigation_msgs/msg/path_msg.hpp"
#include <memory>
#include <map>
#include <vector>
#include <cmath>
#include <Astar.hpp>
#include <ControlUtils.hpp>

class MissionPath
{
public:
  nav_msgs::msg::OccupancyGrid LoadMap();
  void VisualizePath(navigation_msgs::msg::PathMsg path);
  void Init_Publishers_Subscribers();

  navigation_msgs::msg::PathMsg FindPath(geometry_msgs::msg::Pose start,
                                         geometry_msgs::msg::Pose end,
                                         navigation_msgs::msg::DistMapMsg dist_map);

  std::shared_ptr<rclcpp::Node> m_node;
  Astar m_astar;
  bool m_recieved_map_msg = false;
  float m_ds = 0.1;

private:
  void MapCallBack(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  navigation_msgs::msg::PathMsg Smooth(navigation_msgs::msg::PathMsg path,
                                       float weight_data,
                                       float weight_smooth,
                                       double tolerance,
                                       int max_iterations);
  navigation_msgs::msg::PathMsg Interpolation(navigation_msgs::msg::PathMsg path);

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr m_sub_map;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_pub_path;

  nav_msgs::msg::OccupancyGrid m_map;
  ControlUtils m_control_utils;

protected:
};
