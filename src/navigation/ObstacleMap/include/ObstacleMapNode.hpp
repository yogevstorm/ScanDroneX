#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "navigation_msgs/msg/world_point.hpp"
#include "navigation_msgs/msg/dist_map_msg.hpp"
#include "ObstacleMap.hpp"
#include <memory>
#include <map>
#include <vector>
#include <cmath>

class ObstacleMapNode
{
public:
  void Init_Publishers_Subscribers();
  void RunObstacleMapNode();
  void InitParams();
  void UpdateParams();

  std::shared_ptr<rclcpp::Node> m_node;
  ObstacleMap m_obstacle_map;

private:
  void MapCallBack(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void LaserScanCallBack(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void DroneStateCallBack(const geometry_msgs::msg::Pose::SharedPtr msg);

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr m_pub_obs_map;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr m_sub_map;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr m_sub_laser;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr m_sub_drone_state;

  geometry_msgs::msg::Pose m_drone_state;
  nav_msgs::msg::OccupancyGrid m_occ_map;
  sensor_msgs::msg::LaserScan m_laser_scan;

  bool m_recieved_map_msg = false;
  bool m_recieved_laser_msg = false;

protected:
};
