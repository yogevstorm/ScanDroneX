#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "navigation_msgs/msg/world_point.hpp"
#include "navigation_msgs/msg/dist_map_msg.hpp"
#include "navigation_msgs/msg/path_msg.hpp"
#include <DistMap.hpp>
#include <memory>
#include <map>
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include <unsupported/Eigen/Splines>
#include <ControlUtils.hpp>

class ObstacleMap
{
public:
  void RunObstacleMap(nav_msgs::msg::OccupancyGrid occ_map,
                      sensor_msgs::msg::LaserScan laser_scan,
                      geometry_msgs::msg::Pose drone_state);

  nav_msgs::msg::OccupancyGrid m_obs_map;

  float m_max_range_laser_scan_filter = 4.0;
  bool m_sim_flag = true;

private:
  void Init(nav_msgs::msg::OccupancyGrid occ_map,
            sensor_msgs::msg::LaserScan laser_scan,
            geometry_msgs::msg::Pose drone_state);

  void BuildObstacleMap();
  Eigen::Matrix3f CreateTransformationMatrix(float x, float y, float yaw);
  void FilterLaserScan();

  navigation_msgs::msg::WorldPoint
  MapFrame2DroneStateFrame(navigation_msgs::msg::WorldPoint map_frame_point);

  DistMap m_distmap;
  ControlUtils m_control_utils;

  std::vector<std::tuple<float, int>> m_filtered_scan_ranges;

  nav_msgs::msg::OccupancyGrid m_occ_map;
  sensor_msgs::msg::LaserScan m_laser_scan;
  geometry_msgs::msg::Pose m_drone_state;

protected:
};
