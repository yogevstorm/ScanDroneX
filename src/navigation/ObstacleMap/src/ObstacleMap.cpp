#include <ObstacleMap.hpp>
using namespace std::chrono_literals;

void ObstacleMap::RunObstacleMap(nav_msgs::msg::OccupancyGrid occ_map,
                                 sensor_msgs::msg::LaserScan laser_scan,
                                 geometry_msgs::msg::Pose drone_state)
{
  Init(occ_map, laser_scan, drone_state);

  FilterLaserScan();

  BuildObstacleMap();
}

void ObstacleMap::Init(nav_msgs::msg::OccupancyGrid occ_map,
                       sensor_msgs::msg::LaserScan laser_scan,
                       geometry_msgs::msg::Pose drone_state)
{
  m_occ_map = occ_map;
  m_laser_scan = laser_scan;
  m_drone_state = drone_state;

  m_obs_map = occ_map;

  m_filtered_scan_ranges.clear();
}

void ObstacleMap::FilterLaserScan()
{
  for(size_t i = 0; i < m_laser_scan.ranges.size(); i++)
  {
    if(m_laser_scan.ranges[i] < m_max_range_laser_scan_filter)
    {
      std::tuple<float, int> filtered_laser = std::make_tuple(m_laser_scan.ranges[i], i);
      m_filtered_scan_ranges.push_back(filtered_laser);
    }
  }
}

void ObstacleMap::BuildObstacleMap()
{
  float x_offset = 0.2;
  float y_offset = 0.0;

  for(size_t i = 0; i < m_filtered_scan_ranges.size(); i++)
  {
    auto [range, laser_idx] = m_filtered_scan_ranges[i];

    navigation_msgs::msg::WorldPoint obs_wpoint_map_frame, obs_wpoint_drone_frame;

    if(m_sim_flag)
    {
      obs_wpoint_map_frame.x =
          cos(laser_idx * m_laser_scan.angle_increment + M_PI) * range + x_offset;

      obs_wpoint_map_frame.y =
          sin(laser_idx * m_laser_scan.angle_increment + M_PI) * range + y_offset;
    }
    else
    {
      obs_wpoint_map_frame.x =
          sin(laser_idx * m_laser_scan.angle_increment) * range + x_offset;

      obs_wpoint_map_frame.y =
          -cos(laser_idx * m_laser_scan.angle_increment) * range + y_offset;
    }

    obs_wpoint_drone_frame = MapFrame2DroneStateFrame(obs_wpoint_map_frame);

    int obs_mpoint_idx = m_distmap.World2Map1D(m_occ_map.info, obs_wpoint_drone_frame);

    if(obs_mpoint_idx > m_occ_map.info.height * m_occ_map.info.width || obs_mpoint_idx < 0)
    {
      continue;
    }

    m_obs_map.data[obs_mpoint_idx] = 200;
  }
}

navigation_msgs::msg::WorldPoint
ObstacleMap::MapFrame2DroneStateFrame(navigation_msgs::msg::WorldPoint map_frame_point)
{
  navigation_msgs::msg::WorldPoint drone_frame_point;

  Eigen::Matrix3f MapFrame_2_DroneFrame =
      CreateTransformationMatrix(m_drone_state.position.x,
                                 m_drone_state.position.y,
                                 m_drone_state.position.z);

  Eigen::Vector3f MapFrame_Point(map_frame_point.x, map_frame_point.y, 1);

  Eigen::Vector3f drone_frame_point_eigen = MapFrame_2_DroneFrame * MapFrame_Point;

  drone_frame_point.x = drone_frame_point_eigen(0);
  drone_frame_point.y = drone_frame_point_eigen(1);

  return drone_frame_point;
}

Eigen::Matrix3f ObstacleMap::CreateTransformationMatrix(float x, float y, float yaw)
{
  Eigen::Matrix3f transformationMatrix;

  // Rotation part
  transformationMatrix(0,0) = std::cos(yaw);
  transformationMatrix(0,1) = -std::sin(yaw);
  transformationMatrix(1,0) = std::sin(yaw);
  transformationMatrix(1,1) = std::cos(yaw);

  // Translation part
  transformationMatrix(0,2) = x;
  transformationMatrix(1,2) = y;

  // Homogeneous coordinate part
  transformationMatrix(2,0) = 0;
  transformationMatrix(2,1) = 0;
  transformationMatrix(2,2) = 1;

  return transformationMatrix;
}
