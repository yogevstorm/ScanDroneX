#include <DistMap.hpp>


std::vector<float> DistMap::GenCostMap(navigation_msgs::msg::DistMapMsg p_dist_map, navigation_msgs::msg::WorldPoint p_goal, float p_collision_radius)
{
  auto p_map_info = p_dist_map.info;
  auto goal_map = World2Map2D(p_map_info, p_goal);
  int start_idx = Map2DTo1D(p_map_info, goal_map);
  std::vector<float> costmap(p_map_info.width *  p_map_info.height, 999);
  std::vector<std::tuple<int, float>> neighbors_vec{std::make_tuple(1, p_map_info.resolution), std::make_tuple(-1, p_map_info.resolution), std::make_tuple((int)p_map_info.width, p_map_info.resolution), std::make_tuple(-(int)p_map_info.width, p_map_info.resolution),
                                                   std::make_tuple(1 + (int)p_map_info.width, 1.4 * p_map_info.resolution), std::make_tuple(1 - (int)p_map_info.width, 1.4 * p_map_info.resolution),
                                                   std::make_tuple(-1 + (int)p_map_info.width, 1.4 * p_map_info.resolution), std::make_tuple(-1 - (int)p_map_info.width, 1.4 * p_map_info.resolution)};
  std::map<int, float> open;
  costmap[start_idx] = 0;
  open[start_idx] = 0;
  while (!open.empty())
  {
    int current_node = open.begin()->first;
    open.erase(current_node);
    for (auto [idx_change, cost] : neighbors_vec)
    {
      int neighbor = current_node + idx_change;
      float neighbor_cost = costmap[current_node] + cost;
      if (neighbor < 0 || (unsigned int)neighbor >= p_map_info.width * p_map_info.height)
          continue;
      if (p_dist_map.data[neighbor] < p_collision_radius)
          continue;
      if (neighbor_cost >= costmap[neighbor])
          continue;
      costmap[neighbor] = neighbor_cost;
      open[neighbor] = neighbor_cost;
    }
  }
  return costmap;
}


std::vector<int> DistMap::World2Map2D(nav_msgs::msg::MapMetaData p_map_info, navigation_msgs::msg::WorldPoint p_point)
{
  std::vector<int> map_2d_coordinates;
  int p_map_ind = World2Map1D(p_map_info, p_point);
  int y = (int)(p_map_ind/p_map_info.width);
  int x = p_map_ind - y*p_map_info.width;
  map_2d_coordinates.push_back(x);
  map_2d_coordinates.push_back(y);
  return map_2d_coordinates;
}

int DistMap::World2Map1D(nav_msgs::msg::MapMetaData p_map_info, navigation_msgs::msg::WorldPoint p_point)
{
  float map_yaw = m_control_utils.Quat2Yaw(p_map_info.origin.orientation);
  float x = cos(-map_yaw)*(p_point.x - p_map_info.origin.position.x) - sin(-map_yaw)*(p_point.y - p_map_info.origin.position.y);
  float y = sin(-map_yaw)*(p_point.x - p_map_info.origin.position.x) + cos(-map_yaw)*(p_point.y - p_map_info.origin.position.y);
  int ind = (int)((y)/p_map_info.resolution)*p_map_info.width + (int)((x)/p_map_info.resolution);

  return ind;
}

navigation_msgs::msg::WorldPoint DistMap::Map2World1D(nav_msgs::msg::MapMetaData p_map_info, int p_map_ind){
  std::vector<int> map_2d_coordinates = Map1DTo2D(p_map_info, p_map_ind);
  navigation_msgs::msg::WorldPoint world_coordinates = Map2World2D(p_map_info, map_2d_coordinates);
  return world_coordinates;
}

navigation_msgs::msg::WorldPoint DistMap::Map2World2D(nav_msgs::msg::MapMetaData p_map_info, std::vector<int> p_map_coordinates){
  navigation_msgs::msg::WorldPoint pos;
  float map_yaw;
  map_yaw = m_control_utils.Quat2Yaw(p_map_info.origin.orientation);
  float y = p_map_info.origin.position.y + p_map_info.resolution/2  + int(p_map_coordinates[1])*p_map_info.resolution;
  float x = p_map_info.origin.position.x + p_map_info.resolution/2 + int(p_map_coordinates[0])*p_map_info.resolution;
  pos.x = cos(map_yaw)*x - sin(map_yaw)*y;
  pos.y = sin(map_yaw)*x + cos(map_yaw)*y;
  return pos;
}

std::vector<int> DistMap::Map1DTo2D(nav_msgs::msg::MapMetaData p_map_info,  int p_map_ind)
{
  int y = int(p_map_ind/p_map_info.width);
  int x = int(p_map_ind - y*p_map_info.width);
  std::vector<int> map_2d_coordinates;
  map_2d_coordinates.push_back(x);
  map_2d_coordinates.push_back(y);
  return map_2d_coordinates;
}

int DistMap::Map2DTo1D(nav_msgs::msg::MapMetaData p_map_info, std::vector<int> p_map_coordinates)
{
  return p_map_coordinates[1]*p_map_info.width + p_map_coordinates[0];
}

float DistMap::GetDist(navigation_msgs::msg::WorldPoint p_point, navigation_msgs::msg::DistMapMsg p_dist_map)
{
  int point_idx = World2Map1D(p_dist_map.info, p_point);
  return p_dist_map.data[point_idx];
}


navigation_msgs::msg::DistMapMsg DistMap::CreateDistMap(nav_msgs::msg::OccupancyGrid map, std::vector<int> free_vals){
  navigation_msgs::msg::DistMapMsg dist_map;
  dist_map.info = map.info;
  cv::Mat ogrid(map.info.height, map.info.width, CV_8UC1, map.data.data());
  cv::Mat free_mask = cv::Mat::zeros(ogrid.size(), CV_8UC1);
  for (int val : free_vals)
    free_mask |= (ogrid == static_cast<uint8_t>(static_cast<int8_t>(val)));
  cv::Mat dist_matrix;
  cv::distanceTransform(free_mask, dist_matrix, cv::DIST_L2, cv::DIST_MASK_PRECISE, CV_32FC1);
  assignValues(dist_matrix, dist_map, map.info);
  return dist_map;
}

void DistMap::assignValues(cv::Mat& dist_matrix, navigation_msgs::msg::DistMapMsg& p_dist_map, nav_msgs::msg::MapMetaData p_map_info)
{
  for (int r = 0; r < int(p_map_info.height); r++)
  {
    for (int c = 0; c < int(p_map_info.width); c++)
    {
      //distmap_[i + j*x_size_] = dist.at<float>(j,i) * map.info.resolution;
      p_dist_map.data.push_back(dist_matrix.at<float>(r,c) * p_map_info.resolution);
    }
  }
}


