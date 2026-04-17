#include <ScanLayer.hpp>

bool ScanLayer::FindRandomGoal(const nav_msgs::msg::OccupancyGrid & map,
                               geometry_msgs::msg::PoseStamped & goal)
{
  std::vector<int> unknown_indices;
  for (size_t i = 0; i < map.data.size(); i++)
  {
    if (map.data[i] == -1)
    {
      unknown_indices.push_back(i);
    }
  }

  if (unknown_indices.empty())
  {
    return false;
  }

  std::uniform_int_distribution<int> dist(0, unknown_indices.size() - 1);
  int idx = unknown_indices[dist(m_rng)];

  int col = idx % map.info.width;
  int row = idx / map.info.width;

  goal.header.frame_id = "map";
  goal.pose.position.x = map.info.origin.position.x + (col + 0.5) * map.info.resolution;
  goal.pose.position.y = map.info.origin.position.y + (row + 0.5) * map.info.resolution;
  goal.pose.position.z = 0.0;
  goal.pose.orientation.w = 1.0;

  return true;
}
