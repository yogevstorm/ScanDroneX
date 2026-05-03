#include <ScanLayer.hpp>
#include <limits>
#include <algorithm>

bool ScanLayer::FindRandomGoal(const nav_msgs::msg::OccupancyGrid & map,
                               geometry_msgs::msg::PoseStamped & goal,
                               bool has_last_goal,
                               const geometry_msgs::msg::PoseStamped & last_goal,
                               int num_candidates)
{
  std::vector<int> unknown_indices;
  for (size_t i = 0; i < map.data.size(); i++)
  {
    if (map.data[i] == -1)
      unknown_indices.push_back(i);
  }

  if (unknown_indices.empty())
    return false;

  auto idx_to_world = [&](int idx, float & wx, float & wy)
  {
    int col = idx % map.info.width;
    int row = idx / map.info.width;
    wx = map.info.origin.position.x + (col + 0.5f) * map.info.resolution;
    wy = map.info.origin.position.y + (row + 0.5f) * map.info.resolution;
  };

  std::uniform_int_distribution<int> dist(0, unknown_indices.size() - 1);
  int chosen_idx;

  if (!has_last_goal || num_candidates <= 1 || unknown_indices.size() == 1)
  {
    chosen_idx = unknown_indices[dist(m_rng)];
  }
  else
  {
    int k = std::min(num_candidates, static_cast<int>(unknown_indices.size()));
    float lx = static_cast<float>(last_goal.pose.position.x);
    float ly = static_cast<float>(last_goal.pose.position.y);

    chosen_idx = -1;
    float best_dist_sq = -1.0f;

    for (int i = 0; i < k; i++)
    {
      int idx = unknown_indices[dist(m_rng)];
      float wx, wy;
      idx_to_world(idx, wx, wy);
      float dx = wx - lx;
      float dy = wy - ly;
      float dist_sq = dx * dx + dy * dy;
      if (dist_sq > best_dist_sq)
      {
        best_dist_sq = dist_sq;
        chosen_idx = idx;
      }
    }
  }

  float wx, wy;
  idx_to_world(chosen_idx, wx, wy);

  goal.header.frame_id = "map";
  goal.pose.position.x = wx;
  goal.pose.position.y = wy;
  goal.pose.position.z = 0.0;
  goal.pose.orientation.w = 1.0;

  return true;
}

bool ScanLayer::FindCornerGoal(const nav_msgs::msg::OccupancyGrid & map,
                                geometry_msgs::msg::PoseStamped & goal,
                                int corner_index)
{
  const float ox  = map.info.origin.position.x;
  const float oy  = map.info.origin.position.y;
  const float map_w = map.info.width  * map.info.resolution;
  const float map_h = map.info.height * map.info.resolution;

  // 0=bottom-left  1=bottom-right  2=top-right  3=top-left
  const float cx[4] = { ox,          ox + map_w,  ox + map_w,  ox         };
  const float cy[4] = { oy,          oy,           oy + map_h,  oy + map_h };

  const float target_x = cx[corner_index & 3];
  const float target_y = cy[corner_index & 3];

  int   best_idx     = -1;
  float best_dist_sq = std::numeric_limits<float>::max();

  for (size_t i = 0; i < map.data.size(); i++)
  {
    if (map.data[i] != -1) continue;

    int col = i % map.info.width;
    int row = i / map.info.width;
    float wx = ox + (col + 0.5f) * map.info.resolution;
    float wy = oy + (row + 0.5f) * map.info.resolution;

    float dx = wx - target_x;
    float dy = wy - target_y;
    float dist_sq = dx * dx + dy * dy;
    if (dist_sq < best_dist_sq)
    {
      best_dist_sq = dist_sq;
      best_idx     = static_cast<int>(i);
    }
  }

  if (best_idx < 0) return false;

  int col = best_idx % map.info.width;
  int row = best_idx / map.info.width;

  goal.header.frame_id    = "map";
  goal.pose.position.x    = ox + (col + 0.5f) * map.info.resolution;
  goal.pose.position.y    = oy + (row + 0.5f) * map.info.resolution;
  goal.pose.position.z    = 0.0;
  goal.pose.orientation.w = 1.0;

  return true;
}
