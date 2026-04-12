#include <Astar.hpp>

navigation_msgs::msg::PathMsg Astar::BackTracking(std::unordered_map<int, Node>& closed_map,
                                                   nav_msgs::msg::MapMetaData map_info)
{
  navigation_msgs::msg::PathMsg path;
  Node curr_node = m_end_node;

  while(curr_node.parent_ind != -1)
  {
    navigation_msgs::msg::WorldPoint path_point;
    path_point = m_dist_map_obj.Map2World2D(map_info, curr_node.mpoint);
    path_point.yaw = curr_node.wpoint.yaw;
    path.path_msg.push_back(path_point);
    curr_node = closed_map[curr_node.parent_ind];
  }

  std::reverse(path.path_msg.begin(), path.path_msg.end());
  return path;
}

// Rebuild neighbor template each Search() call — clears first to prevent accumulation.
void Astar::CreateNeighbors()
{
  m_neighbors_related_to_node.clear();
  int s = m_step_size;
  m_neighbors_related_to_node = {
    { 0, -s,  -M_PI                      },
    { 0,  s,   0.0f                      },
    {-s,  0,   (float)M_PI_2             },
    { s,  0,  -(float)M_PI_2             },
    {-s, -s,   (float)(M_PI_4 + M_PI_2)  },
    {-s,  s,   (float)M_PI_4             },
    { s, -s,  -(float)(M_PI_4 + M_PI_2)  },
    { s,  s,  -(float)M_PI_4             },
  };
}

bool Astar::IsNodeOutOfBounds(Node node, navigation_msgs::msg::DistMapMsg dist_map)
{
  if(node.mpoint[0] > (int)dist_map.info.width  || node.mpoint[0] < 0 ||
     node.mpoint[1] > (int)dist_map.info.height || node.mpoint[1] < 0)
  {
    return true;
  }
  return false;
}

bool Astar::IsNodeCollide(Node node, navigation_msgs::msg::DistMapMsg dist_map)
{
  if(dist_map.data[node.ind] < m_collision_r)
  {
    return true;
  }
  return false;
}

// Weighted heuristic: m_heuristic_weight > 1 reduces node expansions at the cost of
// path optimality (path length <= W * optimal).
float Astar::GetHeuristic(Node node)
{
  float dx = (node.mpoint[0] - m_end_node.mpoint[0]) * m_map_info.resolution;
  float dy = (node.mpoint[1] - m_end_node.mpoint[1]) * m_map_info.resolution;
  return m_heuristic_weight * std::sqrt(dx * dx + dy * dy);
}

navigation_msgs::msg::PathMsg Astar::Search(navigation_msgs::msg::DistMapMsg dist_map,
                                             geometry_msgs::msg::Pose start,
                                             geometry_msgs::msg::Pose end)
{
  m_map_info = dist_map.info;

  navigation_msgs::msg::WorldPoint start_pose, end_pose;
  start_pose.x = start.position.x;
  start_pose.y = start.position.y;
  end_pose.x   = end.position.x;
  end_pose.y   = end.position.y;

  // Set goal first — GetHeuristic() needs m_end_node.mpoint.
  m_end_node.mpoint     = m_dist_map_obj.World2Map2D(dist_map.info, end_pose);
  m_end_node.wpoint.yaw = m_control_utils.Quat2Yaw(end.orientation);
  m_end_node.ind        = m_dist_map_obj.Map2DTo1D(dist_map.info, m_end_node.mpoint);

  Node start_node;
  start_node.mpoint     = m_dist_map_obj.World2Map2D(dist_map.info, start_pose);
  start_node.wpoint.yaw = start.position.z;
  start_node.ind        = m_dist_map_obj.Map2DTo1D(dist_map.info, start_node.mpoint);
  start_node.g          = 0.0f;
  start_node.h          = GetHeuristic(start_node);
  start_node.f          = start_node.h;
  start_node.parent_ind = -1;

  CreateNeighbors();

  // Min-heap on f. Lazy deletion: re-inserted nodes are skipped when popped
  // if already in closed_map — no decrease-key needed.
  std::priority_queue<Node, std::vector<Node>, NodeCompare> open_pq;
  std::unordered_map<int, float> open_cost;   // best g seen per cell
  std::unordered_map<int, Node>  closed_map;

  open_pq.push(start_node);
  open_cost[start_node.ind] = 0.0f;

  const float goal_tol      = m_step_size * 1.5f;
  const float straight_cost = m_step_size * dist_map.info.resolution;
  const float diag_cost     = m_step_size * (float)M_SQRT2 * dist_map.info.resolution;

  while(!open_pq.empty())
  {
    Node current = open_pq.top();
    open_pq.pop();

    // Already settled with lower cost — discard this stale entry.
    if(closed_map.count(current.ind))
      continue;

    closed_map[current.ind] = current;

    if(m_control_utils.DisBetweenMPoints(current.mpoint, m_end_node.mpoint) < goal_tol)
    {
      std::cout << "FINISHED" << std::endl;
      m_end_node.parent_ind = current.ind;
      return BackTracking(closed_map, dist_map.info);
    }

    for(auto& [dx, dy, yaw] : m_neighbors_related_to_node)
    {
      Node nb;
      nb.mpoint.push_back(current.mpoint[0] + dx);
      nb.mpoint.push_back(current.mpoint[1] + dy);
      nb.wpoint.yaw = yaw + (float)M_PI_2;

      if(IsNodeOutOfBounds(nb, dist_map))
        continue;

      nb.ind = m_dist_map_obj.Map2DTo1D(dist_map.info, nb.mpoint);

      if(IsNodeCollide(nb, dist_map))
        continue;

      if(closed_map.count(nb.ind))
        continue;

      // Diagonal steps cost sqrt(2) more than straight steps.
      float step_cost = (dx != 0 && dy != 0) ? diag_cost : straight_cost;
      nb.g = current.g + step_cost;

      // Skip if we already know a cheaper route to this cell.
      auto it = open_cost.find(nb.ind);
      if(it != open_cost.end() && it->second <= nb.g)
        continue;

      nb.h          = GetHeuristic(nb);
      nb.f          = nb.g + nb.h;
      nb.parent_ind = current.ind;

      open_pq.push(nb);
      open_cost[nb.ind] = nb.g;
    }
  }

  return navigation_msgs::msg::PathMsg{};
}
