#include <Astar.hpp>

navigation_msgs::msg::PathMsg Astar::BackTracking(std::map<int, Node> closed_list,
                                                  nav_msgs::msg::MapMetaData map_info)
{
  navigation_msgs::msg::PathMsg path;
  Node curr_node = m_end_node;
  int c = 0;

  while(curr_node.parent_ind != -1)
  {
    navigation_msgs::msg::WorldPoint path_point;
    path_point = m_dist_map_obj.Map2World2D(map_info, curr_node.mpoint);
    path_point.yaw = curr_node.wpoint.yaw;
    path.path_msg.push_back(path_point);
    curr_node = closed_list[curr_node.parent_ind];
    c++;
  }

  std::reverse(path.path_msg.begin(), path.path_msg.end());
  return path;
}

void Astar::CreateNeighbors()
{
  std::tuple<int, int, float> n1 = {0, -1, -M_PI};
  std::tuple<int, int, float> n2 = {0, 1, 0};
  std::tuple<int, int, float> n3 = {-1, 0, M_PI_2};
  std::tuple<int, int, float> n4 = {1, 0, -M_PI_2};

  std::tuple<int, int, float> n5 = {-1, -1, M_PI_4 + M_PI_2};
  std::tuple<int, int, float> n6 = {-1, 1, M_PI_4};
  std::tuple<int, int, float> n7 = {1, -1, -M_PI_4 - M_PI_2};
  std::tuple<int, int, float> n8 = {1, 1, -M_PI_4};

  m_neighbors_related_to_node.push_back(n1);
  m_neighbors_related_to_node.push_back(n2);
  m_neighbors_related_to_node.push_back(n3);
  m_neighbors_related_to_node.push_back(n4);
  m_neighbors_related_to_node.push_back(n5);
  m_neighbors_related_to_node.push_back(n6);
  m_neighbors_related_to_node.push_back(n7);
  m_neighbors_related_to_node.push_back(n8);
}

bool Astar::IsNodeOutOfBounds(Node node, navigation_msgs::msg::DistMapMsg dist_map)
{
  if(node.mpoint[0] > (dist_map.info.width) || node.mpoint[0] < 0 ||
     node.mpoint[1] > (dist_map.info.height) || node.mpoint[1] < 0)
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

void Astar::NeighborNode(Node current_node, navigation_msgs::msg::DistMapMsg dist_map)
{
  for(int i = 0; i < m_neighbors_related_to_node.size(); i++)
  {
    // Get node position
    Node neighbor_node;
    neighbor_node.mpoint.push_back(current_node.mpoint[0] + std::get<0>(m_neighbors_related_to_node[i]));
    neighbor_node.mpoint.push_back(current_node.mpoint[1] + std::get<1>(m_neighbors_related_to_node[i]));
    neighbor_node.wpoint.yaw = std::get<2>(m_neighbors_related_to_node[i]) + M_PI_2;

    // Make sure within range
    if(IsNodeOutOfBounds(neighbor_node, dist_map))
    {
      continue;
    }

    // Make sure walkable terrain
    neighbor_node.parent_ind = current_node.ind;
    neighbor_node.ind = m_dist_map_obj.Map2DTo1D(dist_map.info, neighbor_node.mpoint);

    if(IsNodeCollide(neighbor_node, dist_map))
    {
      continue;
    }

    m_neighbors.push_back(neighbor_node);
  }
}

float Astar::GetHeuristic(Node node)
{
  float dx = (node.mpoint[0] - m_end_node.mpoint[0]) * m_map_info.resolution;
  float dy = (node.mpoint[1] - m_end_node.mpoint[1]) * m_map_info.resolution;
  return std::sqrt(dx * dx + dy * dy);
}

std::map<int, Node> Astar::AppendToOpenList(std::map<int, Node> open_list,
                                            std::map<int, Node> closed_list,
                                            nav_msgs::msg::MapMetaData map_info,
                                            Node current_node)
{
  for(int i = 0; i < m_neighbors.size(); i++)
  {
    bool skip = false;

    // neighbor is on the closed list
    for(auto element : closed_list)
    {
      if(m_neighbors[i].ind == element.first)
      {
        skip = true;
        break;
      }
    }

    if(skip)
    {
      continue;
    }

    // Create the f, g, and h values
    m_neighbors[i].g = current_node.g + map_info.resolution;
    // m_neighbors[i].h = Dis(m_neighbors[i].mpoint, m_end_node.mpoint);
    m_neighbors[i].h = GetHeuristic(m_neighbors[i]);
    m_neighbors[i].f = m_neighbors[i].g + m_neighbors[i].h;

    // Child is already in the open list
    for(auto element : open_list)
    {
      if(m_neighbors[i].ind == element.first && m_neighbors[i].g > element.second.g)
      {
        skip = true;
        break;
      }
    }

    if(skip)
    {
      continue;
    }

    // Add the child to the open list
    open_list[m_neighbors[i].ind] = m_neighbors[i];
  }

  return open_list;
}

navigation_msgs::msg::PathMsg Astar::Search(navigation_msgs::msg::DistMapMsg dist_map,
                                            geometry_msgs::msg::Pose start,
                                            geometry_msgs::msg::Pose end)
{
  m_map_info = dist_map.info;

  Node start_node;
  Node current_node;

  navigation_msgs::msg::PathMsg m_path;
  navigation_msgs::msg::WorldPoint start_pose, end_pose;

  start_pose.x = start.position.x;
  start_pose.y = start.position.y;

  end_pose.x = end.position.x;
  end_pose.y = end.position.y;

  start_node.mpoint = m_dist_map_obj.World2Map2D(dist_map.info, start_pose);
  m_end_node.mpoint = m_dist_map_obj.World2Map2D(dist_map.info, end_pose);

  start_node.wpoint.yaw = start.position.z;
  m_end_node.wpoint.yaw = m_control_utils.Quat2Yaw(end.orientation);

  start_node.ind = m_dist_map_obj.Map2DTo1D(dist_map.info, start_node.mpoint);
  m_end_node.ind = m_dist_map_obj.Map2DTo1D(dist_map.info, m_end_node.mpoint);

  // Create neighbors
  CreateNeighbors();

  // Initialize both open and closed list
  std::map<int, Node> open_list;
  std::map<int, Node> closed_list;
  open_list[start_node.ind] = start_node;

  // Loop until you find the end
  while(open_list.size() > 0)
  {
    // Get the current node
    current_node = open_list.begin()->second;

    for(auto element : open_list)
    {
      if(element.second.f < current_node.f)
      {
        current_node = element.second;
      }
    }

    // VisNode(current_node, dist_map.info);

    open_list.erase(current_node.ind);
    closed_list[current_node.ind] = current_node;

    if(m_control_utils.DisBetweenMPoints(current_node.mpoint, m_end_node.mpoint) < m_goal_tolerance)
    {
      // Found the goal
      std::cout << "FINISHED" << std::endl;
      m_end_node.parent_ind = current_node.ind;
      m_path = BackTracking(closed_list, dist_map.info);
      return m_path;
    }

    // Loop through neighbors
    NeighborNode(current_node, dist_map);
    open_list = AppendToOpenList(open_list, closed_list, dist_map.info, current_node);
    m_neighbors.clear();
  }

  // If no path found, return empty path
  return navigation_msgs::msg::PathMsg{};
}
