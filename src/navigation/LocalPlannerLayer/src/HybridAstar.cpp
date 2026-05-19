#include <HybridAstar.hpp>


void HybridAstar::BackTrackNodes(Node p_end_node, const std::unordered_map<int, Node>& p_closed_list)
{
  Node curr_node = p_end_node;
  m_path.path_msg.clear();

  while(curr_node.parent_ind!=-1){
    navigation_msgs::msg::WorldPoint path_point;
    path_point = curr_node.wpoint;
    m_path.path_msg.push_back(path_point);
    curr_node = p_closed_list.at(curr_node.parent_ind);
  }  
  m_start_node.wpoint.gear = m_path.path_msg.back().gear;
  m_path.path_msg.push_back(m_start_node.wpoint);
  std::reverse(m_path.path_msg.begin(), m_path.path_msg.end());
}



Node HybridAstar::KinematicStep(Node p_parent_node, double p_steer, float p_ds, bool p_is_reverse)
{

  float k = std::tan(p_steer) / m_wheel_base;
  Node neighbor;
  if(p_is_reverse)
  {
    neighbor.wpoint.yaw = p_parent_node.wpoint.yaw - p_ds*float(k);
    neighbor.wpoint.x = p_parent_node.wpoint.x - p_ds*cos(neighbor.wpoint.yaw);
    neighbor.wpoint.y = p_parent_node.wpoint.y - p_ds*sin(neighbor.wpoint.yaw);
    neighbor.wpoint.gear = -1;
  }

  else
  {
    neighbor.wpoint.yaw = p_parent_node.wpoint.yaw + p_ds*float(k);
    neighbor.wpoint.x = p_parent_node.wpoint.x + p_ds*cos(neighbor.wpoint.yaw);
    neighbor.wpoint.y = p_parent_node.wpoint.y + p_ds*sin(neighbor.wpoint.yaw);
    neighbor.wpoint.gear = 1;
  }


  return neighbor;
}


bool HybridAstar::IsDestination(Node p_node)
{
  float yaw_e = m_control_utils.angle_wrap(p_node.wpoint.yaw - m_lane.clusters.back().yaw);

  if(GetHeuristic(p_node) < m_goal_tolerance && std::abs(yaw_e) < m_yaw_tolerance)
  {
    return true;
  }
  return false;
}

float HybridAstar::GetHeuristic(Node p_node)
{
  float goal_s = std::min((m_end_node.wpoint.s), (m_lane.clusters.back().s));

  return (abs(goal_s - p_node.wpoint.s));
}




void HybridAstar::NeighborsNode(std::vector<Node>& p_neighbors, nav_msgs::msg::MapMetaData p_map_info, Node p_parent_node,
  int p_num_neighbors, double p_max_steer, float p_ds){
  for(int i=-p_num_neighbors/2; i<=p_num_neighbors/2; i++)
  {
    Node neighbor = KinematicStep(p_parent_node, (i/(p_num_neighbors/2.0))*p_max_steer, p_ds, false);
    neighbor.parent_ind = p_parent_node.ind;
    neighbor.ind = FlatIndex(m_dist_map.info, neighbor.wpoint);
    auto [s, d] = m_control_utils.Cartesian2Curvlinear(m_lane, neighbor.wpoint.x, neighbor.wpoint.y);
    int cluster_idx = m_control_utils.SIdxLane(s, m_path_ds, m_lane);
    float yawe = m_control_utils.angle_wrap(neighbor.wpoint.yaw - m_lane.clusters[cluster_idx].yaw);
    float clearance = std::min((std::abs(d - m_lane.clusters[cluster_idx].dr)), (std::abs(d - m_lane.clusters[cluster_idx].dl)));
    float cost = (1 + m_d_cost*std::abs(d)) * (1 + m_yawe_cost*std::abs(yawe)) * (1 + m_clearance_cost*std::abs(1/(clearance + 0.0001)));
    neighbor.wpoint.s = s; neighbor.wpoint.d = d; 
    neighbor.g = p_parent_node.g + p_ds;
    neighbor.h = m_heuristic_gain*GetHeuristic(neighbor);
    neighbor.f = neighbor.g + neighbor.h;
    neighbor.f =  neighbor.f * (1 + p_ds * cost);
    p_neighbors.push_back(neighbor);
  }
}




bool HybridAstar::IsNodeOutOfBounds(Node p_node)
{
  p_node.mpoint = m_dist_mapobj.World2Map2D(m_dist_map.info, p_node.wpoint);
  if (p_node.mpoint[0] > (m_dist_map.info.width) || p_node.mpoint[0] < 0 ||
      p_node.mpoint[1] > (m_dist_map.info.height) || p_node.mpoint[1] < 0){
    return true;
  }
  return false;
}

bool HybridAstar::IsNodeCollide(Node p_node)
{
  float yaw = p_node.wpoint.yaw;
  float collision_r = m_collision_r;

  for(size_t i = 0; i < m_collision_pnts.size(); i++)
  {
    navigation_msgs::msg::WorldPoint collision_pnt;
    collision_pnt.x = p_node.wpoint.x  + m_collision_pnts[i] * cos(yaw);
    collision_pnt.y = p_node.wpoint.y  + m_collision_pnts[i] * sin(yaw);
    int map_cell_id = m_dist_mapobj.World2Map1D(m_dist_map.info, collision_pnt); 
    if(m_dist_map.data[map_cell_id] < collision_r)
    {
      return true;
    }
  }
  return false;
}

bool HybridAstar::IsNodeOutOfLane(Node p_node)
{
  int cluster_idx = m_control_utils.SIdxLane(p_node.wpoint.s, m_path_ds, m_lane);

  if((p_node.wpoint.d < m_lane.clusters[cluster_idx].dr) || (p_node.wpoint.d > m_lane.clusters[cluster_idx].dl))
  {
    return true;
  }
  return false;
}

void HybridAstar::DeleteInvalidNeighbors(std::vector<Node>& p_neighbors)
{

  std::vector<Node> selected_neighbors;

  for(int i = 0; i < p_neighbors.size(); i++)
  {
    if(!(IsNodeOutOfBounds(p_neighbors[i])
    || IsNodeCollide(p_neighbors[i])
    || IsNodeOutOfLane(p_neighbors[i])))
    {
      selected_neighbors.push_back(p_neighbors[i]);
    }
  }
  p_neighbors = selected_neighbors;
}



void HybridAstar::AppendNeighborsToOpenList(std::unordered_map<int, Node>& p_open_list, OpenPQ& p_open_pq,
                      const std::unordered_map<int, Node>& p_closed_list, const std::vector<Node>& p_neighbors)
{
  for (const auto& neighbor : p_neighbors)
  {
    if (p_closed_list.count(neighbor.ind))
      continue;

    auto it = p_open_list.find(neighbor.ind);
    if (it != p_open_list.end() && neighbor.f >= it->second.f)
      continue;

    p_open_list[neighbor.ind] = neighbor;
    p_open_pq.push({neighbor.f, neighbor.ind});
  }
}

int HybridAstar::FlatIndex(nav_msgs::msg::MapMetaData p_map_info, navigation_msgs::msg::WorldPoint p_wpoint){
  float yaw_sector = (2.0 * M_PI) / m_ang_n;
  int yawi = m_control_utils.mod_2_pi(p_wpoint.yaw + yaw_sector / 2) / yaw_sector;
  std::vector<int> mpoint = m_dist_mapobj.World2Map2D(p_map_info, p_wpoint);

  return int(mpoint[0]/ m_downsample_ratio) + int(p_map_info.width/ m_downsample_ratio) *
                (int(mpoint[1]/ m_downsample_ratio) + int(p_map_info.height/m_downsample_ratio) * yawi);
}


void HybridAstar::Init(navigation_msgs::msg::DistMapMsg p_dist_map, navigation_msgs::msg::WorldPoint p_start,
   navigation_msgs::msg::Lane p_lane)
{
  m_dist_map = p_dist_map;
  m_lane = p_lane;
  m_start_node.wpoint = p_start;
  m_end_node.wpoint.x = p_lane.clusters.back().x; m_end_node.wpoint.y = p_lane.clusters.back().y;
  m_end_node.wpoint.yaw = p_lane.clusters.back().yaw; m_end_node.wpoint.s = p_lane.clusters.back().s;
  m_start_node.ind = FlatIndex(p_dist_map.info, m_start_node.wpoint);
  m_start_node.wpoint.gear = 0;
}



navigation_msgs::msg::PathMsg HybridAstar::PreemptedPath(Node& p_node, const std::unordered_map<int, Node>& p_closed_list)
{
  BackTrackNodes(p_node, p_closed_list);
  return m_path;
}


navigation_msgs::msg::PathMsg HybridAstar::Search(navigation_msgs::msg::DistMapMsg p_dist_map, navigation_msgs::msg::WorldPoint p_start,
   navigation_msgs::msg::Lane p_lane){

  // Create start and end nodes
  Node current_node, best_node;
  //Create neighbors
  Init(p_dist_map, p_start, p_lane);
  float timeout = m_timeout;
  best_node = m_start_node;
  //Initialize both open and closed list
  int64 start_t = cv::getTickCount();
  std::unordered_map<int, Node> open_list, closed_list;
  OpenPQ open_pq;
  open_list[m_start_node.ind] = m_start_node;
  open_pq.push({m_start_node.f, m_start_node.ind});
  std::vector<Node> neighbors;
  float dt = 0.0;

  while (!open_list.empty() && dt < timeout){

    neighbors.clear();

    // skip stale pq entries (node was updated or already closed)
    while (!open_pq.empty()) {
      auto [f, ind] = open_pq.top();
      auto it = open_list.find(ind);
      if (it != open_list.end() && it->second.f == f) break;
      open_pq.pop();
    }
    if (open_pq.empty()) break;

    auto [f, ind] = open_pq.top(); open_pq.pop();
    current_node = open_list[ind];
    open_list.erase(ind);
    closed_list[current_node.ind] = current_node;

    if (m_do_preempt && current_node.wpoint.s >= best_node.wpoint.s)
      best_node = current_node;

    if(IsDestination(current_node)){
      BackTrackNodes(current_node, closed_list);
      return m_path;
    }

    NeighborsNode(neighbors, p_dist_map.info, current_node, 7, m_max_steer_angle * (M_PI/180.0), m_ds);
    DeleteInvalidNeighbors(neighbors);
    AppendNeighborsToOpenList(open_list, open_pq, closed_list, neighbors);

    dt = (cv::getTickCount() - start_t)/cv::getTickFrequency();
  }

  if (m_do_preempt && best_node.parent_ind>0)
  {
    return PreemptedPath(best_node, closed_list);
  }

  if(open_list.empty())
  {
    navigation_msgs::msg::PathMsg empty_path;
    m_path = empty_path;
  }

  return m_path;
}









