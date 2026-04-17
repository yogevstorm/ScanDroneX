#include <BehaviorPlanner.hpp>
using namespace std::chrono_literals;

void BehaviorPlanner::RunBehaviorPlanner(navigation_msgs::msg::PathMsg p_m_path, navigation_msgs::msg::DistMapMsg p_dist_map,
    navigation_msgs::msg::DroneState p_drone_state)
{
    Init(p_m_path, p_dist_map, p_drone_state);

    SetStaticConstrains();

    SetLaneClusters();

    SearchLane();

    SmoothLane();
}

void BehaviorPlanner::Init(navigation_msgs::msg::PathMsg p_m_path, navigation_msgs::msg::DistMapMsg p_dist_map,
      navigation_msgs::msg::DroneState p_drone_state)
{
    m_m_path = p_m_path;

    m_dist_map = p_dist_map;

    m_drone_state = p_drone_state;

    m_lane_d_vec_m = m_control_utils.LinSpace(-m_d_max_param, m_d_max_param, int(2*m_d_max_param/ m_dd_param + 0.5));

    m_lane_constrains.clear();

    m_lane_clusters.clear();
}


void BehaviorPlanner::SetStaticConstrains()
{
    int max_m_path_iterations = int(m_m_path.path_msg.size() - m_control_utils.SIdxMPath(m_drone_state.s, m_ds_param, m_m_path) -1);

    int s_iterations = std::min(int(m_max_lane_len_param/m_ds_param), max_m_path_iterations);

    for(int s_idx = 0; s_idx < s_iterations; s_idx++)
    {
        std::vector<int> lane_constrains_row;

        for(size_t d_idx = 0; d_idx < m_lane_d_vec_m.size(); d_idx++)
        {
            float s = std::max(m_drone_state.s, 0.0f) + s_idx * m_ds_param;

            auto [x, y] = m_control_utils.CurvilinearToCartesian(s, m_lane_d_vec_m[d_idx], m_ds_param, m_m_path);

            navigation_msgs::msg::WorldPoint wpoint;

            wpoint.x = x; wpoint.y = y;

            float clearance = m_dist_map_obj.GetDist(wpoint, m_dist_map);

            if(clearance < m_collision_r_param)
            {
                lane_constrains_row.push_back(1);
            }

            else
            {
                lane_constrains_row.push_back(0);
            }
        }

        m_lane_constrains.push_back(lane_constrains_row);
    }
}

void BehaviorPlanner::SetLaneClusters()
{
    for(size_t s_idx = 0; s_idx < m_lane_constrains.size(); s_idx++)
    {
        std::vector<float> current_cluster;

        std::vector<std::tuple<float, float>> clusters_boundaries;

        bool is_first_time_on_obs = true;

        for(size_t d_idx = 0; d_idx <= m_lane_constrains[s_idx].size(); d_idx++)
        {
            if(m_lane_constrains[s_idx][d_idx] < 1 && d_idx < m_lane_constrains[s_idx].size())
            {
                current_cluster.push_back(m_lane_d_vec_m[d_idx]);
                is_first_time_on_obs = true;
            }

            else
            {
                if(is_first_time_on_obs && !current_cluster.empty())
                {
                    std::tuple<float, float> cluster_boundaries;

                    cluster_boundaries = std::make_tuple(current_cluster.front(), current_cluster.back());

                    clusters_boundaries.push_back(cluster_boundaries);

                    current_cluster.clear();

                    is_first_time_on_obs = false;

                }
            }

        }

        m_lane_clusters.push_back(clusters_boundaries);
    }
}


void BehaviorPlanner::SearchLane()
{
    if(m_lane_clusters.empty()) return;

    LaneNode start_node, best_node;

    start_node.ind = 0; 

    InitStartNode(start_node);

    std::map<int, LaneNode> open_list; std::map<int, LaneNode> closed_list;

    open_list[start_node.ind] = start_node;

    int end_lane_idx = std::min(int(m_max_lane_len_param/m_ds_param), int(m_m_path.path_msg.size() - 1 - m_control_utils.SIdxMPath(m_drone_state.s, m_ds_param, m_m_path)));

    std::vector<LaneNode> neighbors;

    while (!open_list.empty())
    {
        neighbors.clear();

        LaneNode current_node = open_list.begin()->second;

        if((current_node.lane_idx > best_node.lane_idx) 
        || (current_node.lane_idx == best_node.lane_idx && current_node.f < best_node.f))
        {
            best_node = current_node;
        }

        for(auto element :open_list){
            if(element.second.f < current_node.f){
                current_node = element.second;
            }
        }

        open_list.erase(current_node.ind);

        closed_list[current_node.ind] = current_node;

        if(current_node.lane_idx == end_lane_idx - 1){
            //  Found the goal
            m_is_path_blocked = false;
            BackTrackNodes(current_node, closed_list);
            return;
        }
        
        NeighborNodes(current_node, neighbors);

        DeleteInvalidNodes(current_node, neighbors);

        AppendNeighborsToOpenList(open_list, closed_list, neighbors);
    }

    m_is_path_blocked = true;
    FindBlockedEndNode(best_node, closed_list);

    BackTrackNodes(best_node, closed_list);

    return;
}

void BehaviorPlanner::InitStartNode(LaneNode& p_start_node)
{
    float min_dis = 999.9;

    for(size_t i = 0; i < m_lane_clusters[0].size(); i++)
    {
        float cluster_center = (std::get<0>(m_lane_clusters[0][i]) + std::get<1>(m_lane_clusters[0][i]))/2.0;

        float lateral_dis_drone_from_cluster = std::abs(cluster_center - m_drone_state.d);

        if(lateral_dis_drone_from_cluster < min_dis
        && !((std::get<0>(m_lane_clusters[0][i]) > m_drone_state.d) || (std::get<1>(m_lane_clusters[0][i]) < m_drone_state.d)))
        {
            min_dis = lateral_dis_drone_from_cluster;
            p_start_node.cluster = m_lane_clusters[0][i];
        }
    }    
}

void BehaviorPlanner::NeighborNodes(LaneNode p_current_node, std::vector<LaneNode>& p_neighbors)
{
    for(size_t cluster_idx = 0; cluster_idx < m_lane_clusters[p_current_node.lane_idx + 1].size(); cluster_idx++)
    {
        LaneNode neighbor;
        int neighbor_lane_idx = std::min(p_current_node.lane_idx + 1, int(m_lane_clusters.size() - 1));
        neighbor.cluster = m_lane_clusters[neighbor_lane_idx][cluster_idx];
        neighbor.lane_idx = p_current_node.lane_idx + 1;
        neighbor.ind = neighbor.lane_idx*int(m_max_lane_len_param/m_ds_param) + cluster_idx;
        neighbor.parent_ind = p_current_node.ind;
        float cost_cluster_width = 1.0/(std::abs(std::get<0>(neighbor.cluster) - std::get<1>(neighbor.cluster)));
        float cost_cluster_d = std::abs((std::get<0>(neighbor.cluster) + std::get<1>(neighbor.cluster))/2.0f - m_drone_state.d);
        neighbor.f = m_k_cost_width_param * cost_cluster_width + m_k_cost_d_param * cost_cluster_d +  p_current_node.f;
        p_neighbors.push_back(neighbor);
    }
}

void BehaviorPlanner::DeleteInvalidNodes(LaneNode p_current_node, std::vector<LaneNode>& p_neighbors)
{
    std::vector<LaneNode> selected_neighbors;

    for(auto neighbor :p_neighbors)
    {
        if(!((std::get<0>(neighbor.cluster) > std::get<1>(p_current_node.cluster))
          || (std::get<1>(neighbor.cluster) < std::get<0>(p_current_node.cluster))
          || std::get<1>(neighbor.cluster) - std::get<0>(neighbor.cluster) < 0.01))
        {
            selected_neighbors.push_back(neighbor);
        }
    }
    p_neighbors = selected_neighbors;
}


void BehaviorPlanner::AppendNeighborsToOpenList(std::map<int, LaneNode>& p_open_list, std::map<int, LaneNode> p_closed_list,
                      std::vector<LaneNode> p_neighbors)
{
  for(auto neighbor :p_neighbors)
  {
    //neighbor is on the closed list
    bool is_neighbor_in_close_list = false;

    bool is_neighbor_in_open_list = false;
    
    for (auto element :p_closed_list)
    {
      if(neighbor.ind == element.first){
        is_neighbor_in_close_list = true;
        break;
      }
    }

    if(is_neighbor_in_close_list){
      is_neighbor_in_close_list = false;
      continue;
    }

    //Child is already in the open list
    
    for (auto element :p_open_list){

      if(neighbor.ind == element.first && neighbor.f > element.second.f){
        is_neighbor_in_open_list = true;
        break;
      }
    }

    if(is_neighbor_in_open_list){
      is_neighbor_in_open_list = false;
      continue;
    }

    //Add the child to the open list
    p_open_list[neighbor.ind] = neighbor;
  }
}


void BehaviorPlanner::BackTrackNodes(LaneNode p_end_node, std::map<int, LaneNode> p_closed_list)
{
  LaneNode curr_node = p_end_node;
  m_lane.clusters.clear();
  int drone_m_path_idx = m_control_utils.SIdxMPath(m_drone_state.s, m_ds_param, m_m_path);

  while(curr_node.parent_ind!=-1){
    navigation_msgs::msg::LaneCluster lane_cluster;
    lane_cluster.dr = std::get<0>(curr_node.cluster); 
    lane_cluster.dl = std::get<1>(curr_node.cluster);

    lane_cluster.x = m_m_path.path_msg[drone_m_path_idx + curr_node.lane_idx].x;
    lane_cluster.y = m_m_path.path_msg[drone_m_path_idx + curr_node.lane_idx].y;
    lane_cluster.yaw = m_m_path.path_msg[drone_m_path_idx + curr_node.lane_idx].yaw;
    lane_cluster.s = m_m_path.path_msg[drone_m_path_idx + curr_node.lane_idx].s;    

    m_lane.clusters.push_back(lane_cluster);
    curr_node = p_closed_list[curr_node.parent_ind];
  }  

  std::reverse(m_lane.clusters.begin(), m_lane.clusters.end());

}

void BehaviorPlanner::FindBlockedEndNode(LaneNode& p_best_node, std::map<int, LaneNode> p_closed_list)
{
    int blocked_lane_idx = p_best_node.lane_idx - int(m_wheel_base_param/m_ds_param);
    while(p_best_node.lane_idx > blocked_lane_idx)
    {
        p_best_node = p_closed_list[p_best_node.parent_ind];
        if(p_best_node.lane_idx < 2) return;
    }
}


void BehaviorPlanner::SmoothLane()
{
    if(m_lane.clusters.empty()) return;

    float max_smooth = 1.0;

    bool finish_smoothing = false;

    while(!finish_smoothing)
    {
        finish_smoothing = true;

        for(size_t i = 0; i < m_lane.clusters.size() - 1; i++)
        {
            float gradient_r = (m_lane.clusters[i + 1].dr - m_lane.clusters[i].dr)/(m_lane.clusters[i + 1].s - m_lane.clusters[i].s);

            if(gradient_r > max_smooth)
            {
                m_lane.clusters[i].dr = m_lane.clusters[i + 1].dr - max_smooth*0.5*(m_lane.clusters[i + 1].s - m_lane.clusters[i].s);
                finish_smoothing = false;
            }

            if(gradient_r < -max_smooth)
            {
                m_lane.clusters[i + 1].dr = m_lane.clusters[i].dr - max_smooth*0.5*(m_lane.clusters[i + 1].s - m_lane.clusters[i].s);
            }

            float gradient_l = (m_lane.clusters[i + 1].dl - m_lane.clusters[i].dl)/(m_lane.clusters[i + 1].s - m_lane.clusters[i].s);

            if(gradient_l < -max_smooth)
            {
                m_lane.clusters[i].dl = m_lane.clusters[i + 1].dl + max_smooth*0.5*(m_lane.clusters[i + 1].s - m_lane.clusters[i].s);
                finish_smoothing = false;
            }

            if(gradient_l > max_smooth)
            {
                m_lane.clusters[i + 1].dl = m_lane.clusters[i].dl + max_smooth*0.5*(m_lane.clusters[i + 1].s - m_lane.clusters[i].s);
                finish_smoothing = false;
            }
        }
    }
}
