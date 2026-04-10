#include <rclcpp/rclcpp.hpp>
#include "navigation_msgs/msg/world_point.hpp"
#include "navigation_msgs/msg/drone_state.hpp"
#include "navigation_msgs/msg/dist_map_msg.hpp"
#include "navigation_msgs/msg/path_msg.hpp"
#include "navigation_msgs/msg/lane.hpp"
#include "navigation_msgs/msg/lane_cluster.hpp"
#include <DistMap.hpp>
#include <LaneNode.hpp>
#include <memory>
#include <map>
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include <ControlUtils.hpp>

class BehaviorPlanner
{
  public:

    void RunBehaviorPlanner(navigation_msgs::msg::PathMsg p_m_path, navigation_msgs::msg::DistMapMsg p_dist_map,
      navigation_msgs::msg::DroneState p_drone_state);

    navigation_msgs::msg::Lane m_lane;

    float m_d_max_param = 1.0;

    float m_dd_param = 0.1;

    float m_max_lane_len_param = 3.0;

    float m_ds_param = 0.01;

    float m_collision_r_param = 0.02;

    float m_wheel_base_param = 0.26;

    float m_k_cost_d_param = 1.0;

    float m_k_cost_width_param = 1.0;


  private:

    void Init(navigation_msgs::msg::PathMsg p_m_path, navigation_msgs::msg::DistMapMsg p_dist_map,
      navigation_msgs::msg::DroneState p_drone_state); 

    void SetStaticConstrains();

    void SetLaneClusters();

    void SearchLane();

    void InitStartNode(LaneNode& p_start_node);

    void NeighborNodes(LaneNode p_current_node, std::vector<LaneNode>& p_neighbors);

    void DeleteInvalidNodes(LaneNode p_current_node, std::vector<LaneNode>& p_neighbors);

    void AppendNeighborsToOpenList(std::map<int, LaneNode>& p_open_list, std::map<int, LaneNode> p_closed_list, std::vector<LaneNode> p_neighbors);

    void BackTrackNodes(LaneNode p_end_node, std::map<int, LaneNode> p_closed_list);

    void SmoothLane();

    void FindBlockedEndNode(LaneNode& p_best_node, std::map<int, LaneNode> p_closed_list);


    ControlUtils m_control_utils;

    DistMap m_dist_map_obj;

    navigation_msgs::msg::PathMsg m_m_path;

    navigation_msgs::msg::DistMapMsg m_dist_map;

    navigation_msgs::msg::DroneState m_drone_state;

    std::vector<float> m_lane_d_vec_m;

    std::vector<std::vector<int>> m_lane_constrains;

    std::vector<std::vector<std::tuple<float, float>>> m_lane_clusters;
};