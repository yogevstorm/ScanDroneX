#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <std_msgs/msg/int8.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <memory>
#include <map>
#include <unordered_map>
#include <queue>
#include <vector>
#include <cmath>
#include "DistMap.hpp"
#include <Node.hpp>
#include <ControlUtils.hpp>

using OpenPQ = std::priority_queue<std::pair<float,int>, std::vector<std::pair<float,int>>, std::greater<std::pair<float,int>>>;


class HybridAstar
{

  public:

    navigation_msgs::msg::PathMsg Search(navigation_msgs::msg::DistMapMsg p_dist_map, navigation_msgs::msg::WorldPoint p_start,
      navigation_msgs::msg::Lane p_lane);
    std::shared_ptr<rclcpp::Node> m_node;

    

    std::vector<double> m_collision_pnts = {0.0, 0.2};
    float m_wheel_base = 0.26;
    float m_heuristic_gain = 1.0;
    int m_ang_n = 36;
    float m_k_reverse_cost = 0.3; //0.17;
    float m_k_gear_change_cost = 0.1;
    float m_downsample_ratio = 2.0;
    float m_ds = 0.2;
    float m_collision_r = 0.4;
    float m_goal_tolerance = 0.1;
    int m_counter = 0;
    float m_d_cost = 1.0;
    bool m_do_preempt = true;
    float m_timeout = 0.3;
    float m_dist_planning_ahead = 2.0;
    float m_yawe_cost = 0.0;
    float m_clearance_cost = 0.0;
    float m_yaw_tolerance = 0.1;
    float m_min_segment_len = 0.0;
    int m_max_segments_num = 5;
    float m_max_steer_angle = 30.0;
    float m_path_ds = 0.1;
    float m_max_reverse_len = 0.5;
    float m_max_forward_len = 1.0;
    float m_max_reverse_steer_angle = 30.0;
    
    
    
  private:
    bool IsNodeCollide(Node p_node);
    bool IsNodeOutOfBounds(Node p_node);
    bool IsNodeOutOfLane(Node p_node);
    void NeighborsNode(std::vector<Node>& p_neighbors, nav_msgs::msg::MapMetaData p_map_info, Node p_parent_node,
        int p_num_neighbors, double p_max_steer, float p_ds);
    Node KinematicStep(Node p_parent_node, double p_steer, float p_ds,  bool p_is_reverse);
    float GetHeuristic(Node p_node);
    bool IsDestination(Node p_node);
    void BackTrackNodes(Node p_end_node, const std::unordered_map<int, Node>& p_closed_list);
    void DeleteInvalidNeighbors(std::vector<Node>& p_neighbors);
    void AppendNeighborsToOpenList(std::unordered_map<int, Node>& p_open_list, OpenPQ& p_open_pq,
        const std::unordered_map<int, Node>& p_closed_list, const std::vector<Node>& p_neighbors);
    int FlatIndex(nav_msgs::msg::MapMetaData p_map_info, navigation_msgs::msg::WorldPoint p_wpoint);
    void Init(navigation_msgs::msg::DistMapMsg p_dist_map, navigation_msgs::msg::WorldPoint p_start,
      navigation_msgs::msg::Lane p_lane);
    navigation_msgs::msg::PathMsg PreemptedPath(Node& p_node, const std::unordered_map<int, Node>& p_closed_list);

    ControlUtils m_control_utils;
    navigation_msgs::msg::DistMapMsg m_dist_map;
    navigation_msgs::msg::Lane m_lane;
    Node m_start_node;
    Node m_end_node;
    navigation_msgs::msg::PathMsg m_path;
    DistMap m_dist_mapobj;
    



    
  protected:
    
};