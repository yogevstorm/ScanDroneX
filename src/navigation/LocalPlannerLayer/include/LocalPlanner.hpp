#pragma once
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/int8.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include "navigation_msgs/msg/drone_state.hpp"
#include "navigation_msgs/msg/world_point.hpp"
#include "navigation_msgs/msg/dist_map_msg.hpp"
#include "navigation_msgs/msg/path_msg.hpp"
#include "navigation_msgs/msg/lane.hpp"
#include "navigation_msgs/msg/lane_cluster.hpp"
#include <DistMap.hpp>
#include <memory>
#include <map>
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include <ControlUtils.hpp>
#include <HybridAstar.hpp>



class LocalPlanner
{
  public:

    void RunLocalPlanner(navigation_msgs::msg::Lane p_lane, navigation_msgs::msg::DistMapMsg p_dist_map,
      navigation_msgs::msg::DroneState p_drone_state, float p_drone_vel);

    HybridAstar m_hybrid_astar;

    bool m_estop = false;

    bool m_is_unstructured = false;

    float m_stuck_in_unstructured_relase_timer = 10.0;

    navigation_msgs::msg::PathMsg m_path;


  private:

    void Init(navigation_msgs::msg::Lane p_lane, navigation_msgs::msg::DistMapMsg p_dist_map,
      navigation_msgs::msg::DroneState p_drone_state, float p_drone_vel);

    void IsUnStructuredPlanning();

    void StructuredPlanning();

    void UnStructuredPlanning();

    void SetPathSegment();

    void PauseBetweenSegments();

    void StuckInUnstructuredRelease();

    void CreatePathSegments();

    navigation_msgs::msg::WorldPoint InitStartNode(navigation_msgs::msg::DroneState p_drone_state);

    

    ControlUtils m_control_utils;

    DistMap m_dist_map_obj;

    navigation_msgs::msg::PathMsg m_path_unstructured;

    navigation_msgs::msg::Lane m_lane;

    navigation_msgs::msg::DistMapMsg m_dist_map;

    navigation_msgs::msg::WorldPoint m_drone_state;

    navigation_msgs::msg::WorldPoint m_start_wpoint;

    float m_drone_vel = 0.0;

    std::vector<navigation_msgs::msg::PathMsg> m_path_segments;

    rclcpp::Time m_start_time;

    rclcpp::Time m_stuck_in_unstructured_start_time;

    std::vector<float> m_costmap;

    int m_planning_failers_counter = 0;



    
};