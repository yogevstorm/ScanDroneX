#pragma once
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <std_msgs/msg/int8.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/map_meta_data.hpp>
#include "navigation_msgs/msg/world_point.hpp"
#include "navigation_msgs/msg/dist_map_msg.hpp"
#include <memory>
#include <map>
#include <vector>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <ControlUtils.hpp>

class DistMap
{

  public:
    DistMap(/* args */) {}
    ~DistMap() {}
    
    nav_msgs::msg::OccupancyGrid CreateLocalMap(nav_msgs::msg::OccupancyGrid p_occmap, geometry_msgs::msg::Pose p_pos, int p_occ_w, int p_occ_h);
    navigation_msgs::msg::DistMapMsg CreateDistMap(nav_msgs::msg::OccupancyGrid p_map, std::vector<int> p_free_vals);
    std::vector<int> World2Map2D(nav_msgs::msg::MapMetaData p_map_info, navigation_msgs::msg::WorldPoint p_point);
    int World2Map1D(nav_msgs::msg::MapMetaData p_map_info, navigation_msgs::msg::WorldPoint p_point);
    navigation_msgs::msg::WorldPoint Map2World1D(nav_msgs::msg::MapMetaData p_map_info, int p_map_ind);
    std::vector<int>  Map1DTo2D(nav_msgs::msg::MapMetaData p_map_info,  int p_map_ind);
    int Map2DTo1D(nav_msgs::msg::MapMetaData p_map_info, std::vector<int> p_map_coordinates);
    navigation_msgs::msg::WorldPoint Map2World2D(nav_msgs::msg::MapMetaData p_map_info, std::vector<int> p_map_coordinates);
    std::vector<float> GenCostMap(navigation_msgs::msg::DistMapMsg p_dist_map, navigation_msgs::msg::WorldPoint p_goal, float p_collision_radius);
    float GetDist(navigation_msgs::msg::WorldPoint p_point, navigation_msgs::msg::DistMapMsg p_dist_map);
    

  private:
    void assignValues(cv::Mat& p_dist_matrix, navigation_msgs::msg::DistMapMsg& p_dist_map, nav_msgs::msg::MapMetaData p_map_info);

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr m_pub_dist_map;
    ControlUtils m_control_utils;
       

  protected:
    
};