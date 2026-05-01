#pragma once
#include <memory>
#include <map>
#include <vector>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <LocalPlanner.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include "navigation_msgs/msg/lane.hpp"
#include "navigation_msgs/msg/drone_state.hpp"
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <ControlUtils.hpp>


class LocalPlannerNode
{
  public:

    void Init();

    void InitPublishers();

    void InitSubscribers();

    void InitParams();

    void UpdateParams();

    void RunLocalPlanner();

    std::shared_ptr<rclcpp::Node> m_node;

  private:

    void DroneVelCallBack(const std_msgs::msg::Float32::SharedPtr msg);

    void LaneCallBack(const navigation_msgs::msg::Lane::SharedPtr msg);

    void DistMapCallBack(const navigation_msgs::msg::DistMapMsg::SharedPtr msg);

    void DroneStateCallBack(const navigation_msgs::msg::DroneState::SharedPtr msg);

    void IsPathBlockedCallBack(const std_msgs::msg::Bool::SharedPtr msg);

    void PublishPathMsg();

    void PubEstop(bool estop);

    void VisualizePath(navigation_msgs::msg::PathMsg path);

    ControlUtils m_control_utils;

    LocalPlanner m_local_planner;

    rclcpp::Publisher<navigation_msgs::msg::PathMsg>::SharedPtr m_pub_local_path;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_pub_estop;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_pub_path_vis;

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr m_sub_drone_vel;

    rclcpp::Subscription<navigation_msgs::msg::Lane>::SharedPtr m_sub_lane;

    rclcpp::Subscription<navigation_msgs::msg::DistMapMsg>::SharedPtr m_sub_dist_map;

    rclcpp::Subscription<navigation_msgs::msg::DroneState>::SharedPtr m_sub_drone_state;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_sub_is_path_blocked;

    navigation_msgs::msg::Lane m_lane;

    navigation_msgs::msg::DistMapMsg m_dist_map;

    navigation_msgs::msg::DroneState m_drone_state;

    float m_drone_vel;

    bool m_is_recieved_lane        = false;
    bool m_is_recieved_dist_map    = false;
    bool m_is_recieved_drone_state = false;
    bool m_is_path_blocked         = false;


};