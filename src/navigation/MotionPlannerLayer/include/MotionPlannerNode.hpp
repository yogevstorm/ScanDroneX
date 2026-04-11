#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/int8.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "navigation_msgs/msg/world_point.hpp"
#include "navigation_msgs/msg/dist_map_msg.hpp"
#include "navigation_msgs/msg/path_msg.hpp"
#include "navigation_msgs/msg/drone_state.hpp"
#include "MotionPlanner.hpp"
#include <memory>
#include <map>
#include <vector>
#include <cmath>
#include <ControlUtils.hpp>


class MotionPlannerNode
{

  public:

    void Init();

    void InitPublishers();

    void InitSubscribers();

    void InitParams();

    void UpdateParams();

    void RunMotionPlanner();

    std::shared_ptr<rclcpp::Node> m_node;

    navigation_msgs::msg::PathMsg m_trajectory, m_processed_lpath;

    rclcpp::Subscription<navigation_msgs::msg::PathMsg>::SharedPtr m_sub_lpath;

    

  private:  

    void InitTrajectory();

    void RoadPlanningCallBack(const navigation_msgs::msg::PathMsg::SharedPtr p_msg); 

    visualization_msgs::msg::Marker CreateVisWorldPoint(navigation_msgs::msg::WorldPoint p_WorldPoint);

    void CreateTraj();

    void VisTrajectory(navigation_msgs::msg::PathMsg p_path);

    void StateCallBack(const navigation_msgs::msg::DroneState::SharedPtr p_msg);

    MotionPlanner m_motion_planner;

    navigation_msgs::msg::DroneState m_drone_state_msg;

    ControlUtils m_control_utils;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_pub_path_vis;

    rclcpp::Publisher<navigation_msgs::msg::PathMsg>::SharedPtr m_pub_path;

    navigation_msgs::msg::PathMsg m_lpath;

    rclcpp::Subscription<navigation_msgs::msg::DroneState>::SharedPtr m_sub_state;

    bool m_is_unstructured = true;


  protected:
    
    
};
