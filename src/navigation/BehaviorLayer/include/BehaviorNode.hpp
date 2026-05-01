#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/bool.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "navigation_msgs/msg/world_point.hpp"
#include "navigation_msgs/msg/dist_map_msg.hpp"
#include "navigation_msgs/msg/path_msg.hpp"
#include "navigation_msgs/msg/lane.hpp"
#include "navigation_msgs/msg/lane_cluster.hpp"
#include "navigation_msgs/msg/drone_state.hpp"
#include <memory>
#include <map>
#include <vector>
#include <cmath>

#include <BehaviorPlanner.hpp>
#include <ControlUtils.hpp>


class BehaviorNode
{

  public:

    void Init();

    void InitParams();

    void UpdateParams();

    void RunBehaviorNode();

    std::shared_ptr<rclcpp::Node> m_node;
    

  private:  

    void InitPublishers();

    void InitSubscribers();

    void MissionPathCallBack(const navigation_msgs::msg::PathMsg::SharedPtr msg);

    void DistMapCallBack(const navigation_msgs::msg::DistMapMsg::SharedPtr msg);

    void PoseCallBack(const geometry_msgs::msg::Pose::SharedPtr msg);

    void RunBehaviorPlanner();

    void VisLane();

    void CreateClusterMarker(navigation_msgs::msg::LaneCluster p_cluster, visualization_msgs::msg::Marker &p_r_margin_mkr,
      visualization_msgs::msg::Marker &p_l_margin_mkr);

    
    ControlUtils m_control_utils;

    BehaviorPlanner m_behavior_planner;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_pub_lane_vis;

    rclcpp::Publisher<navigation_msgs::msg::Lane>::SharedPtr m_pub_lane;

    rclcpp::Publisher<navigation_msgs::msg::DroneState>::SharedPtr m_pub_drone_state;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_pub_is_path_blocked;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_pub_is_out_of_lane;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_pub_estop;

    rclcpp::Subscription<navigation_msgs::msg::PathMsg>::SharedPtr m_sub_mission_path;

    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr m_sub_state_pose;

    rclcpp::Subscription<navigation_msgs::msg::DistMapMsg>::SharedPtr m_sub_dist_map;

    

  
    navigation_msgs::msg::PathMsg m_mission_path;

    navigation_msgs::msg::DistMapMsg m_dist_map;

    navigation_msgs::msg::DroneState m_drone_state;

    geometry_msgs::msg::Pose m_drone_state_pose_msg;

    bool m_is_recieved_drone_pose = false;

    bool m_is_recieved_m_path = false;

    bool m_is_recieved_dist_map = false;

  protected:
    
    
};