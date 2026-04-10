#pragma once
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "navigation_msgs/msg/world_point.hpp"
#include "navigation_msgs/msg/path_msg.hpp"
#include "navigation_msgs/msg/lane.hpp"
#include <memory>
#include <map>
#include <vector>
#include <cmath>

class ControlUtils
{

  public:

    geometry_msgs::msg::Quaternion Euler2Quat(double yaw, double pitch, double roll); 

    float Quat2Yaw(geometry_msgs::msg::Quaternion q);

    visualization_msgs::msg::Marker CreateVisWorldPoint(std::shared_ptr<rclcpp::Node> node, navigation_msgs::msg::WorldPoint WorldPoint,
    float color_r, float color_g, float color_b, float color_a = 1.0, float scale = 1.0,
    float color_rev_r = 1.0, float color_rev_g = 0.0, float color_rev_b = 0.0, float color_rev_a = 1.0);

    visualization_msgs::msg::Marker CreateVisCollisionR(std::shared_ptr<rclcpp::Node> node, geometry_msgs::msg::Pose pose, float collision_r, float r = 0.0,
    float g = 0.0, float b = 0.0, float a = 0.5);

    float DisBetweenWPoints(navigation_msgs::msg::WorldPoint WorldPoint1, navigation_msgs::msg::WorldPoint WorldPoint2);

    float DisBetweenMPoints(std::vector<int> MapPoint1, std::vector<int> MapPoint2);

    std::tuple<float, float> Cartesian2Curvlinear(navigation_msgs::msg::PathMsg mission_path, float x, float y);

    std::tuple<float, float> Cartesian2Curvlinear(navigation_msgs::msg::Lane lane, float x, float y);

    std::tuple<float, float> CurvilinearToCartesian(float s, float d);

    int ClosestIdx(navigation_msgs::msg::PathMsg mission_path, float x, float y);

    int ClosestIdx(navigation_msgs::msg::Lane lane, float x, float y);

    float Saturate(float val, float min_val,float max_val);

    navigation_msgs::msg::WorldPoint FindMPathWPointByS(float s, navigation_msgs::msg::PathMsg mission_path);

    int SIdxMPath(float s, float ds, navigation_msgs::msg::PathMsg mission_path);

    int SIdxLane(float s, float ds, navigation_msgs::msg::Lane lane);

    std::tuple<float, float> CurvilinearToCartesian(float s, float d, float ds, navigation_msgs::msg::PathMsg mission_path);

    std::vector<float> LinSpace(float start, float end, int num);

    float angle_wrap(float angle);

    float mod_2_pi(float angle);

    std::vector<float> diff(const std::vector<float>& v);


  private:



  protected:
    
};