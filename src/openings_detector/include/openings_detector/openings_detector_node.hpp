#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

class OpeningsDetectorNode : public rclcpp::Node
{
public:
  OpeningsDetectorNode();

private:
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr  goal_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;

  int min_opening_width_;
};
