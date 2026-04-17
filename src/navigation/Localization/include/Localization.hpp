#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_msgs/msg/int8.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "navigation_msgs/msg/world_point.hpp"
#include "navigation_msgs/msg/dist_map_msg.hpp"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>
#include <memory>
#include <map>
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include <ControlUtils.hpp>

class Localization
{
public:
  void Init_Publishers_Subscribers();

  std::shared_ptr<rclcpp::Node> m_node;

private:
  void CorrectDroneState();
  Eigen::Matrix3f CreateTransformationMatrix(float x, float y, float yaw);
  void DroneStateVis(geometry_msgs::msg::Pose drone_state);
  void TimerCallback();

  ControlUtils m_control_utils;

  std::shared_ptr<tf2_ros::Buffer> m_tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> m_tf_listener;
  rclcpp::TimerBase::SharedPtr m_timer;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr m_pub_drone_state_vis;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr m_pub_drone_state;

  geometry_msgs::msg::Pose m_drone_state, m_drone_state_msg;

  float m_x_offset = -0.2;
  float m_y_offset = -0.06;

protected:
};
