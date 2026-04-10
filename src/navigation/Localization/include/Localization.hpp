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
  void StateCallBack(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

  geometry_msgs::msg::Quaternion m_euler_2_quat(double yaw, double pitch, double roll);
  float m_quat_2_yaw(geometry_msgs::msg::Quaternion q);

  ControlUtils m_control_utils;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr m_pub_drone_state_vis;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr m_pub_drone_state;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr m_sub_state;

  geometry_msgs::msg::Pose m_drone_state, m_drone_state_msg;

  float m_x_offset = -0.2;
  float m_y_offset = -0.06;

protected:
};
