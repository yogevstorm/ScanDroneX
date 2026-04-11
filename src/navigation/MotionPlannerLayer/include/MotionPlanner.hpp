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
#include <memory>
#include <map>
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include <unsupported/Eigen/Splines>
#include <ControlUtils.hpp>


class MotionPlanner
{

  public:

    navigation_msgs::msg::PathMsg ProcessPath(navigation_msgs::msg::PathMsg r_path, navigation_msgs::msg::DroneState drone_state);

    float m_dis_ahead = 1.5;

    float m_ds = 0.1;

    float m_yaw_closest_idx_to_drone_weight = 1.0;

  private:  

    void Init(navigation_msgs::msg::DroneState drone_state);

    void ComputeArcLengthSamples(navigation_msgs::msg::PathMsg r_path);

    void SetXY(navigation_msgs::msg::PathMsg r_path_msg);

    void SetGear(navigation_msgs::msg::PathMsg r_path_msg);

    void SetYaw();

    void SetK();

    void SetS();

    void SetTrajMsg();

    void FindClosestIdxToDrone(navigation_msgs::msg::PathMsg r_path);

    Eigen::VectorXd leastSquaresPolynomial(const Eigen::VectorXd& x, const Eigen::VectorXd& y, int degree);

    float evaluateSpline(const Eigen::VectorXd& coefficients, float x);

    std::vector<float> diff(const std::vector<float>& v);

    std::vector<float> m_x, m_y, m_yaw, m_s, m_k;

    std::vector<int> m_gear;

    int m_spline_degree = 3;

    std::vector<float> m_dist_along, m_dist_along_inter;

    ControlUtils m_control_utils;

    navigation_msgs::msg::PathMsg m_trajectory;

    navigation_msgs::msg::DroneState m_drone_state;

    int m_closest_idx = 0;

  protected:
    
    
};
