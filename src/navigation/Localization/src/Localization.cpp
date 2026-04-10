#include <Localization.hpp>
using namespace std::chrono_literals;

void Localization::Init_Publishers_Subscribers()
{
  m_pub_drone_state_vis = m_node->create_publisher<visualization_msgs::msg::Marker>("drone_state_vis", 1);

  m_pub_drone_state = m_node->create_publisher<geometry_msgs::msg::Pose>("drone_pose", 1);

  m_sub_state = m_node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "pose", 1, std::bind(&Localization::StateCallBack, this, std::placeholders::_1));
}

void Localization::StateCallBack(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  geometry_msgs::msg::PoseWithCovarianceStamped state_msg;
  state_msg = *msg;

  m_drone_state_msg.position.x = state_msg.pose.pose.position.x;
  m_drone_state_msg.position.y = state_msg.pose.pose.position.y;
  m_drone_state_msg.position.z = m_control_utils.Quat2Yaw(state_msg.pose.pose.orientation) - M_PI / 2;

  CorrectDroneState();
}

void Localization::CorrectDroneState()
{
  Eigen::Matrix3f W_2_R = CreateTransformationMatrix(
      m_drone_state_msg.position.x,
      m_drone_state_msg.position.y,
      m_drone_state_msg.position.z + M_PI_2);

  Eigen::Vector3f R_Point(m_x_offset, m_y_offset, 1);
  Eigen::Vector3f corrected_drone_state = W_2_R * R_Point;

  m_drone_state.position.x = corrected_drone_state(0);
  m_drone_state.position.y = corrected_drone_state(1);
  m_drone_state.position.z = m_drone_state_msg.position.z + M_PI_2;

  DroneStateVis(m_drone_state);

  m_pub_drone_state->publish(m_drone_state);
}

Eigen::Matrix3f Localization::CreateTransformationMatrix(float x, float y, float yaw)
{
  Eigen::Matrix3f transformationMatrix;

  // Rotation part
  transformationMatrix(0,0) = std::cos(yaw);
  transformationMatrix(0,1) = -std::sin(yaw);
  transformationMatrix(1,0) = std::sin(yaw);
  transformationMatrix(1,1) = std::cos(yaw);

  // Translation part
  transformationMatrix(0,2) = x;
  transformationMatrix(1,2) = y;

  // Homogeneous coordinate part
  transformationMatrix(2,0) = 0;
  transformationMatrix(2,1) = 0;
  transformationMatrix(2,2) = 1;

  return transformationMatrix;
}

void Localization::DroneStateVis(geometry_msgs::msg::Pose drone_state)
{
  visualization_msgs::msg::Marker drone_state_marker;
  drone_state_marker.header.frame_id = "map";

  rclcpp::Time now = m_node->get_clock()->now();
  drone_state_marker.header.stamp = now;

  drone_state_marker.type = 0;
  drone_state_marker.pose = drone_state;
  drone_state_marker.pose.position.z = 0.0;
  drone_state_marker.pose.orientation = m_control_utils.Euler2Quat(drone_state.position.z, 0, 0);

  drone_state_marker.color.b = 1;
  drone_state_marker.color.g = 1;
  drone_state_marker.color.a = 1.0;

  drone_state_marker.scale.x = 0.6;
  drone_state_marker.scale.y = 0.01;
  drone_state_marker.scale.z = 0.03;

  drone_state_marker.lifetime.sec = 0.5;

  m_pub_drone_state_vis->publish(drone_state_marker);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("Localization");

  Localization localization;     // local var: no underscore needed
  localization.m_node = node;

  localization.Init_Publishers_Subscribers();

  while(rclcpp::ok())
  {
    std::this_thread::sleep_for(20ms);
    rclcpp::spin_some(node);
  }

  rclcpp::shutdown();
  return 0;
}
