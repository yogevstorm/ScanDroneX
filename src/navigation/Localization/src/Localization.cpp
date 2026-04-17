#include <Localization.hpp>
using namespace std::chrono_literals;

void Localization::Init_Publishers_Subscribers()
{
  m_pub_drone_state_vis = m_node->create_publisher<visualization_msgs::msg::Marker>("drone_state_vis", 1);

  m_pub_drone_state = m_node->create_publisher<geometry_msgs::msg::Pose>("drone_pose", 1);

  m_tf_buffer = std::make_shared<tf2_ros::Buffer>(m_node->get_clock());
  m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer, m_node);

  m_timer = m_node->create_wall_timer(
      100ms, std::bind(&Localization::TimerCallback, this));
}

void Localization::TimerCallback()
{
  try {
    auto transform = m_tf_buffer->lookupTransform(
        "map", "simple_drone/base_footprint", tf2::TimePointZero);

    m_drone_state_msg.position.x = transform.transform.translation.x;
    m_drone_state_msg.position.y = transform.transform.translation.y;
    m_drone_state_msg.position.z = m_control_utils.Quat2Yaw(transform.transform.rotation) - M_PI / 2;

    CorrectDroneState();
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(m_node->get_logger(), *m_node->get_clock(), 1000,
        "TF lookup map->base_footprint failed: %s", ex.what());
  }
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
