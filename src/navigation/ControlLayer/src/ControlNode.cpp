#include <ControlNode.hpp>
using namespace std::chrono_literals;

void ControlNode::InitParams()
{
  m_node->declare_parameter<float>("MAX_STEER_ANGLE", 0.523);
  m_node->declare_parameter<float>("WHEEL_BASE", 0.26);
  m_node->declare_parameter<float>("LOOKAHEAD_DIS", 1.0);
  m_node->declare_parameter<float>("DS", 0.2);
  m_node->declare_parameter<float>("MAX_DIS_CAR_FROM_TRAJ", 5.0);
  m_node->declare_parameter<float>("V_MAX", 0.8);
  m_node->declare_parameter<float>("V_MIN", 0.2);
  m_node->declare_parameter<float>("K_GAIN", 1.0);
  m_node->declare_parameter<float>("YAW_K_GAIN", 2.0);
  m_node->declare_parameter<float>("MAX_ANGULAR", 2.0);
  m_node->declare_parameter<float>("CROSS_TRACK_KP", 1.0);
  m_node->declare_parameter<float>("CROSS_TRACK_KI", 0.0);
  m_node->declare_parameter<float>("CROSS_TRACK_KD", 0.0);
  m_node->declare_parameter<float>("MAX_LATERAL", 0.3);
  m_node->declare_parameter<float>("WALL_AVOID_K_GAIN", 3.0);
  m_node->declare_parameter<float>("MAX_WALL_LATERAL", 0.3);
  m_node->declare_parameter<float>("D_MAX", 2.0);
  m_node->declare_parameter<float>("Delta_D", 0.1);

}

void ControlNode::UpdateParams()
{
  m_node->get_parameter("MAX_STEER_ANGLE", m_control.m_max_steer_angle);
  m_node->get_parameter("WHEEL_BASE", m_control.m_wheel_base);
  m_node->get_parameter("LOOKAHEAD_DIS", m_control.m_lookahead_dis);
  m_node->get_parameter("DS", m_control.m_ds);
  m_node->get_parameter("MAX_DIS_CAR_FROM_TRAJ", m_max_dis_drone_from_traj);
  m_node->get_parameter("V_MAX", m_control.m_v_max);
  m_node->get_parameter("V_MIN", m_control.m_v_min);
  m_node->get_parameter("K_GAIN", m_control.m_k_gain);
  m_node->get_parameter("YAW_K_GAIN", m_control.m_yaw_k_gain);
  m_node->get_parameter("MAX_ANGULAR", m_control.m_max_angular);
  m_node->get_parameter("CROSS_TRACK_KP", m_control.m_cross_track_kp);
  m_node->get_parameter("CROSS_TRACK_KI", m_control.m_cross_track_ki);
  m_node->get_parameter("CROSS_TRACK_KD", m_control.m_cross_track_kd);
  m_node->get_parameter("MAX_LATERAL", m_control.m_max_lateral);
  m_node->get_parameter("WALL_AVOID_K_GAIN", m_control.m_wall_avoid_k_gain);
  m_node->get_parameter("MAX_WALL_LATERAL", m_control.m_max_wall_lateral);
  m_node->get_parameter("D_MAX", m_control.m_lane_width_max);
  m_node->get_parameter("Delta_D", m_control.m_lane_width_min);
}

void ControlNode::Init_Publishers_Subscribers()
{
  m_pub_cmd = m_node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
  m_sub_estop_scan = m_node->create_subscription<std_msgs::msg::Bool>(
    "estop/scan", 1,
    [this](const std_msgs::msg::Bool::SharedPtr msg) { m_estop_scan = msg->data; UpdateEstop(); }
  );
  m_sub_estop_local_planner = m_node->create_subscription<std_msgs::msg::Bool>(
    "estop/local_planner", 1,
    [this](const std_msgs::msg::Bool::SharedPtr msg) { m_estop_local_planner = msg->data; UpdateEstop(); }
  );
  m_sub_estop_mission = m_node->create_subscription<std_msgs::msg::Bool>(
    "estop/mission", 1,
    [this](const std_msgs::msg::Bool::SharedPtr msg) { m_estop_mission = msg->data; UpdateEstop(); }
  );
  m_sub_estop_behavior = m_node->create_subscription<std_msgs::msg::Bool>(
    "estop/behavior", 1,
    [this](const std_msgs::msg::Bool::SharedPtr msg) { m_estop_behavior = msg->data; UpdateEstop(); }
  );
  m_sub_state = m_node->create_subscription<navigation_msgs::msg::DroneState>(
    "drone_state", 1, std::bind(&ControlNode::StateCallBack, this, std::placeholders::_1)
  );
  m_sub_lane = m_node->create_subscription<navigation_msgs::msg::Lane>(
    "lane", 1, std::bind(&ControlNode::LaneCallBack, this, std::placeholders::_1)
  );
  m_sub_path = m_node->create_subscription<navigation_msgs::msg::PathMsg>(
    "trajectory", 10, std::bind(&ControlNode::TrajectoryCallBack, this, std::placeholders::_1)
  );
}

void ControlNode::UpdateEstop()
{
  m_estop = m_estop_scan || m_estop_local_planner || m_estop_mission || m_estop_behavior;
}

void ControlNode::StateCallBack(const navigation_msgs::msg::DroneState::SharedPtr msg)
{
  m_drone_state = *msg;
  m_is_recieved_state_msg = true;
}

void ControlNode::LaneCallBack(const navigation_msgs::msg::Lane::SharedPtr msg)
{
  m_control.FindNarrowCluster(*msg);
}

bool ControlNode::SafetyRequirements()
{
  // if(IsCarFarFromTraj())
  // {
  //   return false;
  // }

  if(m_trajectory.path_msg.empty())
  {
    return true;
  }

  if((rclcpp::Clock().now() - m_traj_timer).seconds() > 0.5)
  {
    return false;
  }

  if(m_estop)
  {
    return false;
  }

  if(m_trajectory.path_msg.size() < 2)
  {
    return false;
  }

  return true;
}

void ControlNode::TrajectoryCallBack(const navigation_msgs::msg::PathMsg::SharedPtr msg) 
{
  m_trajectory = *msg;

  m_traj_timer = rclcpp::Clock().now();

  if(!SafetyRequirements())
  {
    Estop();
    return;
  }

  if(m_is_recieved_state_msg)
  {
    std::vector<float> cmd = m_control.GetCmd(m_drone_state, m_trajectory);
    geometry_msgs::msg::Twist cmd_msg;
    cmd_msg.linear.x  = cmd[0];
    cmd_msg.angular.z = cmd[1];
    cmd_msg.linear.y  = cmd[2];
    m_pub_cmd->publish(cmd_msg);
  }

}

void ControlNode::Estop()
{
  m_pub_cmd->publish(geometry_msgs::msg::Twist());
}

bool ControlNode::IsDroneFarFromTraj()
{

  float dis_drone_from_traj = sqrt(pow((m_drone_state.x - m_trajectory.path_msg.front().x), 2)
  + pow((m_drone_state.y - m_trajectory.path_msg.front().y), 2));

  if(dis_drone_from_traj > m_max_dis_drone_from_traj)
  {
    return true;
  }

  return false;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("ControlNode");

  ControlNode control_node;

  control_node.m_node = node;

  control_node.InitParams();

  control_node.Init_Publishers_Subscribers();

  while(rclcpp::ok()) {

    control_node.UpdateParams();

    if(!control_node.SafetyRequirements())
    {
      control_node.Estop();
    }

    std::this_thread::sleep_for(20ms);
    
    rclcpp::spin_some(node);

  }

  rclcpp::shutdown();

  return 0;
}
