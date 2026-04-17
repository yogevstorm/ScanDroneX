#include <ControlNode.hpp>
using namespace std::chrono_literals;

void ControlNode::InitParams()
{
  m_node->declare_parameter<float>("MAX_STEER_ANGLE", 0.523);
  m_node->declare_parameter<float>("WHEEL_BASE", 0.26);
  m_node->declare_parameter<float>("LOOKAHEAD_DIS", 1.3);
  m_node->declare_parameter<float>("DS", 0.2);
  m_node->declare_parameter<float>("MAX_DIS_CAR_FROM_TRAJ", 5.0);
  m_node->declare_parameter<float>("V_MAX", 0.8);
  m_node->declare_parameter<float>("V_MIN", 0.2);
  m_node->declare_parameter<float>("K_GAIN", 1.0);

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
}

void ControlNode::Init_Publishers_Subscribers()
{
  m_pub_cmd = m_node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
  m_sub_estop = m_node->create_subscription<std_msgs::msg::Bool>(
    "estop", 1, std::bind(&ControlNode::EstopCallBack, this, std::placeholders::_1)
  );
  m_sub_state = m_node->create_subscription<navigation_msgs::msg::DroneState>(
    "drone_state", 1, std::bind(&ControlNode::StateCallBack, this, std::placeholders::_1)
  );
  m_sub_path = m_node->create_subscription<navigation_msgs::msg::PathMsg>(
    "trajectory", 10, std::bind(&ControlNode::TrajectoryCallBack, this, std::placeholders::_1)
  );
}

void ControlNode::EstopCallBack(const std_msgs::msg::Bool::SharedPtr msg) 
{
  m_estop = msg->data;
}

void ControlNode::StateCallBack(const navigation_msgs::msg::DroneState::SharedPtr msg)
{
  m_drone_state = *msg;
  m_is_recieved_state_msg = true;
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
    cmd_msg.angular.z = -cmd[1];
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
