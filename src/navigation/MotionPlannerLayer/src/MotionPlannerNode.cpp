#include <MotionPlannerNode.hpp>
using namespace std::chrono_literals;

void MotionPlannerNode::Init()
{
  InitPublishers();
  InitSubscribers();
  InitParams();
}

void MotionPlannerNode::InitPublishers()
{
  m_pub_path_vis = m_node->create_publisher<visualization_msgs::msg::MarkerArray>("trajectory_vis", 1);
  m_pub_path = m_node->create_publisher<navigation_msgs::msg::PathMsg>("trajectory", 1);
}

void MotionPlannerNode::InitSubscribers()
{
  m_sub_state = m_node->create_subscription<navigation_msgs::msg::DroneState>(
      "drone_state", 1,
      std::bind(&MotionPlannerNode::StateCallBack, this, std::placeholders::_1));

  m_sub_lpath = m_node->create_subscription<navigation_msgs::msg::PathMsg>(
      "local_path", 10,
      std::bind(&MotionPlannerNode::RoadPlanningCallBack, this, std::placeholders::_1));
}

void MotionPlannerNode::InitParams()
{
  m_node->declare_parameter<float>("DIS_AHEAD", 1.5);
  m_node->declare_parameter<float>("DS", 0.1);
  m_node->declare_parameter<float>("YAW_CLOSEST_IDX_TO_DRONE_WEIGHT", 2.0);
}

void MotionPlannerNode::UpdateParams()
{
  m_node->get_parameter("DIS_AHEAD", m_motion_planner.m_dis_ahead);
  m_node->get_parameter("DS", m_motion_planner.m_ds);
  m_node->get_parameter("YAW_CLOSEST_IDX_TO_DRONE_WEIGHT", m_motion_planner.m_yaw_closest_idx_to_drone_weight);
}

void MotionPlannerNode::StateCallBack(const navigation_msgs::msg::DroneState::SharedPtr p_msg)
{
  m_drone_state_msg = *p_msg;
}

void MotionPlannerNode::RoadPlanningCallBack(const navigation_msgs::msg::PathMsg::SharedPtr p_msg) 
{
  m_lpath = *p_msg;
}

void MotionPlannerNode::InitTrajectory()
{
  m_processed_lpath.path_msg.clear();
  m_trajectory.path_msg.clear();
  m_processed_lpath = m_motion_planner.ProcessPath(m_lpath, m_drone_state_msg);
}

void MotionPlannerNode::RunMotionPlanner()
{
  if(m_lpath.path_msg.empty()) return;
  InitTrajectory();
  CreateTraj();
  VisTrajectory(m_trajectory);
  m_pub_path->publish(m_trajectory);
}

void MotionPlannerNode::CreateTraj()
{
  for(size_t i = 0; i < m_processed_lpath.path_msg.size(); i++)
  {
    m_trajectory.path_msg.push_back(m_processed_lpath.path_msg[i]);
  }
}


void MotionPlannerNode::VisTrajectory(navigation_msgs::msg::PathMsg p_path)
{
  visualization_msgs::msg::MarkerArray vis_path;

  for(int i = 0; i < p_path.path_msg.size(); i++)
  {
    visualization_msgs::msg::Marker WorldPoint =
        m_control_utils.CreateVisWorldPoint(m_node, p_path.path_msg[i],
                                            1.0, 0.54, 0.13, 0.3,
                                            2.0, 0.0, 1.0, 1.0, 0.3);

    WorldPoint.id = i + 2000;
    vis_path.markers.push_back(WorldPoint);
  }

  m_pub_path_vis->publish(vis_path);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("MotionPlanner");

  MotionPlannerNode motion_planner_node;
  
  motion_planner_node.m_node = node;

  motion_planner_node.Init();

  while(rclcpp::ok())
  {
    motion_planner_node.UpdateParams();
    motion_planner_node.RunMotionPlanner();
    std::this_thread::sleep_for(20ms);
    rclcpp::spin_some(node);
  }

  rclcpp::shutdown();

  return 0;
}
