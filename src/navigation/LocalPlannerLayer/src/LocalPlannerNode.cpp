#include <LocalPlannerNode.hpp>
using namespace std::chrono_literals;

void LocalPlannerNode::Init()
{
  InitPublishers();
  InitSubscribers();
  InitParams();
}

void LocalPlannerNode::InitPublishers()
{
  m_pub_local_path       = m_node->create_publisher<navigation_msgs::msg::PathMsg>("local_path", 1);
  m_pub_estop            = m_node->create_publisher<std_msgs::msg::Bool>("estop/local_planner", false);
  m_pub_path_vis         = m_node->create_publisher<visualization_msgs::msg::MarkerArray>("road_path_vis", 1);
}

void LocalPlannerNode::InitSubscribers()
{
  m_sub_drone_vel = m_node->create_subscription<std_msgs::msg::Float32>("drone_vel", 10, std::bind(&LocalPlannerNode::DroneVelCallBack, this, std::placeholders::_1));

  m_sub_lane = m_node->create_subscription<navigation_msgs::msg::Lane>("lane", 10, std::bind(&LocalPlannerNode::LaneCallBack, this, std::placeholders::_1));

  m_sub_dist_map = m_node->create_subscription<navigation_msgs::msg::DistMapMsg>("dist_map", 10, std::bind(&LocalPlannerNode::DistMapCallBack, this, std::placeholders::_1));

  m_sub_drone_state = m_node->create_subscription<navigation_msgs::msg::DroneState>("drone_state", 10, std::bind(&LocalPlannerNode::DroneStateCallBack, this, std::placeholders::_1));

  m_sub_is_path_blocked = m_node->create_subscription<std_msgs::msg::Bool>(
      "is_path_blocked", 1, std::bind(&LocalPlannerNode::IsPathBlockedCallBack, this, std::placeholders::_1));
}

void LocalPlannerNode::InitParams()
{
  m_node->declare_parameter<std::vector<double>>("COLLISION_PNTS", {0.0, 0.2});
  m_node->declare_parameter<float>("COLLISION_R", 0.25);
  m_node->declare_parameter<float>("WHEEL_BASE", 0.26);
  m_node->declare_parameter<float>("HEURISTIC_GAIN", 1.0);
  m_node->declare_parameter<float>("K_REVERSE_COST", 1.0);
  m_node->declare_parameter<float>("K_GEAR_CHANGE_COST", 0.0);
  m_node->declare_parameter<float>("DOWNSAMPLE_RATIO", 2.0);
  m_node->declare_parameter<float>("DS", 0.4);
  m_node->declare_parameter<float>("GOAL_TOLERANCE", 0.4);
  m_node->declare_parameter<float>("D_COST", 0.3);
  m_node->declare_parameter<bool>("DO_PREEMPT", true);
  m_node->declare_parameter<float>("TIMEOUT", 0.3);
  m_node->declare_parameter<float>("YAWE_COST", 0.3);
  m_node->declare_parameter<float>("YAW_TOLERANCE", 0.3);
  m_node->declare_parameter<float>("MIN_SEGMENT_LEN", 0.5);
  m_node->declare_parameter<int>("MAX_SEGMENTS_NUM", 4);
  m_node->declare_parameter<float>("END_TOLERANCE", 0.2);
  m_node->declare_parameter<float>("MAX_STEER_ANGLE_DEG", 25.0);
  m_node->declare_parameter<float>("REVERSE_MAX_STEER_ANGLE", 25.0);
  m_node->declare_parameter<float>("CLEARANCE_COST", 0.2);
  m_node->declare_parameter<float>("MAX_FORWARD_LEN", 1.0);
}


void LocalPlannerNode::UpdateParams()
{
  m_node->get_parameter("COLLISION_R", m_local_planner.m_hybrid_astar.m_collision_r);
  m_node->get_parameter("COLLISION_PNTS", m_local_planner.m_hybrid_astar.m_collision_pnts);
  m_node->get_parameter("WHEEL_BASE", m_local_planner.m_hybrid_astar.m_wheel_base);
  m_node->get_parameter("HEURISTIC_GAIN", m_local_planner.m_hybrid_astar.m_heuristic_gain);
  m_node->get_parameter("K_REVERSE_COST", m_local_planner.m_hybrid_astar.m_k_reverse_cost);
  m_node->get_parameter("K_GEAR_CHANGE_COST", m_local_planner.m_hybrid_astar.m_k_gear_change_cost);
  m_node->get_parameter("DOWNSAMPLE_RATIO", m_local_planner.m_hybrid_astar.m_downsample_ratio);
  m_node->get_parameter("DS", m_local_planner.m_hybrid_astar.m_ds);
  m_node->get_parameter("GOAL_TOLERANCE", m_local_planner.m_hybrid_astar.m_goal_tolerance);
  m_node->get_parameter("D_COST", m_local_planner.m_hybrid_astar.m_d_cost);
  m_node->get_parameter("DO_PREEMPT", m_local_planner.m_hybrid_astar.m_do_preempt);
  m_node->get_parameter("TIMEOUT", m_local_planner.m_hybrid_astar.m_timeout);
  m_node->get_parameter("YAWE_COST", m_local_planner.m_hybrid_astar.m_yawe_cost);
  m_node->get_parameter("CLEARANCE_COST", m_local_planner.m_hybrid_astar.m_clearance_cost);
  m_node->get_parameter("YAW_TOLERANCE", m_local_planner.m_hybrid_astar.m_yaw_tolerance);
  m_node->get_parameter("MIN_SEGMENT_LEN", m_local_planner.m_hybrid_astar.m_min_segment_len);
  m_node->get_parameter("MAX_SEGMENTS_NUM", m_local_planner.m_hybrid_astar.m_max_segments_num);
  m_node->get_parameter("MAX_STEER_ANGLE_DEG", m_local_planner.m_hybrid_astar.m_max_steer_angle);
  m_node->get_parameter("REVERSE_MAX_STEER_ANGLE", m_local_planner.m_hybrid_astar.m_max_reverse_steer_angle);
  m_node->get_parameter("MAX_FORWARD_LEN", m_local_planner.m_hybrid_astar.m_max_forward_len);
}

void LocalPlannerNode::DroneVelCallBack(const std_msgs::msg::Float32::SharedPtr p_msg)
{
  std_msgs::msg::Float32 vel_msg = *p_msg;

  m_drone_vel = vel_msg.data;
}

void LocalPlannerNode::LaneCallBack(const navigation_msgs::msg::Lane::SharedPtr p_msg)
{
  m_lane = *p_msg;

  if(m_lane.clusters.size() < 2.0f/m_local_planner.m_hybrid_astar.m_ds)
  {
    PubEstop(true);

    m_is_recieved_lane = false;

    return;
  }

  m_is_recieved_lane = true;
}

void LocalPlannerNode::DistMapCallBack(const navigation_msgs::msg::DistMapMsg::SharedPtr p_msg)
{
  m_dist_map = *p_msg;

  m_is_recieved_dist_map = true;
}


void LocalPlannerNode::DroneStateCallBack(const navigation_msgs::msg::DroneState::SharedPtr p_msg)
{
  m_drone_state = *p_msg;

  m_is_recieved_drone_state = true;
}

void LocalPlannerNode::PublishPathMsg()
{
  if(m_local_planner.m_path.path_msg.empty()) return;

  m_pub_local_path->publish(m_local_planner.m_path);
}

void LocalPlannerNode::PubEstop(bool p_estop)
{
  std_msgs::msg::Bool estop_msg;

  estop_msg.data = p_estop;

  m_pub_estop->publish(estop_msg);
}


void LocalPlannerNode::VisualizePath(navigation_msgs::msg::PathMsg p_path)
{
  visualization_msgs::msg::MarkerArray vis_path;

  for(size_t i = 0; i < p_path.path_msg.size(); i++){

    visualization_msgs::msg::Marker WorldPoint = m_control_utils.CreateVisWorldPoint(m_node, p_path.path_msg[i], 0.0, 1.0, 0.0);

    WorldPoint.id = i + 1000;

    vis_path.markers.push_back(WorldPoint);
  }
  m_pub_path_vis->publish(vis_path);
}

void LocalPlannerNode::IsPathBlockedCallBack(const std_msgs::msg::Bool::SharedPtr msg)
{
  bool was_blocked  = m_is_path_blocked;
  m_is_path_blocked = msg->data;

  if (!m_is_path_blocked && was_blocked)
  {
    // Clear stale path — RunLocalPlanner will produce a fresh one before publishing
    m_local_planner.m_path.path_msg.clear();
  }
}

void LocalPlannerNode::RunLocalPlanner()
{
  if (m_is_path_blocked) return;

  if(!m_is_recieved_lane || !m_is_recieved_dist_map || !m_is_recieved_drone_state || (m_lane.clusters.empty())) return;

  m_local_planner.RunLocalPlanner(m_lane, m_dist_map, m_drone_state, m_drone_vel);

  PubEstop(m_local_planner.m_estop);

  PublishPathMsg();

  VisualizePath(m_local_planner.m_path);
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("LocalPlannerNode");

  LocalPlannerNode local_planner_node;
  
  local_planner_node.m_node = node;

  local_planner_node.Init();
  
  while(rclcpp::ok()) {

    local_planner_node.UpdateParams();

    local_planner_node.RunLocalPlanner();

    std::this_thread::sleep_for(20ms);
    
    rclcpp::spin_some(node);

  }

  rclcpp::shutdown();

  return 0;
}