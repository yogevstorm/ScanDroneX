#include <BehaviorNode.hpp>
using namespace std::chrono_literals;

void BehaviorNode::Init()
{
  InitPublishers();
  InitSubscribers();
}

void BehaviorNode::InitPublishers()
{
  m_pub_lane_vis = m_node->create_publisher<visualization_msgs::msg::MarkerArray>("lane_viz", 1);

  m_pub_lane = m_node->create_publisher<navigation_msgs::msg::Lane>("lane", 1);

  m_pub_drone_state = m_node->create_publisher<navigation_msgs::msg::DroneState>("drone_state", 1);

  m_pub_is_path_blocked  = m_node->create_publisher<std_msgs::msg::Bool>("is_path_blocked", 1);
  m_pub_is_out_of_lane   = m_node->create_publisher<std_msgs::msg::Bool>("is_out_of_lane", 1);
  m_pub_estop            = m_node->create_publisher<std_msgs::msg::Bool>("estop/behavior", 1);
}

void BehaviorNode::InitSubscribers()
{
  m_sub_mission_path = m_node->create_subscription<navigation_msgs::msg::PathMsg>("global_path", 10, std::bind(&BehaviorNode::MissionPathCallBack, this, std::placeholders::_1));

  m_sub_state_pose = m_node->create_subscription<geometry_msgs::msg::Pose>("drone_pose", 1, std::bind(&BehaviorNode::PoseCallBack, this, std::placeholders::_1));

  m_sub_dist_map = m_node->create_subscription<navigation_msgs::msg::DistMapMsg>("dist_map", 10, std::bind(&BehaviorNode::DistMapCallBack, this, std::placeholders::_1));
} 

void BehaviorNode::InitParams()
{
  m_node->declare_parameter<float>("COLLISION_R", 0.2);
  m_node->declare_parameter<float>("WHEEL_BASE", 0.26);
  m_node->declare_parameter<float>("D_MAX", 3.0);
  m_node->declare_parameter<float>("Delta_D", 0.1);
  m_node->declare_parameter<float>("MAX_LANE_LENGTH", 5.0);
  m_node->declare_parameter<float>("DS", 0.1);
  m_node->declare_parameter<float>("K_COST_D", 1.0);
  m_node->declare_parameter<float>("K_COST_WIDTH", 1.0);
  m_node->declare_parameter<float>("MAX_SMOOTH", 1.0);
}

void BehaviorNode::UpdateParams()
{
  m_node->get_parameter("COLLISION_R", m_behavior_planner.m_collision_r_param);
  m_node->get_parameter("WHEEL_BASE", m_behavior_planner.m_wheel_base_param);
  m_node->get_parameter("D_MAX", m_behavior_planner.m_d_max_param);
  m_node->get_parameter("Delta_D", m_behavior_planner.m_dd_param);
  m_node->get_parameter("MAX_LANE_LENGTH", m_behavior_planner.m_max_lane_len_param);
  m_node->get_parameter("DS", m_behavior_planner.m_ds_param);
  m_node->get_parameter("K_COST_D", m_behavior_planner.m_k_cost_d_param);
  m_node->get_parameter("K_COST_WIDTH", m_behavior_planner.m_k_cost_width_param);
  m_node->get_parameter("MAX_SMOOTH", m_behavior_planner.m_max_smooth_param);

}

void BehaviorNode::MissionPathCallBack(const navigation_msgs::msg::PathMsg::SharedPtr msg)
{
  m_mission_path = *msg;
  m_is_recieved_m_path = true;
}

void BehaviorNode::DistMapCallBack(const navigation_msgs::msg::DistMapMsg::SharedPtr msg)
{
  m_dist_map = *msg;
  m_is_recieved_dist_map = true;
}

void BehaviorNode::PoseCallBack(const geometry_msgs::msg::Pose::SharedPtr msg)
{
  m_drone_state_pose_msg = *msg;
  m_is_recieved_drone_pose = true;
} 

void BehaviorNode::RunBehaviorNode()
{
  if(!m_is_recieved_m_path || !m_is_recieved_drone_pose) return;

  m_drone_state.x = m_drone_state_pose_msg.position.x; m_drone_state.y = m_drone_state_pose_msg.position.y;

  m_drone_state.yaw = m_control_utils.angle_wrap(m_drone_state_pose_msg.position.z);

  auto [s, d] = m_control_utils.Cartesian2Curvlinear(m_mission_path, m_drone_state.x, m_drone_state.y);

  m_drone_state.s = s; m_drone_state.d = d;

  m_pub_drone_state->publish(m_drone_state);

  RunBehaviorPlanner();
}

void BehaviorNode::RunBehaviorPlanner()
{
  if(!m_is_recieved_dist_map) return;

  m_behavior_planner.RunBehaviorPlanner(m_mission_path, m_dist_map, m_drone_state);

  m_pub_lane->publish(m_behavior_planner.m_lane);

  std_msgs::msg::Bool blocked_msg;
  blocked_msg.data = m_behavior_planner.m_is_path_blocked;
  m_pub_is_path_blocked->publish(blocked_msg);

  std_msgs::msg::Bool estop_msg;
  estop_msg.data = m_behavior_planner.m_is_path_blocked;
  m_pub_estop->publish(estop_msg);

  bool out_of_lane = false;
  if (!m_behavior_planner.m_lane.clusters.empty())
  {
    const auto& nearest = m_behavior_planner.m_lane.clusters.front();
    out_of_lane = (m_drone_state.d < nearest.dr - 0.5 || m_drone_state.d > nearest.dl + 0.5);
  }
  std_msgs::msg::Bool out_of_lane_msg;
  out_of_lane_msg.data = out_of_lane;
  m_pub_is_out_of_lane->publish(out_of_lane_msg);

  VisLane();
}

void BehaviorNode::VisLane()
{
  if(m_behavior_planner.m_lane.clusters.empty()) return;

  visualization_msgs::msg::Marker right_margin_mkr, left_margin_mkr, end_line_mkr;
  visualization_msgs::msg::MarkerArray lane_mkr;
  right_margin_mkr.header.frame_id="map";
  right_margin_mkr.header.stamp = rclcpp::Clock().now();
  right_margin_mkr.action = visualization_msgs::msg::Marker::ADD;
  right_margin_mkr.type = visualization_msgs::msg::Marker::LINE_STRIP;

  end_line_mkr = right_margin_mkr;
  right_margin_mkr.color.r = 0.4;
  right_margin_mkr.color.g = 1.;
  right_margin_mkr.color.b = 1.;
  right_margin_mkr.color.a = 1.;

  if(m_behavior_planner.m_is_path_blocked)
  {
    end_line_mkr.color.r = 1.;
    end_line_mkr.color.g = 0.1;
    end_line_mkr.color.b = 0.1;
  }
  else
  {
    end_line_mkr.color.r = 0.1;
    end_line_mkr.color.g = 1.;
    end_line_mkr.color.b = 0.1;
  }
  end_line_mkr.color.a = 1.;

  right_margin_mkr.scale.x = 0.05;
  right_margin_mkr.scale.y = 0.05;
  right_margin_mkr.scale.z = 0.05;

  end_line_mkr.scale.x = 0.3;
  end_line_mkr.scale.y = 0.3;
  end_line_mkr.scale.z = 0.05;

  right_margin_mkr.id = 0;
  left_margin_mkr = right_margin_mkr;
  left_margin_mkr.id = 1;
  end_line_mkr.id = 2;

  
  for(auto cluster :m_behavior_planner.m_lane.clusters)
  {
    CreateClusterMarker(cluster, right_margin_mkr, left_margin_mkr);
  }

  end_line_mkr.points.push_back(left_margin_mkr.points.back());
  end_line_mkr.points.push_back(right_margin_mkr.points.back());

  lane_mkr.markers.push_back(right_margin_mkr);
  lane_mkr.markers.push_back(left_margin_mkr);
  lane_mkr.markers.push_back(end_line_mkr);

  m_pub_lane_vis->publish(lane_mkr);
}


void BehaviorNode::CreateClusterMarker(navigation_msgs::msg::LaneCluster p_cluster, visualization_msgs::msg::Marker &p_r_margin_mkr
        , visualization_msgs::msg::Marker &p_l_margin_mkr)
{
  visualization_msgs::msg::Marker cluster_mkr;
  auto [xr, yr] = m_control_utils.CurvilinearToCartesian(p_cluster.s, p_cluster.dr, m_behavior_planner.m_ds_param, m_mission_path);
  auto [xl, yl] = m_control_utils.CurvilinearToCartesian(p_cluster.s, p_cluster.dl, m_behavior_planner.m_ds_param, m_mission_path);
  geometry_msgs::msg::Point pnt_r, pnt_l;
  pnt_r.x = xr; pnt_r.y = yr; pnt_r.z = 0.0;
  pnt_l.x = xl; pnt_l.y = yl; pnt_l.z = 0.0;
  p_r_margin_mkr.points.push_back(pnt_r);
  p_l_margin_mkr.points.push_back(pnt_l);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("BehaviorNode");

  BehaviorNode behavior_node;
  
  behavior_node.m_node = node;

  behavior_node.Init();

  behavior_node.InitParams();
  
  
  while(rclcpp::ok()) {

    behavior_node.RunBehaviorNode();

    behavior_node.UpdateParams();

    std::this_thread::sleep_for(20ms);
    
    rclcpp::spin_some(node);

  }

  rclcpp::shutdown();

  return 0;
}