#include <MissionPathNode.hpp>
using namespace std::chrono_literals;

void MissionPathNode::Init_Publishers_Subscribers()
{
  m_sub_goal_pose = m_node->create_subscription<geometry_msgs::msg::PoseStamped>(
      "goal_pose", 10, std::bind(&MissionPathNode::GoalPoseCallBack, this, _1));

  m_sub_state_pose = m_node->create_subscription<geometry_msgs::msg::Pose>(
      "drone_pose", 10, std::bind(&MissionPathNode::StateCallBack, this, _1));

  m_sub_obstacle_map = m_node->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "obstacle_map", 10, std::bind(&MissionPathNode::ObstacleMapCallBack, this, _1));

  m_pub_collision_r_vis =
      m_node->create_publisher<visualization_msgs::msg::MarkerArray>("collision_r", 1);

  m_pub_esp_collision_r_vis =
      m_node->create_publisher<visualization_msgs::msg::MarkerArray>("esp_collision_r", 1);

  m_pub_mission_path =
      m_node->create_publisher<navigation_msgs::msg::PathMsg>("global_path", 1);

  m_pub_dist_map =
      m_node->create_publisher<navigation_msgs::msg::DistMapMsg>("dist_map", 1);

  m_pub_estop = m_node->create_publisher<std_msgs::msg::Bool>("estop/mission", 1);

  m_pub_is_destination = m_node->create_publisher<std_msgs::msg::Bool>("is_destination", 1);

  m_pub_goal_unreachable = m_node->create_publisher<std_msgs::msg::Bool>("is_goal_unreachable", 1);

  m_pub_drone_vel = m_node->create_publisher<std_msgs::msg::Float32>("drone_vel", 1);
}

void MissionPathNode::Init()
{
  m_mission_path.m_node = m_node;
  InitParams();
  Init_Publishers_Subscribers();
  m_mission_path.Init_Publishers_Subscribers();
}

void MissionPathNode::InitParams()
{
  m_node->declare_parameter<float>("COLLISION_R", 0.2);
  m_node->declare_parameter<float>("ESP_COLLISION_R", 0.15);
  m_node->declare_parameter<std::vector<double>>("COLLISION_PNTS", {0.0, 0.2});
  m_node->declare_parameter<float>("END_TOLERANCE", 0.2);
  m_node->declare_parameter<double>("ASTAR_TIMEOUT_SEC", 10.0);
}

void MissionPathNode::UpdateParams()
{
  m_node->get_parameter("COLLISION_R", m_collision_r);
  m_node->get_parameter("ESP_COLLISION_R", m_esp_collision_r);
  m_node->get_parameter("COLLISION_PNTS", m_collision_pnts);
  m_node->get_parameter("END_TOLERANCE", m_end_tolerance);
  m_mission_path.m_astar.m_collision_r = m_collision_r;
  m_node->get_parameter("ASTAR_TIMEOUT_SEC", m_mission_path.m_astar.m_search_timeout_sec);
}

void MissionPathNode::InitMission()
{
  if(!m_is_recieved_state_msg || !m_mission_path.m_recieved_map_msg)
  {
    return;
  }

  m_map = m_mission_path.LoadMap();

  m_dist_map = m_distmap.CreateDistMap(m_map, {0, -1});

  m_mpath = m_mission_path.FindPath(m_drone_state, m_goal_pose.pose, m_dist_map);

  std_msgs::msg::Bool unreachable_msg;
  unreachable_msg.data = m_mpath.path_msg.empty();
  m_pub_goal_unreachable->publish(unreachable_msg);

  if (m_mpath.path_msg.empty())
  {
    return;
  }

  m_pub_mission_path->publish(m_mpath);

  m_is_recieved_mpath = true;
}

void MissionPathNode::RunMission()
{
  if(!m_is_recieved_mpath)
  {
    return;
  }

  float seconds_passed_from_latest_state_msg =
      (cv::getTickCount() - m_start_time) / cv::getTickFrequency();

  if(seconds_passed_from_latest_state_msg > 3.0)
  {
    m_drone_vel = 0.0;
  }

  std_msgs::msg::Float32 vel_msg;
  vel_msg.data = m_drone_vel;

  m_pub_drone_vel->publish(vel_msg);

  m_mission_path.VisualizePath(m_mpath);
}

void MissionPathNode::StateCallBack(const geometry_msgs::msg::Pose::SharedPtr msg)
{
  geometry_msgs::msg::Pose current_drone_state = *msg;
  CalcDroneVel(current_drone_state);
  m_drone_state = current_drone_state;
  m_is_recieved_state_msg = true;
  m_start_time = cv::getTickCount();
}

void MissionPathNode::CalcDroneVel(const geometry_msgs::msg::Pose current_drone_state)
{
  m_localization_dt = (cv::getTickCount() - m_start_time) / cv::getTickFrequency();

  float ds = sqrt((pow((current_drone_state.position.x - m_drone_state.position.x), 2)
                 + pow((current_drone_state.position.y - m_drone_state.position.y), 2)));

  m_drone_vel = ds / m_localization_dt;
}

void MissionPathNode::GoalPoseCallBack(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  m_goal_pose = *msg;

  m_recieved_goal_pose_msg = true;

  InitMission();
}

void MissionPathNode::ObstacleMapCallBack(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  m_obs_map = *msg;

  navigation_msgs::msg::DistMapMsg dist_map_msg = m_distmap.CreateDistMap(m_obs_map, {0, -1});

  m_pub_dist_map->publish(dist_map_msg);
}

void MissionPathNode::ESTOPCollisonR()
{
  visualization_msgs::msg::MarkerArray esp_collision_r_pnts_viz;

  for(size_t i = 0; i < m_collision_pnts.size(); i++)
  {
    float yaw = m_drone_state.position.z;
    geometry_msgs::msg::Pose collision_pnt_pose;
    navigation_msgs::msg::WorldPoint collision_pnt;

    collision_pnt_pose.position.x = m_drone_state.position.x + m_collision_pnts[i] * cos(yaw);
    collision_pnt_pose.position.y = m_drone_state.position.y + m_collision_pnts[i] * sin(yaw);

    collision_pnt.x = collision_pnt_pose.position.x;
    collision_pnt.y = collision_pnt_pose.position.y;

    visualization_msgs::msg::Marker collision_r_pnt_viz =
        m_control_utils.CreateVisCollisionR(m_node, collision_pnt_pose, m_esp_collision_r,
                                            1.0, 0.0, 0.0, 0.5);

    collision_r_pnt_viz.id = i;
    esp_collision_r_pnts_viz.markers.push_back(collision_r_pnt_viz);

    int map_cell_id = m_distmap.World2Map1D(m_dist_map.info, collision_pnt);
    // if(m_dist_map.data[map_cell_id] < m_esp_collision_r)
    // {
    //   PubEstop(true);
    // }
  }

  m_pub_esp_collision_r_vis->publish(esp_collision_r_pnts_viz);
}

void MissionPathNode::CollisionRViz()
{
  visualization_msgs::msg::MarkerArray collision_r_pnts_viz;

  for(size_t i = 0; i < m_collision_pnts.size(); i++)
  {
    float yaw = m_drone_state.position.z;
    geometry_msgs::msg::Pose collision_pnt_pose;

    collision_pnt_pose.position.x = m_drone_state.position.x + m_collision_pnts[i] * cos(yaw);
    collision_pnt_pose.position.y = m_drone_state.position.y + m_collision_pnts[i] * sin(yaw);

    visualization_msgs::msg::Marker collision_r_pnt_viz =
        m_control_utils.CreateVisCollisionR(m_node, collision_pnt_pose, m_collision_r,
                                            0.0, 1.0, 0.0, 0.5);

    collision_r_pnt_viz.id = i;
    collision_r_pnts_viz.markers.push_back(collision_r_pnt_viz);
  }

  m_pub_collision_r_vis->publish(collision_r_pnts_viz);
}

void MissionPathNode::PubEstop(bool estop)
{
  std_msgs::msg::Bool estop_msg;
  estop_msg.data = estop;
  m_pub_estop->publish(estop_msg);
}

bool MissionPathNode::IsDestination()
{
  if(m_mpath.path_msg.empty())
  {
    return false;
  }

  float dist = sqrt(pow((m_drone_state.position.x - m_mpath.path_msg.back().x), 2)
                  + pow((m_drone_state.position.y - m_mpath.path_msg.back().y), 2));

  if(dist < m_end_tolerance)
  {
    return true;
  }
  return false;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("MissionPathNode");

  MissionPathNode MissionPathNode;          // (local variable, no underscore, no need to rename)
  MissionPathNode.m_node = node;    // MissionPathNode.node_ -> MissionPathNode.m_node

  MissionPathNode.Init();

  while(rclcpp::ok())
  {
    MissionPathNode.UpdateParams();
    MissionPathNode.CollisionRViz();

    std_msgs::msg::Bool dest_msg;
    dest_msg.data = MissionPathNode.IsDestination();
    MissionPathNode.m_pub_is_destination->publish(dest_msg);

    if(MissionPathNode.IsDestination())
    {
      MissionPathNode.PubEstop(true);
    }
    else
    {
      if(MissionPathNode.m_recieved_goal_pose_msg && MissionPathNode.m_is_recieved_mpath)
      {
        MissionPathNode.RunMission();
        MissionPathNode.ESTOPCollisonR();
      }
    }

    std::this_thread::sleep_for(20ms);
    rclcpp::spin_some(node);
  }

  rclcpp::shutdown();
  return 0;
}
