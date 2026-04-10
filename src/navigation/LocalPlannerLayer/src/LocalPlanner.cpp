#include <LocalPlanner.hpp>


void LocalPlanner::Init(navigation_msgs::msg::Lane p_lane, navigation_msgs::msg::DistMapMsg p_dist_map,
  navigation_msgs::msg::DroneState p_drone_state, float p_drone_vel)
{
  m_lane = p_lane;

  m_dist_map = p_dist_map;

  m_start_wpoint = InitStartNode(p_drone_state);

  m_drone_vel = p_drone_vel;

  m_is_unstructured = false;
}

navigation_msgs::msg::WorldPoint LocalPlanner::InitStartNode(navigation_msgs::msg::DroneState p_drone_state)
{
  navigation_msgs::msg::WorldPoint start_wpoint;

  start_wpoint.x = p_drone_state.x; start_wpoint.y = p_drone_state.y; start_wpoint.yaw = p_drone_state.yaw;

  start_wpoint.s = p_drone_state.s; start_wpoint.d = p_drone_state.d;

  return start_wpoint;
}

void LocalPlanner::PauseBetweenSegments()
{
  if((rclcpp::Clock().now() - m_start_time).seconds() < 1.0)
  {
    m_estop = true;
  }

  else
  {
    m_estop = false;
  }
}

void LocalPlanner::StuckInUnstructuredRelease()
{
  if(std::abs(m_drone_vel) > 0.1)
  {
    m_stuck_in_unstructured_start_time = rclcpp::Clock().now();
  }

  if(std::abs(m_drone_vel) < 0.05 && !m_path_segments.empty())
  {
    auto seconds_passed = (rclcpp::Clock().now() - m_stuck_in_unstructured_start_time).seconds();

    if(seconds_passed > m_stuck_in_unstructured_relase_timer)
    {
      m_path_segments.clear();
    }
  }
}

void LocalPlanner::IsUnStructuredPlanning()
{
  float yaw_e = m_control_utils.angle_wrap(m_lane.clusters.front().yaw - m_start_wpoint.yaw);

  bool is_change_direction = (abs(yaw_e) > 45 * (M_PI/180.0));

  if(is_change_direction) m_is_unstructured = true;
}

void LocalPlanner::StructuredPlanning()
{
  m_path = m_hybrid_astar.Search(m_dist_map, m_start_wpoint, m_lane);

  if(m_path.path_msg.empty())
  {
    m_planning_failers_counter += 1;
  }

  else
  {
    m_planning_failers_counter = 0;

    m_path_segments.clear();
  }
}

void LocalPlanner::UnStructuredPlanning()
{
  StuckInUnstructuredRelease();

  if((std::abs(m_drone_vel) < 0.05 && m_path_segments.empty()))
  {
    navigation_msgs::msg::WorldPoint goal_wpoint;

    goal_wpoint.x = m_lane.clusters.back().x; goal_wpoint.y = m_lane.clusters.back().y; goal_wpoint.yaw = m_lane.clusters.back().yaw;

    goal_wpoint.s = m_lane.clusters.back().s;

    m_costmap = m_dist_map_obj.GenCostMap(m_dist_map, goal_wpoint, m_hybrid_astar.m_collision_r);

    m_path_unstructured = m_hybrid_astar.Search(m_dist_map, m_start_wpoint, m_lane, m_is_unstructured, m_costmap);

    m_stuck_in_unstructured_start_time = rclcpp::Clock().now();

    CreatePathSegments();
  }
}

void LocalPlanner::CreatePathSegments()
{
  m_path_segments.clear();

  navigation_msgs::msg::PathMsg segment;

  if(m_path_unstructured.path_msg.empty()) return;

  for(size_t p = 0; p < m_path_unstructured.path_msg.size(); p++)
  {
    segment.path_msg.push_back(m_path_unstructured.path_msg[p]);

    if(m_path_unstructured.path_msg[p].gear != m_path_unstructured.path_msg[p + 1].gear)
    {
      m_path_segments.push_back(segment);

      segment.path_msg.clear();

      navigation_msgs::msg::WorldPoint change_gear_node = m_path_unstructured.path_msg[p];

      change_gear_node.gear = m_path_unstructured.path_msg[p + 1].gear;

      segment.path_msg.push_back(change_gear_node);
    }
  }
  m_path_segments.push_back(segment);
}

void LocalPlanner::SetPathSegment()
{
  float min_dist = 999.9;

  if(m_path_segments.size() < 1)
  {
    return;
  }

  for(size_t p = 0; p < m_path_segments[0].path_msg.size(); p++)
  {
    float dist = sqrt(pow((m_start_wpoint.x - m_path_segments[0].path_msg[p].x), 2)
    + (pow((m_start_wpoint.y - m_path_segments[0].path_msg[p].y), 2)));

    if(dist < min_dist)
    {
      min_dist = dist;

      if(p == m_path_segments[0].path_msg.size() - 1)
      {
        m_path_segments.erase(m_path_segments.begin());

        m_start_time = rclcpp::Clock().now();

        break;
      }
    }
  }
}

void LocalPlanner::RunLocalPlanner(navigation_msgs::msg::Lane p_lane, navigation_msgs::msg::DistMapMsg p_dist_map,
  navigation_msgs::msg::DroneState p_drone_state, float p_drone_vel)
{
  Init(p_lane, p_dist_map, p_drone_state, p_drone_vel);

  PauseBetweenSegments();

  if(m_estop)
  {        
    return;
  }

  IsUnStructuredPlanning();

  if(m_is_unstructured)
  {
    UnStructuredPlanning();
    if(m_path_segments.empty())
    {
      m_path.path_msg.clear();

      return;
    }

    SetPathSegment();

    m_path = m_path_segments[0];
  }

  else
  {
    StructuredPlanning();
  }
}