#include <LocalPlanner.hpp>


void LocalPlanner::Init(navigation_msgs::msg::Lane p_lane, navigation_msgs::msg::DistMapMsg p_dist_map,
  navigation_msgs::msg::DroneState p_drone_state, float p_drone_vel)
{
  m_lane = p_lane;

  m_dist_map = p_dist_map;

  m_start_wpoint = InitStartNode(p_drone_state);

  m_drone_vel = p_drone_vel;
}

navigation_msgs::msg::WorldPoint LocalPlanner::InitStartNode(navigation_msgs::msg::DroneState p_drone_state)
{
  navigation_msgs::msg::WorldPoint start_wpoint;

  start_wpoint.x = p_drone_state.x; start_wpoint.y = p_drone_state.y; start_wpoint.yaw = p_drone_state.yaw;

  start_wpoint.s = p_drone_state.s; start_wpoint.d = p_drone_state.d;

  return start_wpoint;
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
  }
}

void LocalPlanner::RunLocalPlanner(navigation_msgs::msg::Lane p_lane, navigation_msgs::msg::DistMapMsg p_dist_map,
  navigation_msgs::msg::DroneState p_drone_state, float p_drone_vel)
{
  Init(p_lane, p_dist_map, p_drone_state, p_drone_vel);

  StructuredPlanning();
}