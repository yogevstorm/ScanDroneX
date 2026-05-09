#include <ScanNode.hpp>
#include <algorithm>
using namespace std::chrono_literals;

void ScanNode::Init()
{
  m_pub_goal_pose = m_node->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 1);
  m_pub_estop     = m_node->create_publisher<std_msgs::msg::Bool>("estop/scan", 1);

  m_sub_map = m_node->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", 1, std::bind(&ScanNode::MapCallBack, this, std::placeholders::_1));

  m_sub_is_destination = m_node->create_subscription<std_msgs::msg::Bool>(
      "is_destination", 1, std::bind(&ScanNode::IsDestinationCallBack, this, std::placeholders::_1));

  m_sub_lane_end = m_node->create_subscription<std_msgs::msg::Bool>(
      "lane_end", 1, std::bind(&ScanNode::IsDestinationCallBack, this, std::placeholders::_1));

  m_sub_is_path_blocked = m_node->create_subscription<std_msgs::msg::Bool>(
      "is_path_blocked", 1, std::bind(&ScanNode::IsPathBlockedCallBack, this, std::placeholders::_1));

  m_sub_goal_unreachable = m_node->create_subscription<std_msgs::msg::Bool>(
      "is_goal_unreachable", 1, std::bind(&ScanNode::GoalUnreachableCallBack, this, std::placeholders::_1));

  m_sub_is_out_of_lane = m_node->create_subscription<std_msgs::msg::Bool>(
      "is_out_of_lane", 1, std::bind(&ScanNode::IsOutOfLaneCallBack, this, std::placeholders::_1));

  m_sub_drone_state = m_node->create_subscription<navigation_msgs::msg::DroneState>(
      "drone_state", 1, std::bind(&ScanNode::DroneStateCallBack, this, std::placeholders::_1));
}

void ScanNode::Run()
{
  if (!m_is_map_received) return;
  if (m_returning_home)   return;

  // No goal yet — pick one as soon as the map arrives
  if (!m_has_goal)
  {
    PublishNewGoal();
    return;
  }

}

void ScanNode::MapCallBack(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  m_map = *msg;
  m_is_map_received = true;
}

void ScanNode::IsDestinationCallBack(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (m_returning_home) return;

  if (msg->data && m_has_goal)
  {
    PublishNewGoal();
  }
}

void ScanNode::IsPathBlockedCallBack(const std_msgs::msg::Bool::SharedPtr msg)
{
  bool was_blocked  = m_is_path_blocked;
  m_is_path_blocked = msg->data;

  if (m_is_path_blocked && !was_blocked)
  {
    // Newly blocked — re-publish the same goal once so the planner retries.
    PublishEstop(true);
    if (m_has_goal)
    {
      m_current_goal.header.stamp = m_node->get_clock()->now();
      m_pub_goal_pose->publish(m_current_goal);
    }
  }
  else if (!m_is_path_blocked && was_blocked)
  {
    PublishEstop(false);
  }
}

void ScanNode::IsOutOfLaneCallBack(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (m_returning_home) return;

  if (msg->data)
  {
    PublishEstop(true);
    if (m_has_goal)
    {
      m_current_goal.header.stamp = m_node->get_clock()->now();
      m_pub_goal_pose->publish(m_current_goal);
    }
  }
  else
  {
    PublishEstop(false);
  }
}

void ScanNode::GoalUnreachableCallBack(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (m_returning_home) return;
  if (!msg->data) { m_corner_attempt = 0; return; }
  if (!m_has_goal) return;

  m_is_path_blocked = false;
  PublishEstop(false);

  m_corner_attempt++;

  if (m_corner_attempt < 4)
  {
    // Same corner, next spiral point outward.
    PublishNewGoal(false);
    return;
  }

  // Current corner exhausted — move to the next one.
  m_corner_attempt = 0;
  m_corner_index++;

  if (m_corner_index >= 4)
  {
    RCLCPP_WARN(m_node->get_logger(), "All corners exhausted — returning home.");
    geometry_msgs::msg::PoseStamped home;
    home.header.stamp    = m_node->get_clock()->now();
    home.header.frame_id = "map";
    home.pose.position.x = 0.0;
    home.pose.position.y = 0.0;
    home.pose.orientation.w = 1.0;
    m_pub_goal_pose->publish(home);
    m_returning_home = true;
    m_has_goal       = false;
    return;
  }

  PublishNewGoal(false);
}

void ScanNode::PublishEstop(bool estop)
{
  std_msgs::msg::Bool msg;
  msg.data = estop;
  m_pub_estop->publish(msg);
}

void ScanNode::InitParams()
{
  m_node->declare_parameter<int>("SCAN_NUM_CANDIDATES", 10);
}

void ScanNode::UpdateParams()
{
  m_node->get_parameter("SCAN_NUM_CANDIDATES", m_num_candidates);
}

void ScanNode::DroneStateCallBack(const navigation_msgs::msg::DroneState::SharedPtr msg)
{
  m_drone_x         = msg->x;
  m_drone_y         = msg->y;
  m_has_drone_state = true;
}

void ScanNode::PublishNewGoal(bool new_session)
{
  if (new_session)
  {
    m_corner_index   = 0;
    m_corner_attempt = 0;

    for (int i = 0; i < 4; i++) m_sorted_corners[i] = i;

    if (m_has_drone_state)
    {
      const float ox    = m_map.info.origin.position.x;
      const float oy    = m_map.info.origin.position.y;
      const float map_w = m_map.info.width  * m_map.info.resolution;
      const float map_h = m_map.info.height * m_map.info.resolution;

      const float cx[4] = { ox,          ox + map_w,  ox + map_w,  ox         };
      const float cy[4] = { oy,          oy,           oy + map_h,  oy + map_h };

      std::sort(m_sorted_corners, m_sorted_corners + 4, [&](int a, int b) {
        float dxa = cx[a] - m_drone_x, dya = cy[a] - m_drone_y;
        float dxb = cx[b] - m_drone_x, dyb = cy[b] - m_drone_y;
        return (dxa*dxa + dya*dya) < (dxb*dxb + dyb*dyb);
      });
    }
  }

  geometry_msgs::msg::PoseStamped goal;
  int corner = m_sorted_corners[m_corner_index];

  if (m_scan_layer.FindCornerGoal(m_map, goal, corner, m_corner_attempt))
  {
    goal.header.stamp = m_node->get_clock()->now();
    m_current_goal    = goal;
    m_has_goal        = true;
    m_pub_goal_pose->publish(m_current_goal);
    RCLCPP_INFO(m_node->get_logger(),
      "[ScanNode] New goal → corner %d attempt %d  (%.2f, %.2f)",
      corner, m_corner_attempt, goal.pose.position.x, goal.pose.position.y);
    return;
  }

  RCLCPP_WARN(m_node->get_logger(), "No unknown cells remaining in map — scan complete.");
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("ScanNode");

  ScanNode scan_node;
  scan_node.m_node = node;
  scan_node.InitParams();
  scan_node.Init();

  while (rclcpp::ok())
  {
    scan_node.UpdateParams();
    scan_node.Run();
    std::this_thread::sleep_for(20ms);
    rclcpp::spin_some(node);
  }

  rclcpp::shutdown();
  return 0;
}
