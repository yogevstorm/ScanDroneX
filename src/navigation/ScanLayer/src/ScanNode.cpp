#include <ScanNode.hpp>
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

  if (!msg->data)
  {
    m_unreachable_count = 0;
    return;
  }

  if (!m_has_goal) return;

  m_is_path_blocked = false;
  PublishEstop(false);
  m_unreachable_count++;

  if (m_unreachable_count >= 4)
  {
    RCLCPP_WARN(m_node->get_logger(), "All 4 corners unreachable — returning home.");
    geometry_msgs::msg::PoseStamped home;
    home.header.stamp    = m_node->get_clock()->now();
    home.header.frame_id = "map";
    home.pose.position.x = 0.0;
    home.pose.position.y = 0.0;
    home.pose.orientation.w = 1.0;  // yaw = 0
    m_pub_goal_pose->publish(home);
    m_returning_home = true;
    m_has_goal       = false;
    return;
  }

  PublishNewGoal();
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

void ScanNode::PublishNewGoal()
{
  geometry_msgs::msg::PoseStamped goal;

  // Try each of the 4 corners in order starting from m_corner_index.
  // Skip corners that have no unknown cells (fully explored quadrant).
  for (int attempt = 0; attempt < 4; attempt++)
  {
    int corner = (m_corner_index + attempt) % 4;
    if (m_scan_layer.FindCornerGoal(m_map, goal, corner))
    {
      m_corner_index = (corner + 1) % 4;
      goal.header.stamp = m_node->get_clock()->now();
      m_current_goal    = goal;
      m_has_goal        = true;
      m_pub_goal_pose->publish(m_current_goal);
      RCLCPP_INFO(m_node->get_logger(),
        "[ScanNode] New goal → corner %d  (%.2f, %.2f)",
        corner, goal.pose.position.x, goal.pose.position.y);
      return;
    }
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
