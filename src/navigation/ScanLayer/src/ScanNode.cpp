#include <ScanNode.hpp>
using namespace std::chrono_literals;

void ScanNode::Init()
{
  m_pub_goal_pose = m_node->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 1);

  m_sub_map = m_node->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", 1, std::bind(&ScanNode::MapCallBack, this, std::placeholders::_1));

  m_sub_is_destination = m_node->create_subscription<std_msgs::msg::Bool>(
      "is_destination", 1, std::bind(&ScanNode::IsDestinationCallBack, this, std::placeholders::_1));

  m_sub_is_path_blocked = m_node->create_subscription<std_msgs::msg::Bool>(
      "is_path_blocked", 1, std::bind(&ScanNode::IsPathBlockedCallBack, this, std::placeholders::_1));
}

void ScanNode::Run()
{
  if (!m_is_map_received) return;

  // No goal yet — pick one as soon as the map arrives
  if (!m_has_goal)
  {
    PublishNewGoal();
    return;
  }

  // Path is blocked — re-publish the same goal at ~1 Hz until it clears
  if (m_is_path_blocked)
  {
    rclcpp::Time now = m_node->get_clock()->now();
    if ((now - m_last_blocked_pub).seconds() >= 1.0)
    {
      m_current_goal.header.stamp = now;
      m_pub_goal_pose->publish(m_current_goal);
      m_last_blocked_pub = now;
    }
  }
}

void ScanNode::MapCallBack(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  m_map = *msg;
  m_is_map_received = true;
}

void ScanNode::IsDestinationCallBack(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data && m_has_goal)
  {
    PublishNewGoal();
  }
}

void ScanNode::IsPathBlockedCallBack(const std_msgs::msg::Bool::SharedPtr msg)
{
  m_is_path_blocked = msg->data;
}

void ScanNode::PublishNewGoal()
{
  geometry_msgs::msg::PoseStamped goal;
  if (!m_scan_layer.FindRandomGoal(m_map, goal))
  {
    RCLCPP_WARN(m_node->get_logger(), "No unknown cells remaining in map — scan complete.");
    return;
  }

  goal.header.stamp = m_node->get_clock()->now();
  m_current_goal = goal;
  m_has_goal = true;
  m_pub_goal_pose->publish(m_current_goal);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("ScanNode");

  ScanNode scan_node;
  scan_node.m_node = node;
  scan_node.Init();

  while (rclcpp::ok())
  {
    scan_node.Run();
    std::this_thread::sleep_for(20ms);
    rclcpp::spin_some(node);
  }

  rclcpp::shutdown();
  return 0;
}
