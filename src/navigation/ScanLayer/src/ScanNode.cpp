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

  // Re-publish the same goal every 10 s while the path is blocked so
  // MissionPathNode keeps retrying the replan.
  if (m_is_path_blocked)
  {
    rclcpp::Time now = m_node->get_clock()->now();
    if ((now - m_last_blocked_pub).seconds() >= 10.0)
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

  if (m_is_path_blocked)
  {
    PublishEstop(true);
  }
  else if (was_blocked)
  {
    // Transition: blocked → clear. Resume drone and re-publish goal once so
    // MissionPathNode triggers a fresh replan on the now-open path.
    PublishEstop(false);
    if (m_has_goal)
    {
      m_current_goal.header.stamp = m_node->get_clock()->now();
      m_pub_goal_pose->publish(m_current_goal);
    }
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

  if (m_unreachable_count >= 10)
  {
    RCLCPP_WARN(m_node->get_logger(), "10 consecutive unreachable goals — returning home.");
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
  if (!m_scan_layer.FindRandomGoal(m_map, goal, m_has_goal, m_current_goal, m_num_candidates))
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
