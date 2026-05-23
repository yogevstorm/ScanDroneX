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

  m_sub_drone_state = m_node->create_subscription<navigation_msgs::msg::DroneState>(
      "drone_state", 1, std::bind(&ScanNode::DroneStateCallBack, this, std::placeholders::_1));

  m_sub_openings = m_node->create_subscription<geometry_msgs::msg::PoseArray>(
      "/openings_detector/openings", 1,
      std::bind(&ScanNode::OpeningsCallBack, this, std::placeholders::_1));
}

void ScanNode::Run()
{
  if (!m_is_map_received) return;
  if (m_returning_home)   return;

  if (!m_has_goal)
  {
    if (m_openings.empty()) {
      if (m_no_openings_timer_set) {
        auto elapsed = (m_node->get_clock()->now() - m_no_openings_since).seconds();
        if (elapsed >= 5.0) {
          ReturnHome();
        }
      }
      return;
    }
    PublishNewGoal();
  }
}

void ScanNode::MapCallBack(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  m_map = *msg;
  m_is_map_received = true;
}

void ScanNode::OpeningsCallBack(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
  m_openings      = msg->poses;
  m_opening_index = 0;

  if (m_openings.empty()) {
    if (!m_no_openings_timer_set) {
      m_no_openings_since     = m_node->get_clock()->now();
      m_no_openings_timer_set = true;
    }
  } else {
    m_no_openings_timer_set = false;
  }
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
    PublishEstop(true);
    PublishNewGoal();
  }
  else if (!m_is_path_blocked && was_blocked)
  {
    if (!m_is_goal_unreachable)
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
  m_is_goal_unreachable = msg->data;

  if (m_returning_home) return;
  if (!msg->data)
  {
    if (!m_is_path_blocked)
      PublishEstop(false);
    return;
  }
  if (!m_has_goal) return;

  PublishNewGoal();
}

void ScanNode::PublishEstop(bool estop)
{
  std_msgs::msg::Bool msg;
  msg.data = estop;
  m_pub_estop->publish(msg);
}

void ScanNode::InitParams() {}

void ScanNode::UpdateParams() {}

void ScanNode::DroneStateCallBack(const navigation_msgs::msg::DroneState::SharedPtr /*msg*/) {}

void ScanNode::ReturnHome()
{
  RCLCPP_WARN(m_node->get_logger(), "[ScanNode] Returning home (0, 0).");
  geometry_msgs::msg::PoseStamped home;
  home.header.stamp    = m_node->get_clock()->now();
  home.header.frame_id = "map";
  home.pose.position.x = 0.0;
  home.pose.position.y = 0.0;
  home.pose.orientation.w = 1.0;
  m_pub_goal_pose->publish(home);
  m_returning_home = true;
  m_has_goal       = false;
}

void ScanNode::PublishNewGoal()
{
  if (m_returning_home) return;

  if (m_opening_index < m_openings.size())
  {
    geometry_msgs::msg::PoseStamped goal;
    goal.header.stamp    = m_node->get_clock()->now();
    goal.header.frame_id = "map";
    goal.pose            = m_openings[m_opening_index++];
    m_current_goal = goal;
    m_has_goal     = true;
    m_pub_goal_pose->publish(m_current_goal);
    RCLCPP_INFO(m_node->get_logger(), "[ScanNode] New goal → opening[%zu] at (%.2f, %.2f)",
      m_opening_index - 1,
      m_current_goal.pose.position.x, m_current_goal.pose.position.y);
    return;
  }

  RCLCPP_WARN(m_node->get_logger(), "[ScanNode] All openings exhausted.");
  ReturnHome();
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
