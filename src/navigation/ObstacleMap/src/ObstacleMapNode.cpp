#include <ObstacleMapNode.hpp>
using namespace std::chrono_literals;

void ObstacleMapNode::Init_Publishers_Subscribers()
{
  m_pub_obs_map = m_node->create_publisher<nav_msgs::msg::OccupancyGrid>("obstacle_map", 1);

  m_sub_map = m_node->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "map", 10, std::bind(&ObstacleMapNode::MapCallBack, this, std::placeholders::_1));

  m_sub_laser = m_node->create_subscription<sensor_msgs::msg::LaserScan>(
      "simple_drone/scan", 10, std::bind(&ObstacleMapNode::LaserScanCallBack, this, std::placeholders::_1));

  m_sub_drone_state = m_node->create_subscription<geometry_msgs::msg::Pose>(
      "drone_pose", 10, std::bind(&ObstacleMapNode::DroneStateCallBack, this, std::placeholders::_1));
}

void ObstacleMapNode::InitParams()
{
  m_node->declare_parameter<float>("MAX_RANGE_LASER_SCAN_FILTER", 4.0);
  m_node->declare_parameter<bool>("SIM_FLAG", true);
}

void ObstacleMapNode::UpdateParams()
{
  m_node->get_parameter("MAX_RANGE_LASER_SCAN_FILTER",
                        m_obstacle_map.m_max_range_laser_scan_filter);

  m_node->get_parameter("SIM_FLAG",
                        m_obstacle_map.m_sim_flag);
}

void ObstacleMapNode::MapCallBack(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  m_occ_map = *msg;
  m_recieved_map_msg = true;
}

void ObstacleMapNode::LaserScanCallBack(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  m_laser_scan = *msg;
  m_recieved_laser_msg = true;
}

void ObstacleMapNode::DroneStateCallBack(const geometry_msgs::msg::Pose::SharedPtr msg)
{
  m_drone_state = *msg;
  RunObstacleMapNode();
}

void ObstacleMapNode::RunObstacleMapNode()
{
  if(!m_recieved_map_msg || !m_recieved_laser_msg)
  {
    return;
  }

  m_obstacle_map.RunObstacleMap(m_occ_map, m_laser_scan, m_drone_state);

  m_pub_obs_map->publish(m_obstacle_map.m_obs_map);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node =
      std::make_shared<rclcpp::Node>("ObstacleMapNode");

  ObstacleMapNode obstacle_map_node;
  obstacle_map_node.m_node = node;

  obstacle_map_node.Init_Publishers_Subscribers();
  obstacle_map_node.InitParams();

  while(rclcpp::ok())
  {
    obstacle_map_node.UpdateParams();
    std::this_thread::sleep_for(20ms);
    rclcpp::spin_some(node);
  }

  rclcpp::shutdown();
  return 0;
}
