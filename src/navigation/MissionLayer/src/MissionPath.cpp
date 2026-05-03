#include <MissionPath.hpp>

void MissionPath::Init_Publishers_Subscribers()
{
  m_pub_path = m_node->create_publisher<visualization_msgs::msg::MarkerArray>("path_viz", 1);
  m_sub_map = m_node->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "obstacle_map", 10, std::bind(&MissionPath::MapCallBack, this, _1));
}

void MissionPath::VisualizePath(navigation_msgs::msg::PathMsg path)
{
  visualization_msgs::msg::MarkerArray vis_path;

  for(int i = 0; i < path.path_msg.size(); i++)
  {
    visualization_msgs::msg::Marker WorldPoint =
        m_control_utils.CreateVisWorldPoint(m_node, path.path_msg[i],
                                            0.0, 0.0, 1.0, 1.0, 1.5);
    WorldPoint.id = i;
    vis_path.markers.push_back(WorldPoint);
  }

  m_pub_path->publish(vis_path);
}

void MissionPath::MapCallBack(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  m_map = *msg;
  m_recieved_map_msg = true;
}

nav_msgs::msg::OccupancyGrid MissionPath::LoadMap()
{
  return m_map;
}

navigation_msgs::msg::PathMsg MissionPath::Smooth(navigation_msgs::msg::PathMsg path,
                                                  float weight_data,
                                                  float weight_smooth,
                                                  double tolerance,
                                                  int max_iterations)
{
  navigation_msgs::msg::PathMsg new_path = path;
  float change = tolerance + 1.0f;
  int iteration = 0;
  float grad_x, grad_y, grad_yaw, dx, dy, dsi;

  while(change > tolerance && iteration < max_iterations)
  {
    change = 0;

    for(int i = 1; i < (int)path.path_msg.size() - 1; i++)
    {
      grad_x = weight_data * (path.path_msg[i].x - new_path.path_msg[i].x)
             + weight_smooth * (new_path.path_msg[i+1].x + new_path.path_msg[i-1].x - 2*new_path.path_msg[i].x);

      grad_y = weight_data * (path.path_msg[i].y - new_path.path_msg[i].y)
             + weight_smooth * (new_path.path_msg[i+1].y + new_path.path_msg[i-1].y - 2*new_path.path_msg[i].y);

      grad_yaw = weight_data * (m_control_utils.angle_wrap(path.path_msg[i].yaw - new_path.path_msg[i].yaw))
               + weight_smooth * m_control_utils.angle_wrap(
                    (m_control_utils.angle_wrap(new_path.path_msg[i+1].yaw + new_path.path_msg[i-1].yaw))
                    - 2*new_path.path_msg[i].yaw);

      change += abs(grad_yaw);

      new_path.path_msg[i].x += grad_x;
      new_path.path_msg[i].y += grad_y;
      new_path.path_msg[i].yaw += grad_yaw;
    }

    iteration++;
  }

  path.path_msg.clear();
  if (new_path.path_msg.empty())
  {
    return path;
  }
  navigation_msgs::msg::WorldPoint orig = new_path.path_msg[0];
  path.path_msg.push_back(orig);

  for(size_t i = 1; i < new_path.path_msg.size(); i++)
  {
    dx  = new_path.path_msg[i].x - new_path.path_msg[i - 1].x;
    dy  = new_path.path_msg[i].y - new_path.path_msg[i - 1].y;
    dsi = sqrt(dx*dx + dy*dy);

    if(dsi >= 0.01)
    {
      navigation_msgs::msg::WorldPoint wpoint;
      wpoint.x   = new_path.path_msg[i].x;
      wpoint.y   = new_path.path_msg[i].y;
      wpoint.yaw = new_path.path_msg[i].yaw;
      path.path_msg.push_back(wpoint);
    }
  }

  path.path_msg.erase(path.path_msg.begin());
  return path;
}

navigation_msgs::msg::PathMsg MissionPath::Interpolation(navigation_msgs::msg::PathMsg path)
{
  navigation_msgs::msg::PathMsg interpolated_path;

  std::vector<float> x_vec;
  std::vector<float> y_vec;
  std::vector<float> s_vec;

  int j = 0;

  for(size_t i = 0; i < path.path_msg.size(); i++)
  {
    x_vec.push_back(path.path_msg[i].x);
    y_vec.push_back(path.path_msg[i].y);
    s_vec.push_back(path.path_msg[i].s);
  }

  boost::math::interpolators::barycentric_rational<float> spl_x(s_vec.data(), x_vec.data(), s_vec.size());
  boost::math::interpolators::barycentric_rational<float> spl_y(s_vec.data(), y_vec.data(), s_vec.size());

  while(j * m_ds <= s_vec.back())
  {
    navigation_msgs::msg::WorldPoint point;

    point.s = float(j) * m_ds;
    point.x = spl_x(j * m_ds);
    point.y = spl_y(j * m_ds);

    if(j < 1)
    {
      point.yaw = path.path_msg[0].yaw;
    }
    else
    {
      float dx = point.x - interpolated_path.path_msg.back().x;
      float dy = point.y - interpolated_path.path_msg.back().y;
      point.yaw = atan2(dy, dx);
    }

    point.ind = j;

    interpolated_path.path_msg.push_back(point);
    j++;
  }

  return interpolated_path;
}

navigation_msgs::msg::PathMsg MissionPath::FindPath(geometry_msgs::msg::Pose start,
                                                    geometry_msgs::msg::Pose end,
                                                    navigation_msgs::msg::DistMapMsg dist_map)
{
  navigation_msgs::msg::PathMsg path = m_astar.Search(dist_map, start, end);

  path = Smooth(path, 0.1, 0.5, 0.001, 100000);

  if (path.path_msg.empty())
  {
    return path;
  }

  path.path_msg[0].s = 0.0;
  path.path_msg[0].ind = 0;

  for(size_t i = 1; i < path.path_msg.size(); i++)
  {
    path.path_msg[i].s = path.path_msg[i-1].s
      + m_control_utils.DisBetweenWPoints(path.path_msg[i], path.path_msg[i-1]);

    path.path_msg[i].ind = i;
  }

  path = Interpolation(path);
  return path;
}
