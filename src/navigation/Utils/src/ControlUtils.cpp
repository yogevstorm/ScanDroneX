#include "ControlUtils.hpp"


geometry_msgs::msg::Quaternion ControlUtils::Euler2Quat(double yaw, double pitch, double roll) 
{
  geometry_msgs::msg::Quaternion q;
  // Abbreviations for the various angular functions
  double cy = cos(yaw * 0.5);
  double sy = sin(yaw * 0.5);
  double cp = cos(pitch * 0.5);
  double sp = sin(pitch * 0.5);
  double cr = cos(roll * 0.5);
  double sr = sin(roll * 0.5);
  q.w = cr * cp * cy + sr * sp * sy;
  q.x = sr * cp * cy - cr * sp * sy;
  q.y = cr * sp * cy + sr * cp * sy;
  q.z = cr * cp * sy - sr * sp * cy;
  return q;
}

float ControlUtils::Quat2Yaw(geometry_msgs::msg::Quaternion q) {

  double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
  float yaw = std::atan2(siny_cosp, cosy_cosp);
  return yaw;
}


visualization_msgs::msg::Marker ControlUtils::CreateVisCollisionR(std::shared_ptr<rclcpp::Node> node, geometry_msgs::msg::Pose pose, float collision_r, float r,
float g, float b, float a)
{
  visualization_msgs::msg::Marker collision_r_marker;
  collision_r_marker.header.frame_id = "map";
  rclcpp::Time now = node->get_clock()->now();
  collision_r_marker.header.stamp = now;
  collision_r_marker.type = 3;
  collision_r_marker.pose = pose;
  collision_r_marker.color.r = r;
  collision_r_marker.color.g = g;
  collision_r_marker.color.b = b;
  collision_r_marker.color.a = a;
  collision_r_marker.scale.x = 2 * collision_r; collision_r_marker.scale.y = 2 * collision_r; collision_r_marker.scale.z = 0.02;
  collision_r_marker.lifetime.sec = 1.0;
  return collision_r_marker;
}


visualization_msgs::msg::Marker ControlUtils::CreateVisWorldPoint(std::shared_ptr<rclcpp::Node> node, navigation_msgs::msg::WorldPoint WorldPoint,
float color_r, float color_g, float color_b, float color_a, float scale, float color_rev_r, float color_rev_g, float color_rev_b, float color_rev_a)
{
  visualization_msgs::msg::Marker WorldPoint_marker;
  geometry_msgs::msg::Pose WorldPoint_pose;
  WorldPoint_marker.header.frame_id = "map";
  rclcpp::Time now = node->get_clock()->now();
  WorldPoint_marker.header.stamp = now;
  WorldPoint_marker.type = 0;
  WorldPoint_pose.position.x = WorldPoint.x; WorldPoint_pose.position.y = WorldPoint.y;
  WorldPoint_pose.orientation = Euler2Quat(WorldPoint.yaw, 0, 0);
  WorldPoint_marker.pose = WorldPoint_pose;
  WorldPoint_marker.color.r = color_r;
  WorldPoint_marker.color.g = color_g;
  WorldPoint_marker.color.b = color_b;
  WorldPoint_marker.color.a = color_a;

  if(WorldPoint.gear == -1)
  {
    WorldPoint_marker.color.r = color_rev_r;
    WorldPoint_marker.color.g = color_rev_g;
    WorldPoint_marker.color.b = color_rev_b;
    WorldPoint_marker.color.a = color_rev_a;
  }

  WorldPoint_marker.scale.x = 0.1 * scale; WorldPoint_marker.scale.y = 0.02 * scale; WorldPoint_marker.scale.z = 0.01 * scale;
  WorldPoint_marker.lifetime.sec = 1;
  return WorldPoint_marker;
}

float ControlUtils::DisBetweenWPoints(navigation_msgs::msg::WorldPoint WorldPoint1, navigation_msgs::msg::WorldPoint WorldPoint2){
  return sqrt(pow((WorldPoint1.x - WorldPoint2.x),2) + pow((WorldPoint1.y - WorldPoint2.y),2));
}

float ControlUtils::DisBetweenMPoints(std::vector<int> MapPoint1, std::vector<int> MapPoint2){
  return sqrt(pow((MapPoint1[0] - MapPoint2[0]),2) + pow((MapPoint1[1] - MapPoint2[1]),2));
}

std::tuple<float, float> ControlUtils::Cartesian2Curvlinear(navigation_msgs::msg::PathMsg mission_path, float x, float y)
{
  int idx = ClosestIdx(mission_path, x, y);
  float mp_x = mission_path.path_msg[idx].x; float mp_y = mission_path.path_msg[idx].y; float mp_yaw = mission_path.path_msg[idx].yaw;
  float s = std::sin(mp_yaw)*(y - mp_y) + std::cos(mp_yaw) * (x - mp_x) + mission_path.path_msg[idx].s;
  float d = std::cos(mp_yaw)*(y - mp_y) - std::sin(mp_yaw) * (x - mp_x);
  return std::make_tuple(s, d);
}

std::tuple<float, float> ControlUtils::Cartesian2Curvlinear(navigation_msgs::msg::Lane lane, float x, float y)
{
  int cluster_idx = ClosestIdx(lane, x, y);
  float mp_x = lane.clusters[cluster_idx].x; float mp_y = lane.clusters[cluster_idx].y; float mp_yaw = lane.clusters[cluster_idx].yaw;
  float s = std::sin(mp_yaw)*(y - mp_y) + std::cos(mp_yaw) * (x - mp_x) + lane.clusters[cluster_idx].s;
  float d = std::cos(mp_yaw)*(y - mp_y) - std::sin(mp_yaw) * (x - mp_x);
  return std::make_tuple(s, d);
}

int ControlUtils::ClosestIdx(navigation_msgs::msg::Lane lane, float x, float y)
{
  float min_dist = 999.9;
  int closest_idx = 0;

  for(size_t i = 0; i < lane.clusters.size(); i++)
  {
    float dist = sqrt((pow((lane.clusters[i].x - x), 2) + pow((lane.clusters[i].y - y), 2)));
    if(dist < min_dist)
    {
      min_dist = dist;
      closest_idx = i;
    }
  }
  return closest_idx;
}

int ControlUtils::ClosestIdx(navigation_msgs::msg::PathMsg mission_path, float x, float y)
{
  float min_dist = 999.9;
  int closest_idx = 0;

  for(int i = 0; i < mission_path.path_msg.size(); i++)
  {
    float dist = sqrt((pow((mission_path.path_msg[i].x - x), 2) + pow((mission_path.path_msg[i].y - y), 2)));
    if(dist < min_dist)
    {
      min_dist = dist;
      closest_idx = i;
    }
  }
  return closest_idx;
}

int ControlUtils::SIdxMPath(float s, float ds, navigation_msgs::msg::PathMsg mission_path)
{
  if(s < 0)
  {
    return 0;
  }

  return std::min(int(s/ds), int(mission_path.path_msg.size() - 1));
}


int ControlUtils::SIdxLane(float s, float ds, navigation_msgs::msg::Lane lane)
{
  if(s < 0)
  {
    return 0;
  }

  s -= lane.clusters.front().s;

  return std::min(int(s/ds), int(lane.clusters.size() - 1));
}

std::tuple<float, float> ControlUtils::CurvilinearToCartesian(float s, float d, float ds, navigation_msgs::msg::PathMsg mission_path)
{
  int idx = SIdxMPath(s, ds, mission_path);
  idx = std::min(idx, int(mission_path.path_msg.size() - 1));
  float x = -std::sin(mission_path.path_msg[idx].yaw) * d + mission_path.path_msg[idx].x;
  float y = std::cos(mission_path.path_msg[idx].yaw) * d + mission_path.path_msg[idx].y;
  return std::make_tuple(x, y);
}


std::vector<float> ControlUtils::LinSpace(float start, float end, int num) {
  std::vector<float> linspaced;

  if (num == 0) { 
    return linspaced;
  }
  if (num == 1) {
    linspaced.push_back(start);
    return linspaced;
  }

  float delta = (end - start) / (num - 1);

  for (int i = 0; i < num; ++i) {
    linspaced.push_back(start + delta * i);
  }

  return linspaced;
}


navigation_msgs::msg::WorldPoint ControlUtils::FindMPathWPointByS(float s, navigation_msgs::msg::PathMsg mission_path)
{

  for(size_t i = 0; i < mission_path.path_msg.size(); i++)
  {
    float s_diff = s - mission_path.path_msg[i].s;
    if(s_diff <= 0.0)
    {
      return mission_path.path_msg[i];
    }
  }
  return mission_path.path_msg.back();
}

float ControlUtils::angle_wrap(float angle)
{
  return mod_2_pi(angle + (float)M_PI) - M_PI;
}

float ControlUtils::mod_2_pi(float angle)
{
  if (angle < 0)
    angle += 2 * M_PI;
  return std::fmod(angle, 2 * M_PI);
}

float ControlUtils::Saturate(float val, float min_val,float max_val)
{
  return std::clamp(val, min_val, max_val);
}

std::vector<float> ControlUtils::diff(const std::vector<float>& v)
{
  std::vector<float> result(v.size() - 1);

  for (std::size_t i = 0; i < result.size(); i++) {
    result[i] = angle_wrap(v[i + 1] - v[i]);
  }

  result.push_back(angle_wrap(v.back() - v[v.size() - 2]));
  return result;
}
