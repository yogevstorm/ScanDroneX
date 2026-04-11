#include <MotionPlanner.hpp>
using namespace std::chrono_literals;


navigation_msgs::msg::PathMsg MotionPlanner::ProcessPath(navigation_msgs::msg::PathMsg p_lpath, navigation_msgs::msg::DroneState drone_state)
{
  if(p_lpath.path_msg.size() < 2)
  {
    return p_lpath;
  }

  Init(drone_state);
  FindClosestIdxToDrone(p_lpath);
  ComputeArcLengthSamples(p_lpath);
  SetXY(p_lpath);
  SetGear(p_lpath);
  SetYaw();
  SetK();
  SetS();
  SetTrajMsg();
  return m_trajectory;
}

void MotionPlanner::Init(navigation_msgs::msg::DroneState p_drone_state)
{
  m_x.clear();
  m_y.clear();
  m_yaw.clear();
  m_k.clear();
  m_s.clear();
  m_gear.clear();
  m_dist_along.clear();
  m_dist_along_inter.clear();
  m_trajectory.path_msg.clear();
  m_drone_state = p_drone_state;
}

void MotionPlanner::FindClosestIdxToDrone(navigation_msgs::msg::PathMsg p_lpath)
{
  float min_dis = 999.9;

  for(size_t i = 0; i < p_lpath.path_msg.size(); i++)
  {
    float error_yaw = abs(m_control_utils.angle_wrap(p_lpath.path_msg[i].yaw - m_drone_state.yaw));
    float dist = sqrt(pow((m_drone_state.x - p_lpath.path_msg[i].x), 2)
    + pow((m_drone_state.y - p_lpath.path_msg[i].y), 2)) + m_yaw_closest_idx_to_drone_weight * error_yaw;

    if(dist < min_dis)
    {
      min_dis = dist;
      m_closest_idx = i;
    }
  }
}

void MotionPlanner::ComputeArcLengthSamples(navigation_msgs::msg::PathMsg p_lpath)
{
  m_dist_along.push_back(0.0);
  m_dist_along_inter.push_back(0.0);
  // int start_idx = std::max(m_closest_idx, 1);
  int start_idx = m_closest_idx;

  for(size_t p = start_idx; p < p_lpath.path_msg.size() - 1; p++)
  {
    float dist = sqrt(pow((p_lpath.path_msg[p + 1].x - p_lpath.path_msg[p].x), 2)
    + pow((p_lpath.path_msg[p + 1].y - p_lpath.path_msg[p].y), 2));

    if(m_dist_along.back() + dist > m_dis_ahead)
    {
      break;
    }

    m_dist_along.push_back(m_dist_along.back() + dist);
  }

  float end_interp_dist = std::max(m_dist_along.back(), 2*m_ds);
  int num_interp_dist = int(end_interp_dist / m_ds);

  for (int i = 0; i < num_interp_dist; i++) {
    float iterpulated_dist = i * end_interp_dist / num_interp_dist;
    m_dist_along_inter.push_back(iterpulated_dist);
  }
}

void MotionPlanner::SetGear(navigation_msgs::msg::PathMsg p_lpath_msg)
{
  if(p_lpath_msg.path_msg.empty())
  {
    return;
  }
  
  for(size_t i = 0; i < m_dist_along_inter.size(); i++)
  {
    m_gear.push_back(p_lpath_msg.path_msg[0].gear);
  }
}

void MotionPlanner::SetXY(navigation_msgs::msg::PathMsg p_lpath_msg)
{
  if(p_lpath_msg.path_msg.empty())
  {
    return;
  }
  
  Eigen::VectorXd xVec(m_dist_along.size());
  Eigen::VectorXd yVec(m_dist_along.size());
  Eigen::VectorXd distVec(m_dist_along.size());
  int spline_degree = std::min(m_spline_degree, int(m_dist_along.size()));
  // Convert the data points to Eigen vectors

  for (size_t i = 0; i < m_dist_along.size(); i++) {
    xVec(i) = p_lpath_msg.path_msg[m_closest_idx + i].x;
    yVec(i) = p_lpath_msg.path_msg[m_closest_idx + i].y;
    distVec(i) = m_dist_along[i];
  }

  // Compute the least squares polynomial coefficients
  Eigen::VectorXd coefficients_x = leastSquaresPolynomial(distVec, xVec, spline_degree);
  Eigen::VectorXd coefficients_y = leastSquaresPolynomial(distVec, yVec, spline_degree);

  for(size_t i = 0; i < m_dist_along_inter.size(); i++)
  {
    navigation_msgs::msg::WorldPoint traj_point;
    m_x.push_back(evaluateSpline(coefficients_x, m_dist_along_inter[i]));
    m_y.push_back(evaluateSpline(coefficients_y, m_dist_along_inter[i]));
  }
}

void MotionPlanner::SetYaw()
{
  std::vector<float> dx = diff(m_x);
  std::vector<float> dy = diff(m_y);

  for(size_t i = 1; i < dx.size(); i++)
  {
    m_yaw.push_back(m_control_utils.angle_wrap(std::atan2(dy[i], dx[i])));
  }  
}

void MotionPlanner::SetK()
{
  for(size_t i = 0; i < m_yaw.size() - 1; i++)
  {
    float k = (m_yaw[i + 1] - m_yaw[i]) / m_ds;

    m_k.push_back(k);
  }
}

void MotionPlanner::SetS()
{
  m_s.push_back(0.0);

  for(size_t p = 1; p < m_x.size(); p++)
  {
    float dist = sqrt(pow((m_x[p] - m_x[p - 1]), 2)
    + pow((m_y[p] - m_y[p - 1]), 2));

    m_s.push_back(dist);
  }
}

void MotionPlanner::SetTrajMsg()
{
  for(size_t p = 0; p < m_k.size(); p++)
  {
    navigation_msgs::msg::WorldPoint traj_point;
    traj_point.x = m_x[p];
    traj_point.y = m_y[p];
    traj_point.yaw = m_yaw[p];
    traj_point.k = m_k[p];
    traj_point.s = m_s[p];
    traj_point.gear = m_gear[p];
    m_trajectory.path_msg.push_back(traj_point);
  }
}

std::vector<float> MotionPlanner::diff(const std::vector<float>& v)
{
  std::vector<float> result(v.size() - 1);

  for (std::size_t i = 0; i < result.size(); i++) {
    result[i] = m_control_utils.angle_wrap(v[i + 1] - v[i]);
  }

  result.push_back(m_control_utils.angle_wrap(v.back() - v[v.size() - 2]));
  return result;
}

Eigen::VectorXd MotionPlanner::leastSquaresPolynomial(const Eigen::VectorXd& x, const Eigen::VectorXd& y, int degree)
{
  int n = x.size();
  int m = degree + 1;

  Eigen::MatrixXd A(n, m);
  Eigen::VectorXd b(n);

  // Construct the Vandermonde matrix A and the right-hand side vector b
  for (int i = 0; i < n; i++) {
      for (int j = 0; j < m; j++) {
          A(i, j) = std::pow(x(i), j);
      }
      b(i) = y(i);
  }

  // Solve the least squares problem
  Eigen::VectorXd coefficients = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);

  return coefficients;
}

// Function to evaluate the spline at a specific parameter value
float MotionPlanner::evaluateSpline(const Eigen::VectorXd& coefficients, float x)
{
  float result = 0.0;
  int degree = coefficients.size() - 1;

  for (int i = 0; i <= degree; i++) {
      result += coefficients(i) * std::pow(x, i);
  }

  return result;
}
