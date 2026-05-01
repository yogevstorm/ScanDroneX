#include <Control.hpp>
using namespace std::chrono_literals;

std::vector<float> Control::GetCmd(navigation_msgs::msg::DroneState drone_state, navigation_msgs::msg::PathMsg trajectory)
{
  m_drone_state = drone_state;
  m_trajectory = trajectory;

  if(trajectory.path_msg.empty())
  {
    return {0.0f, 0.0f, 0.0f};
  }

  bool is_reverse = (m_trajectory.path_msg[0].gear == -1);

  m_closest_idx = FindClosestIdx();
  m_lookahead_idx = FindLookaheadIdx(m_closest_idx);

  float target_yaw = m_trajectory.path_msg[m_lookahead_idx].yaw;
  float yaw_error  = m_control_utils.angle_wrap(target_yaw - m_drone_state.yaw);
  auto rot_cmd = YawAlignmentCmd(yaw_error);
  if (!rot_cmd.empty()) return rot_cmd;

  float steer = PurePursuit(is_reverse);
  float v = AdaptiveSpeed();
  float lateral = CrossTrackCmd();
  if(is_reverse) v = -v;

  return {v, steer, lateral};
}

// Rotates the drone in place when heading error is large (hysteresis 60 -> 10 deg).
// Returns {0, omega} while rotating, empty vector when not rotating.
std::vector<float> Control::YawAlignmentCmd(float yaw_error)
{
  static constexpr float START_ROT_RAD = 60.0f * M_PI / 180.0f;
  static constexpr float STOP_ROT_RAD  = 10.0f * M_PI / 180.0f;

  if (!m_is_rotating && std::abs(yaw_error) > START_ROT_RAD)
    m_is_rotating = true;
  if (m_is_rotating && std::abs(yaw_error) < STOP_ROT_RAD)
    m_is_rotating = false;

  if (!m_is_rotating) return {};

  float omega = m_control_utils.Saturate(m_yaw_k_gain * yaw_error, -m_max_angular, m_max_angular);
  return {0.0f, omega, 0.0f};
}

// Finds the trajectory point closest to the drone's current position.
int Control::FindClosestIdx()
{
  float min_dist = 999.9f;
  int closest = 0;

  for(size_t i = 0; i < m_trajectory.path_msg.size(); i++)
  {
    float dist = sqrt(pow(m_drone_state.x - m_trajectory.path_msg[i].x, 2)
                    + pow(m_drone_state.y - m_trajectory.path_msg[i].y, 2));
    if(dist < min_dist)
    {
      min_dist = dist;
      closest = i;
    }
  }
  return closest;
}

// Walks forward from closest_idx accumulating arc length until m_lookahead_dis is reached.
int Control::FindLookaheadIdx(int closest_idx)
{
  float arc_len = 0.0f;

  for(size_t i = closest_idx; i < m_trajectory.path_msg.size() - 1; i++)
  {
    float dx = m_trajectory.path_msg[i + 1].x - m_trajectory.path_msg[i].x;
    float dy = m_trajectory.path_msg[i + 1].y - m_trajectory.path_msg[i].y;
    arc_len += sqrt(dx * dx + dy * dy);

    if(arc_len >= m_lookahead_dis) return i + 1;
  }
  return m_trajectory.path_msg.size() - 1;
}

// Unified Pure Pursuit for both forward and reverse.
// For reverse, the lateral offset sign is flipped so the drone steers correctly
// when backing toward the lookahead point.
float Control::PurePursuit(bool is_reverse)
{
  auto [x, y] = RelPointToDrone(m_trajectory.path_msg[m_lookahead_idx].x,
                                 m_trajectory.path_msg[m_lookahead_idx].y);
  if(is_reverse) y = -y;

  float ld2 = x * x + y * y;
  if(ld2 < 1e-6f) return 0.0f;

  float steer = atan(2.0f * m_wheel_base * y / ld2);
  return m_control_utils.Saturate(steer, -m_max_steer_angle, m_max_steer_angle);
}

// Scales speed down with path curvature at the lookahead point:
//   v = v_max / (1 + k_gain * |k|), clamped to [v_min, v_max]
float Control::AdaptiveSpeed()
{
  float k = std::abs(m_trajectory.path_msg[m_lookahead_idx].k);
  float v = m_v_max / (1.0f + m_k_gain * k);
  return std::max(m_v_min, std::min(m_v_max, v));
}

// Computes lateral (strafe) velocity to correct cross-track error.
// Uses the closest trajectory point in the drone's local frame; y-component
// is the signed lateral offset (positive = trajectory is to the left).
float Control::CrossTrackCmd()
{
  auto [x, y] = RelPointToDrone(m_trajectory.path_msg[m_closest_idx].x,
                                 m_trajectory.path_msg[m_closest_idx].y);

  auto now = std::chrono::steady_clock::now();
  float dt = 0.0f;
  if (m_crosstrack_initialized)
    dt = std::chrono::duration<float>(now - m_last_crosstrack_time).count();
  m_last_crosstrack_time = now;
  m_crosstrack_initialized = true;

  m_cross_track_pid.SetGains(m_cross_track_kp, m_cross_track_ki, m_cross_track_kd);
  m_cross_track_pid.SetOutputLimits(-m_max_lateral, m_max_lateral);
  return m_cross_track_pid.Compute(y, y, dt);
}

// Transforms world-frame point (x, y) into the drone's local frame.
std::tuple<float, float> Control::RelPointToDrone(float x, float y)
{
  float drone_yaw = m_control_utils.angle_wrap(m_drone_state.yaw);
  float x1 = cos(drone_yaw) * (x - m_drone_state.x) + sin(drone_yaw) * (y - m_drone_state.y);
  float y1 = -sin(drone_yaw) * (x - m_drone_state.x) + cos(drone_yaw) * (y - m_drone_state.y);
  return std::make_tuple(x1, y1);
}
