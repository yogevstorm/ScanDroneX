#include <Pid.hpp>

void Pid::SetGains(float kp, float ki, float kd)
{
  m_kp = kp;
  m_ki = ki;
  m_kd = kd;
}

void Pid::SetOutputLimits(float min_out, float max_out)
{
  m_out_min = min_out;
  m_out_max = max_out;
}

float Pid::Compute(float error, float measurement, float dt)
{
  if (m_first_run)
  {
    m_prev_measurement = measurement;
    m_first_run = false;
  }

  // Derivative on measurement: avoids derivative kick on setpoint changes
  float derivative = (dt > 1e-6f) ? -(measurement - m_prev_measurement) / dt : 0.0f;
  m_prev_measurement = measurement;

  // Anti-windup: clamp integral to output limits
  m_integral = std::clamp(m_integral + m_ki * error * dt, m_out_min, m_out_max);

  return std::clamp(m_kp * error + m_integral + m_kd * derivative, m_out_min, m_out_max);
}

void Pid::Reset()
{
  m_integral = 0.0f;
  m_first_run = true;
}
