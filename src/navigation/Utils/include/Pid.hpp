#pragma once
#include <algorithm>

class Pid
{
public:
  void SetGains(float kp, float ki, float kd);
  void SetOutputLimits(float min_out, float max_out);

  // error: setpoint - measurement
  // measurement: raw process variable used for derivative-on-measurement (-d(y)/dt)
  // dt: time delta in seconds
  float Compute(float error, float measurement, float dt);

  void Reset();

private:
  float m_kp = 0.0f;
  float m_ki = 0.0f;
  float m_kd = 0.0f;

  float m_integral = 0.0f;
  float m_prev_measurement = 0.0f;

  float m_out_min = -1.0f;
  float m_out_max =  1.0f;

  bool m_first_run = true;
};
