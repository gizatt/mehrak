#pragma once

#include <BasicLinearAlgebra.h>

template <int N>
class PIDMotionSmoother
{
  using State = BLA ::Matrix<N, 1>;

public:
  PIDMotionSmoother(
      const State &x_0, const State &x_des, float freq, float damping_ratio) : m_x(x_0), m_xd(BLA::Zeros<N, 1>()), m_x_des(x_des), m_last_update_t(0.), m_kp(freq * freq), m_kd(-2. * freq * damping_ratio)
  {
  }

  const State &get_state() const
  {
    return m_x;
  }

  void set_target(const State &x_des)
  {
    m_x_des = x_des;
  }

  void update(double t)
  {
    float dt = t - m_last_update_t;
    m_last_update_t = t;

    State xdd = (m_x_des - m_x) * m_kp + m_xd * m_kd;
    m_xd += xdd * dt;
    m_x += m_xd * dt;
  }

private:
  State m_x;
  State m_xd;
  State m_x_des;
  double m_last_update_t;
  float m_kp;
  float m_kd;
};