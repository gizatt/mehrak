#pragma once

#include <BasicLinearAlgebra.h>

template <int N>
class LowPassFilter
{
  using State = BLA ::Matrix<N, 1>;

public:
  LowPassFilter(double rc_constant) : m_last_update_t(-1.), m_rc_constant(rc_constant)
  { 
    m_x.Fill(0);
  }

  void add_measurement(double t, const State& x_new)
  {
    // Special handling of first update.
    if (m_last_update_t < 0){
        m_last_update_t = t;
        m_x = x_new;
        return;
    }

    double dt = t - m_last_update_t;
    m_last_update_t = t;

    float alpha = dt / (dt + m_rc_constant);
    for (int i = 0; i < N; i++){
        m_x(i) = m_x(i) * (1. - alpha) + x_new(i) * alpha;
    }
  }

  const State& get_state() const {
    return m_x;
  }

private:
  State m_x;
  double m_last_update_t;
  double m_rc_constant;
};