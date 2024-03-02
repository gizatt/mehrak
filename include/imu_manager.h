#pragma once

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "low_pass_filter.h"

const double ZERO_BIAS_RC = 20.0;
const double DENOISE_BIAS_RC = 0.1;

class IMUManager {
  public:
    IMUManager() : m_imu_valid(false), m_gyro_lp_slow(ZERO_BIAS_RC), m_gyro_lp_fast(DENOISE_BIAS_RC) {
      try_to_start_imu();
    }

    void update(double t) {
      if (!m_imu_valid){
        try_to_start_imu();
      }
      if (!m_imu_valid){
        return;
      }

      // Run two different low-pass filters on the gyro terms:
      // one at 10s scale to zero out bias, at one at a much faster
      // scale to smooth noise.
      sensors_event_t a, g, temp;
      if (!m_mpu.getEvent(&a, &g, &temp)){
        m_imu_valid = false;
        return;
      }
      m_raw_gyro_measurement(0) = g.gyro.x;
      m_raw_gyro_measurement(1) = g.gyro.y;
      m_raw_gyro_measurement(2) = g.gyro.z;

      m_gyro_lp_fast.add_measurement(t, m_raw_gyro_measurement);
      m_gyro_lp_slow.add_measurement(t, m_raw_gyro_measurement);
      m_smoothed_gyro_measurement = m_gyro_lp_fast.get_state() - m_gyro_lp_slow.get_state();
    }

    bool is_valid() const {
      return m_imu_valid;
    }

    const BLA::Matrix<3, 1> & get_raw_gyro_measurement() const{
      return m_raw_gyro_measurement;
    }

    const BLA::Matrix<3, 1>& get_smoothed_gyro_measurement() const{
      return m_smoothed_gyro_measurement; 
    }
    

  private:
    void try_to_start_imu(){
      if (m_mpu.begin(0x68)){
        m_imu_valid = true;
        m_mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
        m_mpu.setGyroRange(MPU6050_RANGE_250_DEG);
        m_mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
      } else {
        m_imu_valid = false;
      }
    }

    Adafruit_MPU6050 m_mpu;
    bool m_imu_valid;

    LowPassFilter<3> m_gyro_lp_slow;
    LowPassFilter<3> m_gyro_lp_fast;
    BLA ::Matrix<3, 1> m_raw_gyro_measurement;
    BLA ::Matrix<3, 1> m_smoothed_gyro_measurement;

};

