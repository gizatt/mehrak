#pragma once

#include <Adafruit_Protomatter.h>
#include "BasicLinearAlgebra.h"
#include "screen.h"
#include "imu_manager.h"

uint8_t rgbPins[] = {6, A5, A1, A0, A4, 11};
uint8_t addrPins[] = {10, 5, 13, 9};
uint8_t clockPin = 12;
uint8_t latchPin = PIN_SERIAL1_RX;
uint8_t oePin = PIN_SERIAL1_TX;
const int DISPLAY_WIDTH = 64;
const int DISPLAY_HEIGHT = 32;
const float MAX_GYRO_EYE_OFFSET_X = 0.05;
const float MAX_GYRO_EYE_OFFSET_Y = 0.2;
const float HEAD_MOVING_ANG_VEL_THRESHOLD = 0.5;

const float BLINK_DURATION = 0.1;
const float NOMINAL_EYE_HEIGHT = 0.3;
const float NOMINAL_EYE_WIDTH = 0.1;
const float NOMINAL_EYE_SPACING = 0.15;
const float NOMINAL_EYE_FREQ = 10.0;
const float NOMINAL_EYE_DAMPING_RATIO = 0.9;
const float FAST_EYE_FREQ = 30.0;
const float FAST_EYE_DAMPING_RATIO = 0.9;

enum EyeMode
{
  NORMAL = 0,
  HAPPY = 1
};

Adafruit_Protomatter matrix = Adafruit_Protomatter(
    DISPLAY_WIDTH * 2,         // Width of matrix (or matrices, if tiled horizontally)
    4,                         // Bit depth, 1-6
    1, rgbPins,                // # of matrix chains, array of 6 RGB pins for each
    4, addrPins,               // # of address pins (height is inferred), array of pins
    clockPin, latchPin, oePin, // Other matrix control pins
    true                       // No double-buffering here (see "doublebuffer" example)
);                             // Row tiling: two rows in "serpentine" path)

// Dims the supplied color by the given fraction
// ref some stuff from https://forums.adafruit.com/viewtopic.php?t=21536
uint16_t dim565(uint16_t color, float fraction)
{
  uint32_t bits = (uint32_t)color;
  uint32_t blue = bits & 0x001F;  // 5 bits blue
  uint32_t green = bits & 0x07E0; // 6 bits green
  uint32_t red = bits & 0xF800;   // 5 bits red
  blue = fraction * (float)blue;
  green = fraction * (float)green;
  red = fraction * (float)red;
  return (red / 8 << 11) | (green / 4 << 5) | (blue / 8);
}

class DisplayManager
{
public:
  DisplayManager()
  {
    nominal_eye_target = eye_motion_smoother.get_target();

    // Initialize matrix...
    ProtomatterStatus status = matrix.begin();
    if (status != PROTOMATTER_OK)
    {
      // DO NOT CONTINUE if matrix setup encountered an error.
      for (;;)
        ;
    }

    gyro_eye_offset.Fill(0.);
  }

  void update(double t)
  {
    float dt = t - last_update_t;
    last_update_t = t;

    m_imu_manager.update(t);

    // Blink when it's time
    float t_blink = t - time_of_next_blink;
    float eye_height = NOMINAL_EYE_HEIGHT;
    if (t_blink <= 0.0)
    {
    }
    else if (t_blink <= BLINK_DURATION)
    {
      eye_height *= (BLINK_DURATION - t_blink) / BLINK_DURATION;
    }
    else if (t_blink <= 2. * BLINK_DURATION)
    {
      eye_height *= (t_blink - BLINK_DURATION) / BLINK_DURATION;
    }
    else
    {
      // Choose when to next blink
      time_of_next_blink = t + rand_range(2.0, 6.0);
    }

    // // Move target occasionally.
    // if (t >= time_of_next_target_change)
    // {
    //   // Choose new target
    //   time_of_next_target_change = t + rand_range(1.0, 5.0);
    //   eye_motion_smoother.set_target(
    //       {rand_range(0.42, 0.58), rand_range(0.1, 0.5)});
    //   // Force a blink during the move
    //   time_of_next_blink = t + rand_range(0, 0.2);
    // }

    BLA::Matrix<2, 1> eye_offset;
    eye_offset.Fill(0.);

    if (m_imu_manager.is_valid())
    {
      const auto gyro_measurement = m_imu_manager.get_smoothed_gyro_measurement();

      // ang velocity in rad/sec
      float angular_velocity = sqrt(gyro_measurement(0) * gyro_measurement(0) + gyro_measurement(1) * gyro_measurement(1));
      bool head_currently_moving = angular_velocity >= HEAD_MOVING_ANG_VEL_THRESHOLD;
      if (head_currently_moving){
        head_motion_last_moving_t = t;
      } else {
        head_motion_last_stationary_t = t;
      }
      if ((t - head_motion_last_stationary_t) > 0.1 && !head_motion_event_active)
      {
        head_motion_event_active = true;
        head_motion_start_t = t;
        head_motion_second_phase_start_t = t + 100.;
        gyro_eye_offset.Fill(0.);
      } else if ((t - head_motion_last_moving_t) > 1.0 && head_motion_event_active){
        head_motion_event_active = false;
        gyro_eye_offset.Fill(0.);
      }

      if (head_motion_event_active)
      {
        eye_motion_smoother.update_damping(FAST_EYE_FREQ, FAST_EYE_DAMPING_RATIO);
        if (t < head_motion_second_phase_start_t)
        {
          // Compensate for current motion.
          gyro_eye_offset(0) += -gyro_measurement(1) * dt / 10.;
          gyro_eye_offset(1) += gyro_measurement(0) * dt / 5.;

          if (abs(gyro_eye_offset(0)) > 2. * MAX_GYRO_EYE_OFFSET_X || abs(gyro_eye_offset(1) > MAX_GYRO_EYE_OFFSET_Y)){
            head_motion_second_phase_start_t = min(head_motion_second_phase_start_t, t + BLINK_DURATION/2.);
            time_of_next_blink = t;
          } else {
            head_motion_second_phase_start_t = t + 100.;
          }
          // for (int i = 0; i < 2; i++) 
          // {
          //   gyro_eye_offset(i) *= 0.9;
          // }
        }
        else
        {
          // We've reached the edge, so just try to look in the direction of current motion instead.
          gyro_eye_offset(0) = gyro_measurement(1) * dt;
          gyro_eye_offset(1) = -gyro_measurement(0) * dt * 2.;
        }
      }
      else
      {
        eye_motion_smoother.update_damping(NOMINAL_EYE_FREQ, NOMINAL_EYE_DAMPING_RATIO);
        gyro_eye_offset.Fill(0.);
      }

      float total_offset = sqrt(gyro_eye_offset(0) * gyro_eye_offset(0) + gyro_eye_offset(1) + gyro_eye_offset(1));
      if (total_offset < 0.01){
        gyro_eye_offset.Fill(0.);
      } else {
        gyro_eye_offset(0) = max(min(gyro_eye_offset(0), MAX_GYRO_EYE_OFFSET_X), -MAX_GYRO_EYE_OFFSET_X);
        gyro_eye_offset(1) = max(min(gyro_eye_offset(1), MAX_GYRO_EYE_OFFSET_Y), -MAX_GYRO_EYE_OFFSET_Y);
      }

      eye_offset = gyro_eye_offset;
    }

    // Move eye towards target.
    eye_motion_smoother.set_target(nominal_eye_target + eye_offset);
    eye_motion_smoother.update(t);

    if (t - last_draw_t < 1. / 30.)
    {
      return;
    }
    // Actual time-gated 30hz drawing
    last_draw_t = t;
    draw_background();
    auto eye_color = matrix.color565(0x00, 0xFF, 0x00);
    draw_eyes(eye_motion_smoother.get_state(), NOMINAL_EYE_WIDTH, eye_height, NOMINAL_EYE_SPACING, eye_color);
    matrix.show();
  }

  void set_target(const BLA::Matrix<2, 1> &eye_pos)
  {
    nominal_eye_target = eye_pos;
  }

  const BLA::Matrix<2, 1> &get_target()
  {
    return eye_motion_smoother.get_target();
  }

  void set_eye_mode(EyeMode new_eye_mode)
  {
    current_eye_mode = new_eye_mode;
  }

  Adafruit_Protomatter *get_display()
  {
    return &matrix;
  }

private:
  void draw_background()
  {
    matrix.startWrite();
    for (int16_t j = 0; j < DISPLAY_HEIGHT; j++)
    {
      for (int16_t i = 0; i < DISPLAY_WIDTH / 2; i++)
      {
        auto pixel = pgm_read_word(&screen[j * (DISPLAY_WIDTH / 2) + i]);
        // pixel = dim565(pixel, 0.75);

        matrix.writePixel(i, j, pixel);
        // Display is mirrored along centerline
        matrix.writePixel(DISPLAY_WIDTH - i - 1, j, pixel);

        // And again, for display on back
        matrix.writePixel(DISPLAY_WIDTH + i, j, pixel);
        matrix.writePixel(2 * DISPLAY_WIDTH - i - 1, j, pixel);
      }
    }
    matrix.endWrite();
  }

  void draw_eye_normal(const BLA::Matrix<2, 1> &eye_pos, float width, float height, int eye_color)
  {
    int bottom_left_u = floor(DISPLAY_WIDTH * (eye_pos(0) - width / 2.));
    int bottom_left_v = floor(DISPLAY_HEIGHT * (eye_pos(1) - height / 2.));
    int width_u = ceil(DISPLAY_WIDTH * width);
    int width_v = ceil(DISPLAY_HEIGHT * height);
    matrix.fillRect(bottom_left_u, bottom_left_v, width_u, width_v, eye_color);
  }

  void draw_eye_happy(const BLA::Matrix<2, 1> &eye_pos, float width, float height, int eye_color)
  {
    int top_u = floor(DISPLAY_WIDTH * (eye_pos(0)));
    int top_v = floor(DISPLAY_HEIGHT * (eye_pos(1) - height * 0.3));
    int bottom_u = floor(DISPLAY_WIDTH * eye_pos(0));
    int bottom_v = floor(DISPLAY_HEIGHT * eye_pos(1));
    int right_u = floor(DISPLAY_WIDTH * (eye_pos(0) + width * 0.5));
    int right_v = floor(DISPLAY_HEIGHT * (eye_pos(1) + height * 0.1));
    int left_u = floor(DISPLAY_WIDTH * (eye_pos(0) - width * 0.5));
    int left_v = floor(DISPLAY_HEIGHT * (eye_pos(1) + height * 0.1));

    int br_u = floor(DISPLAY_WIDTH * (eye_pos(0) + width * 0.3));
    int br_v = floor(DISPLAY_HEIGHT * (eye_pos(1) + height * 0.4));
    int bl_u = floor(DISPLAY_WIDTH * (eye_pos(0) - width * 0.3));
    int bl_v = floor(DISPLAY_HEIGHT * (eye_pos(1) + height * 0.4));

    matrix.fillTriangle(
        top_u, top_v, right_u, right_v, br_u, br_v, eye_color);
    matrix.fillTriangle(
        top_u, top_v, br_u, br_v, bottom_u, bottom_v, eye_color);

    matrix.fillTriangle(
        top_u, top_v, left_u, left_v, bl_u, bl_v, eye_color);
    matrix.fillTriangle(
        top_u, top_v, bl_u, bl_v, bottom_u, bottom_v, eye_color);
  }

  // Normalized coordinates: bottom-left is 0, 0, top-right is 1., 1., +x is right
  void draw_eye(const BLA::Matrix<2, 1> &eye_pos, float width, float height, int eye_color)
  {
    switch (current_eye_mode)
    {
    case EyeMode::NORMAL:
      draw_eye_normal(eye_pos, width, height, eye_color);
      break;
    case EyeMode::HAPPY:
      draw_eye_happy(eye_pos, width, height, eye_color);
      break;
    default:
      break;
    }
  }

  void draw_eyes(const BLA::Matrix<2, 1> &center_pos, float eye_width, float eye_height, float eye_spacing, int eye_color)
  {
    const BLA::Matrix<2, 1> eye_offset(eye_spacing / 2., 0.);
    draw_eye(
        center_pos - eye_offset,
        eye_width,
        eye_height,
        eye_color);
    draw_eye(
        center_pos + eye_offset,
        eye_width,
        eye_height,
        eye_color);
  }
  BLA::Matrix<2, 1> nominal_eye_target;
  PIDMotionSmoother<2> eye_motion_smoother = PIDMotionSmoother<2>(
      BLA::Matrix<2, 1>(0.5, 1.0),
      BLA::Matrix<2, 1>(0.5, 0.35),
      NOMINAL_EYE_FREQ,
      NOMINAL_EYE_DAMPING_RATIO);
  double last_draw_t = -0.01;
  double last_update_t = -0.01;
  double time_of_next_target_change = 0.0;
  double time_of_next_blink = 0.0;

  bool imu_is_valid;
  EyeMode current_eye_mode = EyeMode::HAPPY;

  // State machine for trying to keep eyes stationary.
  // If we're not moving
  IMUManager m_imu_manager;
  double head_motion_start_t = 0.0;
  bool head_motion_event_active = false;
  double head_motion_second_phase_start_t = 0.;
  double head_motion_last_moving_t = 0.0;
  double head_motion_last_stationary_t = 0.0;
  BLA::Matrix<2, 1> gyro_eye_offset;
};