#pragma once

#include <Adafruit_Protomatter.h>
#include "BasicLinearAlgebra.h"
#include "screen.h"

uint8_t rgbPins[] = {6, A5, A1, A0, A4, 11};
uint8_t addrPins[] = {10, 5, 13, 9};
uint8_t clockPin = 12;
uint8_t latchPin = PIN_SERIAL1_RX;
uint8_t oePin = PIN_SERIAL1_TX;
const int DISPLAY_WIDTH = 64;
const int DISPLAY_HEIGHT = 32;

enum EyeMode {
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


class DisplayManager
{
public:
  DisplayManager()
  {
    // Initialize matrix...
    ProtomatterStatus status = matrix.begin();
    if (status != PROTOMATTER_OK)
    {
      // DO NOT CONTINUE if matrix setup encountered an error.
      for (;;)
        ;
    }
  }

  void update(double t)
  {
    float dt = t - last_display_update_t;
    if (dt < 1. / 30.)
    {
      return;
    }

    last_display_update_t = t;

    // Blink when it's time
    float t_blink = t - time_of_next_blink;
    float blink_duration = 0.1;
    float eye_height = 0.3;
    float eye_width = 0.1;
    float eye_spacing = 0.15;
    if (t_blink <= 0.0)
    {
    }
    else if (t_blink <= blink_duration)
    {
      eye_height *= (blink_duration - t_blink) / blink_duration;
    }
    else if (t_blink <= 2. * blink_duration)
    {
      eye_height *= (t_blink - blink_duration) / blink_duration;
    }
    else
    {
      // Choose when to next blink
      time_of_next_blink = t + rand_range(2.0, 6.0);
    }

    // Move eye towards target.
    eye_motion_smoother.update(t);

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

    draw_background();
    auto eye_color = matrix.color565(0x00, 0xFF, 0x00);

    draw_eyes(eye_motion_smoother.get_state(), eye_width, eye_height, eye_spacing, eye_color);

    matrix.show();
  }

  void set_target(const BLA::Matrix<2, 1> &eye_pos){
    eye_motion_smoother.set_target(eye_pos);
  }
  
  const BLA::Matrix<2, 1>& get_target(){
    return eye_motion_smoother.get_target();
  }


  void set_eye_mode(EyeMode new_eye_mode) {
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
        const auto pixel = pgm_read_word(&screen[j * (DISPLAY_WIDTH / 2) + i]);
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

  void draw_eye_normal(const BLA::Matrix<2, 1> &eye_pos, float width, float height, int eye_color) {
    int bottom_left_u = floor(DISPLAY_WIDTH * (eye_pos(0) - width / 2.));
    int bottom_left_v = floor(DISPLAY_HEIGHT * (eye_pos(1) - height / 2.));
    int width_u = ceil(DISPLAY_WIDTH * width);
    int width_v = ceil(DISPLAY_HEIGHT * height);
    matrix.fillRect(bottom_left_u, bottom_left_v, width_u, width_v, eye_color);
  }

  void draw_eye_happy(const BLA::Matrix<2, 1> &eye_pos, float width, float height, int eye_color) {
    int top_u = floor(DISPLAY_WIDTH * (eye_pos(0)));
    int top_v = floor(DISPLAY_HEIGHT * (eye_pos(1) - height*0.3)); 
    int bottom_u = floor(DISPLAY_WIDTH * eye_pos(0));
    int bottom_v = floor(DISPLAY_HEIGHT * eye_pos(1));
    int right_u = floor(DISPLAY_WIDTH * (eye_pos(0) + width * 0.5));
    int right_v = floor(DISPLAY_HEIGHT * (eye_pos(1) + height*0.1));
    int left_u = floor(DISPLAY_WIDTH * (eye_pos(0) - width * 0.5));
    int left_v = floor(DISPLAY_HEIGHT * (eye_pos(1) + height*0.1));

    int br_u = floor(DISPLAY_WIDTH * (eye_pos(0) + width * 0.3));
    int br_v = floor(DISPLAY_HEIGHT * (eye_pos(1) + height*0.4));
    int bl_u = floor(DISPLAY_WIDTH * (eye_pos(0) - width * 0.3));
    int bl_v = floor(DISPLAY_HEIGHT * (eye_pos(1) + height*0.4));

    
    matrix.fillTriangle(
      top_u, top_v, right_u, right_v, br_u, br_v, eye_color
    );
    matrix.fillTriangle(
      top_u, top_v, br_u, br_v, bottom_u, bottom_v, eye_color
    );
    
    matrix.fillTriangle(
      top_u, top_v, left_u, left_v, bl_u, bl_v, eye_color
    );
    matrix.fillTriangle(
      top_u, top_v, bl_u, bl_v, bottom_u, bottom_v, eye_color
    );
  }

  // Normalized coordinates: bottom-left is 0, 0, top-right is 1., 1., +x is right
  void draw_eye(const BLA::Matrix<2, 1> &eye_pos, float width, float height, int eye_color)
  {
    switch (current_eye_mode){
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
  const float eye_freq = 10.0;
  const float eye_damping_ratio = 0.9;
  PIDMotionSmoother<2> eye_motion_smoother = PIDMotionSmoother<2>(
      BLA::Matrix<2, 1>(0.5, 1.0),
      BLA::Matrix<2, 1>(0.5, 0.35),
      eye_freq,
      eye_damping_ratio);
  double last_display_update_t = -0.01;
  double time_of_next_target_change = 0.0;
  double time_of_next_blink = 0.0;

  EyeMode current_eye_mode = EyeMode::HAPPY;
};