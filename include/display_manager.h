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

Adafruit_Protomatter matrix(
    DISPLAY_WIDTH * 2,             // Width of matrix (or matrices, if tiled horizontally)
    4,                         // Bit depth, 1-6
    1, rgbPins,                // # of matrix chains, array of 6 RGB pins for each
    4, addrPins,               // # of address pins (height is inferred), array of pins
    clockPin, latchPin, oePin, // Other matrix control pins
    true                       // No double-buffering here (see "doublebuffer" example)
);                             // Row tiling: two rows in "serpentine" path

// SETUP - RUNS ONCE AT PROGRAM START --------------------------------------

void setup_display() {
    // Initialize matrix...
  ProtomatterStatus status = matrix.begin();
  Serial.print("Protomatter begin() status: ");
  Serial.println((int)status);
  if (status != PROTOMATTER_OK)
  {
    // DO NOT CONTINUE if matrix setup encountered an error.
    for (;;)
      ;
  }
}

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

// Normalized coordinates: bottom-left is 0, 0, top-right is 1., 1., +x is right
void draw_eye(const BLA::Matrix<2, 1> &eye_pos, float width, float height, int eye_color)
{
  int bottom_left_u = floor(DISPLAY_WIDTH * (eye_pos(0) - width / 2.));
  int bottom_left_v = floor(DISPLAY_HEIGHT * (eye_pos(1) - height / 2.));
  int width_u = ceil(DISPLAY_WIDTH * width);
  int width_v = ceil(DISPLAY_HEIGHT * height);
  matrix.fillRect(bottom_left_u, bottom_left_v, width_u, width_v, eye_color);
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