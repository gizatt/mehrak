/* ----------------------------------------------------------------------
"Tiled" Protomatter library example sketch. Demonstrates use of multiple
RGB LED matrices as a single larger drawing surface. This example is
written for two 64x32 matrices (tiled into a 64x64 display) but can be
adapted to others. If using MatrixPortal, larger multi-panel tilings like
this should be powered from a separate 5V DC supply, not the USB port
(this example works OK because the graphics are very minimal).

PLEASE SEE THE "simple" EXAMPLE FOR AN INTRODUCTORY SKETCH.
------------------------------------------------------------------------- */

#include <Adafruit_Protomatter.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <BasicLinearAlgebra.h>

#include "motion_smoothing.h"
#include "screen.h"

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

/* ----------------------------------------------------------------------
The RGB matrix must be wired to VERY SPECIFIC pins, different for each
microcontroller board. This first section sets that up for a number of
supported boards.
------------------------------------------------------------------------- */

uint8_t rgbPins[] = {6, A5, A1, A0, A4, 11};
uint8_t addrPins[] = {10, 5, 13, 9};
uint8_t clockPin = 12;
uint8_t latchPin = PIN_SERIAL1_RX;
uint8_t oePin = PIN_SERIAL1_TX;
const int DISPLAY_WIDTH = 64;
const int DISPLAY_HEIGHT = 32;

/* ----------------------------------------------------------------------
Matrix initialization is explained EXTENSIVELY in "simple" example sketch!
It's very similar here, but we're passing an extra argument to define the
matrix tiling along the vertical axis: -2 means there are two matrices
(or rows of matrices) arranged in a "serpentine" path (the second matrix
is rotated 180 degrees relative to the first, and positioned below).
A positive 2 would indicate a "progressive" path (both matrices are
oriented the same way), but usually requires longer cables.
------------------------------------------------------------------------- */

Adafruit_Protomatter matrix(
    DISPLAY_WIDTH,             // Width of matrix (or matrices, if tiled horizontally)
    4,                         // Bit depth, 1-6
    1, rgbPins,                // # of matrix chains, array of 6 RGB pins for each
    4, addrPins,               // # of address pins (height is inferred), array of pins
    clockPin, latchPin, oePin, // Other matrix control pins
    true                       // No double-buffering here (see "doublebuffer" example)
);                             // Row tiling: two rows in "serpentine" path

// SETUP - RUNS ONCE AT PROGRAM START --------------------------------------

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

void setup(void)
{
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

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);

  delay(1000);

  // Since this program has no animation, all the drawing can be done
  // here in setup() rather than loop(). It's just a few basic shapes
  // that span across the matrices...nothing showy, the goal of this
  // sketch is just to demonstrate tiling basics.

  matrix.drawLine(0, 0, matrix.width() - 1, matrix.height() - 1,
                  matrix.color565(255, 0, 0)); // Red line
  matrix.drawLine(matrix.width() - 1, 0, 0, matrix.height() - 1,
                  matrix.color565(0, 0, 255)); // Blue line
  int radius = min(matrix.width(), matrix.height()) / 2;
  matrix.drawCircle(matrix.width() / 2, matrix.height() / 2, radius,
                    matrix.color565(0, 255, 0)); // Green circle

  // AFTER DRAWING, A show() CALL IS REQUIRED TO UPDATE THE MATRIX!

  matrix.show(); // Copy data to matrix buffers

  delay(1000);
}

inline float rand_range(float min, float max)
{
  float unit = ((float)random()) / RAND_MAX;
  return (max - min) * unit + min;
}

const float eye_freq = 10.0;
const float eye_damping_ratio = 0.9;
PIDMotionSmoother<2> eye_motion_smoother(
    BLA::Matrix<2, 1>(0.5, 0.5),
    BLA::Matrix<2, 1>(0, 0),
    eye_freq,
    eye_damping_ratio);
double last_display_update_t = -0.01;
double time_of_next_target_change = 0.0;
double time_of_next_blink = 0.0;

const float servo_freq = 10.0;
const float servo_damping_ratio = 1.;
PIDMotionSmoother<1> servo_motion_smoother(
    BLA::Matrix<1, 1>(1.),
    BLA::Matrix<1, 1>(1.),
    servo_freq,
    servo_damping_ratio);
double time_of_next_servo_movement = 0.0;

void loop(void)
{
  // Alternate looking places, blinking during movement
  double t = ((double)millis()) / 1000.;
  float dt = t - last_display_update_t;

  servo_motion_smoother.update(t);
  float servo_target = servo_motion_smoother.get_state()(0);
  if (t >= time_of_next_servo_movement)
  {
    servo_motion_smoother.set_target({-servo_target});
    time_of_next_servo_movement = t + 4.0;
  }
  const float minrange = 1400;
  const float maxrange = 2500;
  const float midrange = (minrange + maxrange) / 2.;
  int microsec = midrange + (maxrange - midrange) * servo_target;
  pwm.writeMicroseconds(0, microsec);

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

  // Move target occasionally.
  if (t >= time_of_next_target_change)
  {
    // Choose new target
    time_of_next_target_change = t + rand_range(1.0, 5.0);
    eye_motion_smoother.set_target(
        {rand_range(0.42, 0.58), rand_range(0.1, 0.5)});
    // Force a blink during the move
    time_of_next_blink = t + rand_range(0, 0.2);
  }

  draw_background();
  auto eye_color = matrix.color565(0x00, 0xFF, 0x00);

  draw_eyes(eye_motion_smoother.get_state(), eye_width, eye_height, eye_spacing, eye_color);

  matrix.setCursor(0, 32 - 9);
  matrix.setTextColor(matrix.color565(255, 255, 255));
  matrix.setTextSize(1);
  matrix.printf("%04d", microsec);

  matrix.show();
}