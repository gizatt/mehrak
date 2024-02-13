/* ----------------------------------------------------------------------
Mehrak main driver code.

Summary of functionality:
- Manages a simple calibration config using the Adafruit_nRF52_Arduino InternalFileSystem library. This calibration stores the "fully open" and "fully closed" servo settings for each of the 4 corners, plus other miscellaneous style settings like movement speeds.
- Allows the editing of the above through BLE GATT.
- Reads the state of 8 control buttons via an I2C I/O daughter board.
- Displays images on the front and back 32x64 pixel display panels.
------------------------------------------------------------------------- */

#include <Wire.h>
#include <Adafruit_NeoPixel.h>


#include "utils.h"
#include "ble_and_fs_helpers.h"
#include "servo_manager.h"
#include "persistent_config.h"
#include "motion_smoothing.h"
#include "display_manager.h"
#include "TCA9555.h"

// PINOUT
//
//  I2c: I/O shield (address 0x27)
//        Left column buttons, top to bottom:  12, 13, 14, 15
//        Right column buttons, top to bottom: 1, 2, 3, 4
//       Servo PWM shield (address 0x40)
//        Upper left: 4
//        Lower left: 5
//        Upper right: 6
//        Lower right: 7
//       MPU 6050 (0x68)
//        mounting: +y up, +x is mehrak "right", +z is out through face.
//
//  Digital I/O:
//       Screens: 0, 1, 5, 6, 9, 10, 11, 12, A0, A1, A4, A5
//       RGBD LEDs: A2
//       Servo OE: A3
//
//

PersistentConfigManager *persistent_config;

ServoManager *upper_left;
ServoManager *upper_right;
ServoManager *lower_left;
ServoManager *lower_right;

Adafruit_NeoPixel strip = Adafruit_NeoPixel(6, A2, NEO_GRB);

TCA9535 TCA(0x27);

void setup(void)
{

  // Do BLE / Internal FS requirement as a pre-req for persistent config
  setup_ble_and_internal_fs();

  // Set up actual persistent config
  persistent_config = new PersistentConfigManager("mehrak_config");

  // Do display setup
  setup_display();

  // Turn the RGB LEDs green
  strip.begin(); // initialize the strip
  strip.clear(); // Initialize all pixels to 'off'
  strip.setPixelColor(0, 0, 200, 0);
  strip.setPixelColor(2, 0, 200, 0);
  strip.setPixelColor(3, 0, 200, 0);
  strip.setPixelColor(5, 0, 200, 0);
  strip.show();

  delay(500);

  setup_pwm();
  upper_left = new ServoManager(persistent_config, 4, "ul", 1500, 1500);
  lower_left = new ServoManager(persistent_config, 5, "ll", 1500, 1500);
  upper_right = new ServoManager(persistent_config, 6, "ur", 1500, 1500);
  lower_right = new ServoManager(persistent_config, 7, "lr", 1500, 1500);

  TCA.begin();

  delay(500);
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

void loop(void)
{
  // Alternate looking places, blinking during movement
  double t = ((double)millis()) / 1000.;
  float dt = t - last_display_update_t;

  upper_left->update(t);
  upper_right->update(t);
  
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

  for (int pin = 0; pin < 16; pin++)
  {
    int val = TCA.read1(pin);
    matrix.drawPixel(DISPLAY_WIDTH + pin / 8, pin % 8, matrix.color565(0., 0., val*255));
  }

  matrix.show();
}