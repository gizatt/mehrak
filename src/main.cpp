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
#include "button_manager.h"

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
DisplayManager *display_manager;

ServoManager *upper_left;
ServoManager *upper_right;
ServoManager *lower_left;
ServoManager *lower_right;

ButtonManager *button_manager;
const int PIN_OPEN = 12;
const int PIN_OPENTOP = 13;
const int PIN_CLOSE = 15;
const int PIN_WAVE = 1;
const int PIN_HAPPY = 2;

Adafruit_NeoPixel strip = Adafruit_NeoPixel(6, A2, NEO_GRB);

class Behavior
{
public:
  Behavior(){};
  virtual void start(double t) = 0;
  virtual bool update(double t) = 0;
};

class IdleBehavior : public Behavior
{
public:
  void start(double t) override
  {
    last_eye_mode_update = t;
  }
  bool update(double t) override
  {
    if (t - last_eye_mode_update > 3.)
    {
      display_manager->set_eye_mode(EyeMode::NORMAL);
      last_eye_mode_update = t;
    }
    return true;
  }

private:
  double last_eye_mode_update;
};
IdleBehavior idle_behavior;

class OpenBehavior : public Behavior
{
public:
  void start(double t) override
  {
  }

  bool update(double t) override
  {
    upper_left->set_target(0.97);
    upper_right->set_target(0.97);
    lower_left->set_target(1.0);
    lower_right->set_target(1.0);
    return true;
  }
};
OpenBehavior open_behavior;

class HappyBehavior : public Behavior
{
public:
  void start(double t) override
  {
  }

  bool update(double t) override
  {
    display_manager->set_eye_mode(EyeMode::HAPPY);
    return true;
  }
};
HappyBehavior happy_behavior;

class OpenTopBehavior : public Behavior
{
public:
  void start(double t) override
  {
  }

  bool update(double t) override
  {
    upper_left->set_target(1.0);
    upper_right->set_target(1.0);
    return true;
  }
};
OpenTopBehavior open_top_behavior;

class CloseBehavior : public Behavior
{
public:
  void start(double t) override
  {
  }

  bool update(double t) override
  {
    upper_left->set_target(0.0);
    upper_right->set_target(0.0);
    lower_left->set_target(0.0);
    lower_right->set_target(0.0);
    return true;
  }
};
CloseBehavior close_behavior;

class WaveBehavior : public Behavior
{
public:
  void start(double t) override
  {
    t_start = t;
    display_manager->set_eye_mode(EyeMode::HAPPY);
  }

  bool update(double t) override
  {
    double t_local = t - t_start;
    double target = 0.3 * cos(2. * 3.1415 * t_local / 2. + 3.1415) + 0.7;

    double fade = min(
      t_local / FADE_IN_DURATION, (WAVE_DURATION - t_local) / FADE_IN_DURATION
    );
    fade = min(fade, 1.);

    upper_left->set_target(target * fade);

    return t_local >= WAVE_DURATION;
  }

  private:
  double t_start;
  const double WAVE_DURATION = 4.;
  const double FADE_IN_DURATION = 0.5;
};
WaveBehavior wave_behavior;

Behavior *current_behavior;

void setup(void)
{
  double t = ((double)millis()) / 1000.;

  // Do BLE / Internal FS requirement as a pre-req for persistent config
  setup_ble_and_internal_fs();

  // Set up actual persistent config
  persistent_config = new PersistentConfigManager("mehrak_config");

  // Do display setup
  display_manager = new DisplayManager();

  // Turn the RGB LEDs green
  strip.begin(); // initialize the strip
  strip.clear(); // Initialize all pixels to 'off'
  strip.setPixelColor(0, 0, 150, 0);
  strip.setPixelColor(2, 0, 150, 0);
  strip.setPixelColor(3, 0, 150, 0);
  strip.setPixelColor(5, 0, 150, 0);
  strip.show();

  delay(500);

  setup_pwm();
  upper_left = new ServoManager(persistent_config, 4, "ul", 640, 1925);
  lower_left = new ServoManager(persistent_config, 5, "ll", 2000, 850);
  upper_right = new ServoManager(persistent_config, 6, "ur", 2070, 800);
  lower_right = new ServoManager(persistent_config, 7, "lr", 780, 1975);

  // Talk to buttons
  button_manager = new ButtonManager({PIN_OPEN, PIN_OPENTOP, PIN_CLOSE, PIN_WAVE, PIN_HAPPY}, t);

  delay(500);
  
  idle_behavior.start(t);
  current_behavior = nullptr;
}

void loop(void)
{
  // Alternate looking places, blinking during movement
  double t = ((double)millis()) / 1000.;

  upper_left->update(t);
  lower_left->update(t);
  upper_right->update(t);
  lower_right->update(t);
  button_manager->update(t);
  display_manager->update(t);

  if (current_behavior != nullptr)
  {
    if (current_behavior->update(t))
    {
      current_behavior = nullptr;
      idle_behavior.start(t);
    }
  }
  else
  {
    idle_behavior.update(t);

    // Try to dispatch new behavior
    if (button_manager->get_button_state(PIN_CLOSE).is_pressed())
    {
      current_behavior = &close_behavior;
    }
    else if (button_manager->get_button_state(PIN_OPENTOP).is_pressed())
    {
      current_behavior = &open_top_behavior;
    }
    else if (button_manager->get_button_state(PIN_WAVE).is_pressed())
    {
      current_behavior = &wave_behavior;
    }
    else if (button_manager->get_button_state(PIN_OPEN).is_pressed())
    {
      current_behavior = &open_behavior;
    }
    else if (button_manager->get_button_state(PIN_HAPPY).is_pressed())
    {
      current_behavior = &happy_behavior;
    }

    if (current_behavior != nullptr)
    {
      current_behavior->start(t);
    }
  }
}