/* ----------------------------------------------------------------------
Mehrak main driver code.

Summary of functionality:
- Manages a simple calibration config using the Adafruit_nRF52_Arduino InternalFileSystem library. This calibration stores the "fully open" and "fully closed" servo settings for each of the 4 corners, plus other miscellaneous style settings like movement speeds.
- Allows the editing of the above through BLE GATT.
- Reads the state of 8 control buttons via an I2C I/O daughter board.
- Displays images on the front and back 32x64 pixel display panels.
------------------------------------------------------------------------- */

#include "persistent_config.h"

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
    DISPLAY_WIDTH * 2,             // Width of matrix (or matrices, if tiled horizontally)
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

BLEDfu  bledfu;  // OTA DFU service
BLEDis  bledis;  // device information
PersistentConfigManager * persistent_config;


void startAdv(void)
{
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);

  // Advertising with only board ID
  struct ATTR_PACKED {
    uint16_t mfr_id;
    
    uint8_t  field_len;
    uint16_t field_key;
    uint16_t field_value;
  } mfr_adv;

  mfr_adv.mfr_id = UUID16_COMPANY_ID_ADAFRUIT;
  mfr_adv.field_len = 4;
  mfr_adv.field_key = 1; // board id
  mfr_adv.field_value = USB_PID;

  Bluefruit.Advertising.addManufacturerData(&mfr_adv, sizeof(mfr_adv));

  // Add name to advertising, since there is enough room
  Bluefruit.Advertising.addName();
  
  /* Start Advertising
   * - Enable auto advertising if disconnected
   * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
   * - Timeout for fast mode is 30 seconds
   * - Start(timeout) with timeout = 0 will advertise forever (until connected)
   * 
   * For recommended advertising interval
   * https://developer.apple.com/library/content/qa/qa1931/_index.html   
   */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds  
}

void setup(void)
{
  InternalFS.begin();
  
  // Config the peripheral connection with maximum bandwidth 
  // more SRAM required by SoftDevice
  // Note: All config***() function must be called before begin()
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
  Bluefruit.begin();
  Bluefruit.setTxPower(8);    // Check bluefruit.h for supported values
  Bluefruit.setName("Mehrak");
  
  // To be consistent OTA DFU should be added first if it exists
  bledfu.begin();

  // Configure and Start Device Information Service
  bledis.setManufacturer("Kara Fjolnir");
  bledis.begin();

  startAdv();
  
  persistent_config = new PersistentConfigManager("mehrak_config");
  

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
  const float minrange = 700;
  const float maxrange = 1900;
  const float midrange = (minrange + maxrange) / 2.;
  int microsec = midrange + (maxrange - midrange) * servo_target;
  for (int k = 0; k < 4; k++){
    pwm.writeMicroseconds(k, microsec);
  }

  // 4-7: Midrange
  // 8-11: Low end
  // 12-15: Top end
  for (int k = 4; k < 8; k++){
    pwm.writeMicroseconds(k, minrange);
  }
  for (int k = 8; k < 12; k++){
    pwm.writeMicroseconds(k, (minrange + maxrange) / 2);
  }
  for (int k = 12; k < 16; k++){
    pwm.writeMicroseconds(k, maxrange);
  }

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

  persistent_config->get_value("/test", 1.23);
}