#pragma once

#include <Adafruit_PWMServoDriver.h>
#include "persistent_config.h"
#include "motion_smoothing.h"

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

const int SERVO_OE_PIN = A3;

void disable_servos()
{
    digitalWrite(SERVO_OE_PIN, HIGH);
}

void enable_servos(

)
{
    digitalWrite(SERVO_OE_PIN, LOW);
}

void setup_pwm()
{
    pwm.begin();
    pwm.setOscillatorFrequency(27000000);
    pwm.setPWMFreq(50);

    pinMode(SERVO_OE_PIN, OUTPUT);
    enable_servos();
}


const float DEFAULT_SERVO_FREQ = 2.5;
const float DEFAULT_SERVO_DAMPING_RATIO = 1.;

class ServoManager
{
public:
    ServoManager(PersistentConfigManager *config, uint8_t servo_index, std::string prefix, float default_closed, float default_open) :  _index(servo_index), _open(default_open), _closed(default_closed), _config(config), _prefix(prefix)
    {
        _open = _config->get_value((prefix + "_C").c_str(), default_closed);
        _closed = _config->get_value((prefix + "_O").c_str(), default_open);

        _motion_smoother = new PIDMotionSmoother<1>(
            BLA::Matrix<1, 1>(_closed),
            BLA::Matrix<1, 1>(_closed),
            _config->get_value("S_FREQ", DEFAULT_SERVO_FREQ),
            _config->get_value("S_DAMP", DEFAULT_SERVO_DAMPING_RATIO));
    }

    void update(double t)
    {
        if (t < _last_settings_update || t - _last_settings_update >= 0.1)
        {
            _last_settings_update = t;
            _motion_smoother->update_damping(
                _config->get_value("S_FREQ", DEFAULT_SERVO_FREQ),
                _config->get_value("S_DAMP", DEFAULT_SERVO_DAMPING_RATIO));
            _closed = _config->get_value((_prefix + "_C").c_str(), _closed);
            _open = _config->get_value((_prefix + "_O").c_str(), _open);
        }
        _motion_smoother->update(t);
 
        float max_range = max(_open, _closed);
        float min_range = min(_open, _closed);

        uint16_t target = _motion_smoother->get_state()(0);
        target = min(target, (uint16_t)max_range);
        target = max(target, (uint16_t)min_range);
        pwm.writeMicroseconds(_index, target);
    }

private:
    uint8_t _index;
    float _open;
    float _closed;
    PIDMotionSmoother<1> *_motion_smoother;
    PersistentConfigManager *_config;
    std::string _prefix;

    double _last_settings_update = 0.0;
};