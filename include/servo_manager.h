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


const float DEFAULT_SERVO_FREQ = 4.0;
const float DEFAULT_SERVO_DAMPING_RATIO = 1.0;

class ServoManager
{
    /**
     *  Operating range [0, 1], where 0 is closed and 1 is open. Starts closed.
    */
public:
    ServoManager(PersistentConfigManager *config, uint8_t servo_index, std::string prefix, float default_closed, float default_open) :  _index(servo_index), _open(default_open), _closed(default_closed), _config(config), _prefix(prefix)
    {
        // _open = _config->get_value((prefix + "_O").c_str(), default_open);
        // _closed = _config->get_value((prefix + "_C").c_str(), default_closed);

        _motion_smoother = new PIDMotionSmoother<1>(
            BLA::Matrix<1, 1>(0.),
            BLA::Matrix<1, 1>(0.),
            DEFAULT_SERVO_FREQ,
            DEFAULT_SERVO_DAMPING_RATIO
        );
            // _config->get_value("S_FREQ", DEFAULT_SERVO_FREQ),
            // _config->get_value("S_DAMP", DEFAULT_SERVO_DAMPING_RATIO));
    }

    void set_target(double target) {
        _target_matrix(0) = max(min(target, 1.), 0.);
        _motion_smoother->set_target(_target_matrix);
    }

    double get_target() const {
        return _motion_smoother->get_state()(0);
    }

    void update(double t)
    {
        // if (t < _last_settings_update || t - _last_settings_update >= 0.1)
        // {
        //     _last_settings_update = t;
        //     _motion_smoother->update_damping(
        //         _config->get_value("S_FREQ", DEFAULT_SERVO_FREQ),
        //         _config->get_value("S_DAMP", DEFAULT_SERVO_DAMPING_RATIO));
        //     _closed = _config->get_value((_prefix + "_C").c_str(), _closed);
        //     _open = _config->get_value((_prefix + "_O").c_str(), _open);
        // }
        _motion_smoother->update(t);

        uint16_t target = _motion_smoother->get_state()(0) * (_open - _closed) + _closed;
        pwm.writeMicroseconds(_index, target);
    }

private:
    uint8_t _index;
    float _open;
    float _closed;
    PIDMotionSmoother<1> *_motion_smoother;
    PersistentConfigManager *_config;
    std::string _prefix;

    BLA::Matrix<1, 1> _target_matrix;
    double _last_settings_update = 0.0;
};